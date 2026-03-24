/**
 * @file motor.c
 * @brief Motor control module with closed-loop PWM regulation
 * 
 * Handles motor direction, PWM duty cycle, and proportional speed controller.
 */

#include "motor.h"
#include "stm32l5xx_hal.h"

/* Motor state variables */
static MotorCommand currentCommand = { MOTOR_STOP, 0 };
static ControllerState controlState = { 0, 0 };
static int32_t controllerIntegral = 0;  /* PI integral accumulator */

/* External HAL handles (defined in main.c/generated) */
extern TIM_HandleTypeDef htim1;

/**
 * @brief Initialize motor hardware
 */
void Motor_Init(void)
{
  /* One path: TB6612 off, PWM 0, directions idle (HAL_TIM_PWM_Start is idempotent if main() already started TIM) */
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  MotorCommand stop = { MOTOR_STOP, 0 };
  Motor_ApplyCommand(stop);
  controlState.error = 0;
  controlState.duty_percent = 0;
}

/**
 * @brief Apply a motor command (direction + duty cycle)
 */
void Motor_ApplyCommand(MotorCommand cmd)
{
  /* Validate duty cycle */
  if (cmd.duty_percent > MOTOR_MAX_DUTY_PERCENT) {
    cmd.duty_percent = MOTOR_MAX_DUTY_PERCENT;
  }
  
  /* Update hardware */
  Motor_SetPWM(cmd.duty_percent, cmd.state);
  
  /* Update state */
  currentCommand = cmd;
}

/**
 * @brief Stop the motor immediately
 */
void Motor_Stop(void)
{
  MotorCommand stopCmd = { MOTOR_STOP, 0 };
  Motor_ApplyCommand(stopCmd);
}

/**
 * @brief Set PWM duty cycle and direction
 * 
 * @param duty_percent  PWM duty cycle (0-100%)
 * @param direction     Motor direction (STOP, CW, CCW)
 */
void Motor_SetPWM(uint16_t duty_percent, MotorState direction)
{
  uint32_t pwm_level;

  if (direction == MOTOR_STOP) {
    duty_percent = 0;
  }
  
  /* Clamp duty cycle */
  if (duty_percent > 100) {
    duty_percent = 100;
  }
  
  /* Edge-aligned PWM1: duty = CCR / (ARR + 1) */
  {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    pwm_level = ((arr + 1U) * (uint32_t)duty_percent) / 100U;
    if (pwm_level > arr) {
      pwm_level = arr;
    }
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_level);
  
  /* Set motor direction and TB6612 STBY (high = driver on, only when running) */
  switch (direction) {
    case MOTOR_CW:
      /* Clockwise: A1=1, A2=0 */
      HAL_GPIO_WritePin(MOTOR_STANDBY_PORT, MOTOR_STANDBY_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_RESET);
      break;
      
    case MOTOR_CCW:
      /* Counter-clockwise: A1=0, A2=1 */
      HAL_GPIO_WritePin(MOTOR_STANDBY_PORT, MOTOR_STANDBY_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_SET);
      break;
      
    case MOTOR_STOP:
    default:
      /* STOP: driver off, both inputs low, PWM 0 */
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_STANDBY_PORT, MOTOR_STANDBY_PIN, GPIO_PIN_RESET);
      break;
  }
}

/**
 * @brief Get the current motor command
 */
MotorCommand Motor_GetCurrentCommand(void)
{
  return currentCommand;
}

/**
 * @brief PI controller: compute PWM from speed error.
 *        Call once per control period (50 ms) when target > 0.
 *        Call Motor_ResetController() whenever the target changes.
 *
 * @param error  Speed error (target_rpm - measured_rpm)
 * @return       Computed PWM duty cycle (MOTOR_CONTROLLER_MIN_DUTY_RUNNING..100%)
 */
uint16_t Motor_ComputePWM_P(int32_t error)
{
  /* Accumulate integral — only allow positive accumulation to prevent reverse
   * braking; clamp to anti-windup limit. */
  controllerIntegral += error;
  if (controllerIntegral > MOTOR_CONTROLLER_INTEGRAL_LIMIT) {
    controllerIntegral = MOTOR_CONTROLLER_INTEGRAL_LIMIT;
  }
  if (controllerIntegral < 0) {
    controllerIntegral = 0;
  }

  int32_t pwm_out = (int32_t)(MOTOR_CONTROLLER_Kp * (float)error)
                  + (int32_t)(MOTOR_CONTROLLER_Ki * (float)controllerIntegral);

  /* Enforce minimum duty when running to prevent TB6612 SHORT-BRAKE (duty=0
   * with direction pins active brakes the motor hard instead of coasting). */
  if (pwm_out < MOTOR_CONTROLLER_MIN_DUTY_RUNNING) {
    pwm_out = MOTOR_CONTROLLER_MIN_DUTY_RUNNING;
  }
  if (pwm_out > MOTOR_MAX_DUTY_PERCENT) {
    pwm_out = MOTOR_MAX_DUTY_PERCENT;
  }

  return (uint16_t)pwm_out;
}

/**
 * @brief Reset the PI integral accumulator.
 *        Must be called whenever the speed target changes to avoid
 *        integral carryover causing overshoot on the new setpoint.
 */
void Motor_ResetController(void)
{
  controllerIntegral = 0;
}

/**
 * @brief Update motor control (to be called from control task)
 * 
 * @param target_rpm    Target speed
 * @param measured_rpm  Measured speed from encoder
 */
void Motor_UpdateControl(uint16_t target_rpm, uint16_t measured_rpm)
{
  MotorCommand cmd;
  
  /* Compute speed error */
  controlState.error = (int32_t)target_rpm - (int32_t)measured_rpm;
  
  /* Apply proportional controller */
  controlState.duty_percent = Motor_ComputePWM_P(controlState.error);
  
  /* Determine direction */
  if (target_rpm == 0) {
    cmd.state = MOTOR_STOP;
    cmd.duty_percent = 0;
  } else {
    cmd.state = MOTOR_CW;  /* Default to CW for positive target */
    cmd.duty_percent = controlState.duty_percent;
  }
  
  /* Apply the command */
  Motor_ApplyCommand(cmd);
}
