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

/* External HAL handles (defined in main.c/generated) */
extern TIM_HandleTypeDef htim1;

/**
 * @brief Initialize motor hardware
 */
void Motor_Init(void)
{
  /* Initialize motor pins */
  /* Direction pins: set to STOP (both low) */
  HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_RESET);
  
  /* Standby: enable motor driver (active high) */
  HAL_GPIO_WritePin(MOTOR_STANDBY_PORT, MOTOR_STANDBY_PIN, GPIO_PIN_SET);
  
  /* Start PWM on TIM1_CH1 (PE9) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  /* Set initial duty to 0% */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  
  currentCommand.state = MOTOR_STOP;
  currentCommand.duty_percent = 0;
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
  
  /* Clamp duty cycle */
  if (duty_percent > 100) {
    duty_percent = 100;
  }
  
  /* Compute PWM compare level from duty percentage */
  /* Assuming TIM1 is configured with appropriate period */
  /* PWM_level = (duty_percent / 100) * ARR */
  pwm_level = (__HAL_TIM_GET_AUTORELOAD(&htim1) * duty_percent) / 100;
  
  /* Set PWM compare register */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_level);
  
  /* Set motor direction */
  switch (direction) {
    case MOTOR_CW:
      /* Clockwise: A1=1, A2=0 */
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_RESET);
      break;
      
    case MOTOR_CCW:
      /* Counter-clockwise: A1=0, A2=1 */
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_SET);
      break;
      
    case MOTOR_STOP:
    default:
      /* STOP: both low */
      HAL_GPIO_WritePin(MOTOR_DIR_A1_PORT, MOTOR_DIR_A1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_DIR_B1_PORT, MOTOR_DIR_B1_PIN, GPIO_PIN_RESET);
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
 * @brief Proportional controller: compute PWM from speed error
 * 
 * @param error  Speed error (target_rpm - measured_rpm)
 * @return       Computed PWM duty cycle (0-100%)
 */
uint16_t Motor_ComputePWM_P(int32_t error)
{
  int32_t pwm_out;
  
  /* Proportional control: PWM = Kp * error + base_pwm */
  pwm_out = (int32_t)(MOTOR_CONTROLLER_Kp * (float)error) + MOTOR_CONTROLLER_DEFAULT_DUTY;
  
  /* Clamp output to valid range */
  if (pwm_out < 0) {
    pwm_out = 0;
  } else if (pwm_out > MOTOR_MAX_DUTY_PERCENT) {
    pwm_out = MOTOR_MAX_DUTY_PERCENT;
  }
  
  return (uint16_t)pwm_out;
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
