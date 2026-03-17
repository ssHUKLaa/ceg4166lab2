#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "main.h"

/* Motor control configuration */
#define MOTOR_PWM_FREQ_HZ        20000   // 20 kHz PWM frequency
#define MOTOR_PWM_PERIOD_US      50      // 50 us period at 20 kHz
#define MOTOR_MAX_DUTY_PERCENT   100     // 0-100%
#define MOTOR_CONTROLLER_Kp      0.5     // Proportional gain
#define MOTOR_CONTROLLER_DEFAULT_DUTY  30  // Default startup duty %

/* Motor hardware pins (from main.c) */
#define MOTOR_PWM_PORT           GPIOE
#define MOTOR_PWM_PIN            GPIO_PIN_9      // PE9: TIM1_CH1
#define MOTOR_DIR_A1_PORT        GPIOB
#define MOTOR_DIR_A1_PIN         GPIO_PIN_10     // PB10: AI1
#define MOTOR_DIR_B1_PORT        GPIOB
#define MOTOR_DIR_B1_PIN         GPIO_PIN_11     // PB11: AI2
#define MOTOR_STANDBY_PORT       GPIOE
#define MOTOR_STANDBY_PIN        GPIO_PIN_15     // PE15: STB

/* Function prototypes */
void Motor_Init(void);
void Motor_ApplyCommand(MotorCommand cmd);
void Motor_Stop(void);
void Motor_SetPWM(uint16_t duty_percent, MotorState direction);
MotorCommand Motor_GetCurrentCommand(void);

/* Closed-loop control */
uint16_t Motor_ComputePWM_P(int32_t error);
void Motor_UpdateControl(uint16_t target_rpm, uint16_t measured_rpm);

#endif // MOTOR_H
