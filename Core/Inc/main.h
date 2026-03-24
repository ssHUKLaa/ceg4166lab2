/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Motor control types */
typedef enum {
  MOTOR_STOP = 0,
  MOTOR_CW = 1,
  MOTOR_CCW = 2
} MotorState;

typedef struct {
  MotorState state;
  uint16_t duty_percent;  // 0-100%
} MotorCommand;

typedef struct {
  int32_t error;          // Speed error (target - measured)
  uint16_t duty_percent;  // Computed PWM duty
} ControllerState;

/* Encoder and speed measurement */
typedef struct {
  uint32_t pulse_count;    // Total encoder pulses received
  uint16_t rpm;            // Current measured RPM
} EncoderData;

/* UI event types */
typedef enum {
  UI_EVENT_KEY_0 = 0,      // STOP
  UI_EVENT_KEY_1 = 1,      // Low speed preset
  UI_EVENT_KEY_2 = 2,      // Medium speed preset
  UI_EVENT_KEY_3 = 3,      // High speed preset
  UI_EVENT_NONE = 255
} UIEventType;

typedef struct {
  UIEventType type;
  uint16_t value;
} UIEvent;

/* Speed control presets */
typedef struct {
  uint16_t low_rpm;        // Low speed preset  (e.g., 100 RPM)
  uint16_t medium_rpm;     // Medium speed preset (e.g., 175 RPM)
  uint16_t high_rpm;       // High speed preset (e.g., 280 RPM)
} SpeedPresets;

/* Control event types for inter-task communication */
typedef enum {
  CONTROL_EVT_BUTTON_PRESS = 0,    // Button press event
  CONTROL_EVT_TOUCH_IRQ = 1,       // Touch/keypad interrupt
  CONTROL_EVT_NONE = 255
} ControlEventType;

typedef struct {
  ControlEventType type;
  uint16_t value;
} ControlEvent;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Task timing constants (in milliseconds) */
#define MOTOR_TASK_PERIOD_MS     10    /* Task1 - Motor control, 10 ms */
#define CONTROL_TASK_PERIOD_MS   50    /* Task2 - Control/Encoder, 50 ms */
#define TOUCH_DEBOUNCE_MS        20    /* Task3 - Touch debouncing, 20 ms */

/* Control event queue configuration */
#define CONTROL_EVENT_QUEUE_LENGTH 64

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* Encoder functions */
void Encoder_Init(void);
void Encoder_ResetPulseCount(void);
uint32_t Encoder_GetPulseCount(void);
uint16_t Encoder_GetRPM(void);
void Encoder_Update(void);  // Called periodically to compute RPM
void Encoder_ISR_ChannelA(void);  // ISR callback for PB0 (Channel A)
void Encoder_ISR_ChannelB(void);  // ISR callback for PB1 (Channel B)

/* Motor functions */
void Motor_Init(void);
void Motor_ApplyCommand(MotorCommand cmd);
void Motor_Stop(void);
void Motor_SetPWM(uint16_t duty_percent, MotorState direction);
MotorCommand Motor_GetCurrentCommand(void);

/* UI functions */
void UI_Init(void);
UIEvent UI_GetKeypadEvent(void);
void UI_DisplaySpeed(uint16_t target_rpm, uint16_t measured_rpm);
void UI_DisplayState(MotorState state);
void UI_TriggerBuzzer(uint8_t beep_count);

/* Extern variables for RTOS queues */
extern QueueHandle_t motorCmdQueue;
extern QueueHandle_t motorAppliedQueue;
extern QueueHandle_t uiEventQueue;

/* Speed presets */
extern const SpeedPresets speedPresets;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
