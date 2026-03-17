#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "main.h"

/* Encoder configuration */
#define ENCODER_PPR_MOTOR        11     // Pulses per revolution at motor shaft
#define ENCODER_GEAR_RATIO       34     // Gear ratio
#define ENCODER_PPR_OUTPUT       (ENCODER_PPR_MOTOR * ENCODER_GEAR_RATIO)  // 374 PPR at output
#define ENCODER_SAMPLE_PERIOD_MS 50     // Sample speed every 50 ms

/* Function prototypes */
void Encoder_Init(void);
void Encoder_ResetPulseCount(void);
uint32_t Encoder_GetPulseCount(void);
uint16_t Encoder_GetRPM(void);
void Encoder_Update(void);
void Encoder_ISR_ChannelA(void);
void Encoder_ISR_ChannelB(void);

/* For interrupt handlers (called from HAL_GPIO_EXTI_Callback) */
void HAL_EXTI_Callback_Encoder(uint16_t GPIO_Pin);

#endif // ENCODER_H
