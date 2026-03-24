#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "main.h"

/* Encoder configuration (Lab 4): 374 pulses/output rev per channel; 4x quadrature => 1496 counts/rev */
#define ENCODER_PPR_MOTOR        11
#define ENCODER_GEAR_RATIO       34
#define ENCODER_PPR_OUTPUT_PER_CH (ENCODER_PPR_MOTOR * ENCODER_GEAR_RATIO)
#define ENCODER_COUNTS_PER_OUTPUT_REV  (ENCODER_PPR_OUTPUT_PER_CH * 4U)
#define ENCODER_SAMPLE_PERIOD_MS 50

/* Function prototypes */
void Encoder_Init(void);
void Encoder_ResetPulseCount(void);
uint32_t Encoder_GetPulseCount(void);
uint16_t Encoder_GetRPM(void);
void Encoder_Update(void);
void Encoder_ISR_ChannelA(void);
void Encoder_ISR_ChannelB(void);

#endif // ENCODER_H
