/**
 * @file encoder.c
 * @brief Quadrature encoder speed measurement module
 * 
 * Captures encoder pulses from PB0 (Channel A) and PB1 (Channel B)
 * Computes RPM periodically using pulse count over a fixed sampling window.
 */

#include "encoder.h"
#include "stm32l5xx_hal.h"
#include <string.h>

/* Encoder state */
static volatile uint32_t pulseCount = 0;       // Total pulses received
static volatile uint32_t lastPulseCount = 0;   // Pulse count at last speed update
static volatile uint16_t measuredRPM = 0;      // Measured RPM

/* Variables for edge detection on quadrature channels */
static volatile uint8_t channelA_lastState = 0;
static volatile uint8_t channelB_lastState = 0;

/**
 * @brief Initialize encoder pins and interrupts
 */
void Encoder_Init(void)
{
  /* PB0 and PB1 are configured as EXTI inputs in CubeMX */
  /* Ensure GPIO pins are initialized and external interrupts are enabled */
  pulseCount = 0;
  lastPulseCount = 0;
  measuredRPM = 0;
  
  /* Read initial state of encoder pins */
  channelA_lastState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) & 0x01;
  channelB_lastState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) & 0x01;
}

/**
 * @brief Reset the pulse counter
 */
void Encoder_ResetPulseCount(void)
{
  pulseCount = 0;
  lastPulseCount = 0;
  measuredRPM = 0;
}

/**
 * @brief Get current pulse count
 */
uint32_t Encoder_GetPulseCount(void)
{
  return pulseCount;
}

/**
 * @brief Get measured RPM
 */
uint16_t Encoder_GetRPM(void)
{
  return measuredRPM;
}

/**
 * @brief Update RPM calculation (call periodically, e.g., 50 ms)
 * 
 * This function computes RPM from the pulse count difference since last update.
 * Formula: RPM = (pulse_delta / PPR) * (1000 ms / sample_period_ms) * 60 s/min
 */
void Encoder_Update(void)
{
  uint32_t pulse_delta;
  
  /* Calculate pulse count difference in the sample period */
  pulse_delta = pulseCount - lastPulseCount;
  lastPulseCount = pulseCount;
  
  /* Convert pulse count to RPM
   * pulse_delta pulses in ENCODER_SAMPLE_PERIOD_MS milliseconds
   * RPM = (pulse_delta / PPR_OUTPUT) * (60,000 ms / sample_period_ms)
   * RPM = (pulse_delta * 60,000) / (PPR_OUTPUT * sample_period_ms)
   */
  if (ENCODER_SAMPLE_PERIOD_MS > 0) {
    measuredRPM = (uint16_t)((pulse_delta * 60000UL) / 
                             (ENCODER_PPR_OUTPUT * ENCODER_SAMPLE_PERIOD_MS));
  }
}

/**
 * @brief ISR callback for encoder channel A (PB0)
 * 
 * Called from HAL_GPIO_EXTI_Callback when PB0 changes
 */
void Encoder_ISR_ChannelA(void)
{
  uint8_t a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) & 0x01;
  uint8_t b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) & 0x01;
  
  /* Quadrature encoder logic: count on A rising edge if B != A */
  if (a != channelA_lastState) {
    if (a == b) {
      pulseCount++;  /* Forward (CW) */
    } else {
      if (pulseCount > 0) pulseCount--;  /* Backward (CCW) */
    }
    channelA_lastState = a;
  }
}

/**
 * @brief ISR callback for encoder channel B (PB1)
 * 
 * Called from HAL_GPIO_EXTI_Callback when PB1 changes
 */
void Encoder_ISR_ChannelB(void)
{
  uint8_t a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) & 0x01;
  uint8_t b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) & 0x01;
  
  /* Update B state and check quadrature */
  if (b != channelB_lastState) {
    if (a != b) {
      pulseCount++;  /* Forward (CW) */
    } else {
      if (pulseCount > 0) pulseCount--;  /* Backward (CCW) */
    }
    channelB_lastState = b;
  }
}
