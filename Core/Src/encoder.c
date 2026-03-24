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
static volatile uint32_t pulseCount = 0;
static volatile uint32_t lastPulseCount = 0;
static volatile uint16_t measuredRPM = 0;
static volatile uint8_t encQuadState = 0;

/* Gray-code quadrature transition: +1 CW, -1 CCW, 0 bounce or no change */
static const int8_t encoderQuadLut[16] = {
  0, +1, -1, 0,
  -1, 0, 0, +1,
  +1, 0, 0, -1,
  0, -1, +1, 0
};

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
  
  {
    uint8_t a = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
    encQuadState = (uint8_t)((a << 1) | b);
  }
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
  
  /* Snapshot the volatile counter exactly once to avoid ISR race between
   * the delta read and the lastPulseCount write. */
  uint32_t current = pulseCount;
  pulse_delta = current - lastPulseCount;
  lastPulseCount = current;
  
  /* Convert pulse count to RPM
   * pulse_delta pulses in ENCODER_SAMPLE_PERIOD_MS milliseconds
   * RPM = (pulse_delta / PPR_OUTPUT) * (60,000 ms / sample_period_ms)
   * RPM = (pulse_delta * 60,000) / (PPR_OUTPUT * sample_period_ms)
   */
  if (ENCODER_SAMPLE_PERIOD_MS > 0) {
    /* Sanity cap: at 350 RPM max, counts per 50 ms < 750. A delta larger than
     * 3x that (2250) indicates a corrupted sample — discard it silently. */
    if (pulse_delta > 2250UL) {
      pulse_delta = 0;
    }
    measuredRPM = (uint16_t)((pulse_delta * 60000UL) /
                             (ENCODER_COUNTS_PER_OUTPUT_REV * ENCODER_SAMPLE_PERIOD_MS));
  }
}

static void Encoder_OnQuadratureEdge(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t curr = (uint8_t)((a << 1) | b);
  uint8_t idx = (uint8_t)((encQuadState << 2) | curr);
  int8_t step = encoderQuadLut[idx];
  encQuadState = curr;
  if (step > 0) {
    pulseCount++;
  } else if (step < 0 && pulseCount > 0U) {
    pulseCount--;
  }
}

void Encoder_ISR_ChannelA(void)
{
  Encoder_OnQuadratureEdge();
}

void Encoder_ISR_ChannelB(void)
{
  Encoder_OnQuadratureEdge();
}
