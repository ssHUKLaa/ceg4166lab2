/**
 * @file ui.c
 * @brief User interface module: LCD display, keypad input, and buzzer control
 */

#include "ui.h"
#include "lcd.h"
#include "stm32l5xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

/* Speed presets */
const SpeedPresets speedPresets = {
  .low_rpm = SPEED_PRESET_LOW,
  .medium_rpm = SPEED_PRESET_MEDIUM,
  .high_rpm = SPEED_PRESET_HIGH
};

/* Buzzer state machine */
typedef struct {
  uint8_t total_beeps;      /* Total beeps to generate */
  uint8_t beeps_done;       /* Beeps completed */
  uint32_t beep_timer_ms;   /* Beep duration timer */
  enum {
    BUZZER_IDLE,
    BUZZER_ON,
    BUZZER_GAP
  } state;
} BuzzerState;

static BuzzerState buzzer = {
  .total_beeps = 0,
  .beeps_done = 0,
  .beep_timer_ms = 0,
  .state = BUZZER_IDLE
};

/* Last displayed values to avoid redundant LCD updates */
static uint16_t lastDisplayedTarget = 0xFFFF;
static uint16_t lastDisplayedMeasured = 0xFFFF;

/**
 * @brief Initialize UI subsystems (LCD, buzzer pin)
 */
void UI_Init(void)
{
  /* Initialize buzzer pin as output */
  /* (Pin should be configured in CubeMX GPIO setup) */
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  
  /* Initialize LCD (already done in main.c, but safe to call again) */
  LCD_Init();
  
  /* Display initial state */
  LCD_DisplayText("Target: STOP    ", FIRST_LINE);
  LCD_DisplayText("Measured: 0 RPM ", SECOND_LINE);
  
  buzzer.state = BUZZER_IDLE;
}

/**
 * @brief Get keypad event from UI event queue
 * 
 * This function checks if a keypad event is available and returns it.
 * Should be called from the UI task.
 */
UIEvent UI_GetKeypadEvent(void)
{
  UIEvent event = { UI_EVENT_NONE, 0 };
  
  /* Try to receive from UI event queue (non-blocking) */
  if (uiEventQueue != NULL) {
    if (xQueueReceive(uiEventQueue, &event, 0) == pdTRUE) {
      return event;
    }
  }
  
  return event;
}

/**
 * @brief Display target and measured speed on LCD
 * 
 * @param target_rpm     Target speed in RPM
 * @param measured_rpm   Measured speed in RPM
 */
void UI_DisplaySpeed(uint16_t target_rpm, uint16_t measured_rpm)
{
  static char line1[20];
  static char line2[20];
  
  /* Only update LCD if values changed (avoid flicker) */
  if (target_rpm != lastDisplayedTarget) {
    if (target_rpm == 0) {
      snprintf(line1, sizeof(line1), "Target: STOP    ");
    } else {
      snprintf(line1, sizeof(line1), "Target:%4u RPM ", target_rpm);
    }
    LCD_DisplayText(line1, FIRST_LINE);
    lastDisplayedTarget = target_rpm;
  }
  
  if (measured_rpm != lastDisplayedMeasured) {
    snprintf(line2, sizeof(line2), "Measured:%3u RPM", measured_rpm);
    LCD_DisplayText(line2, SECOND_LINE);
    lastDisplayedMeasured = measured_rpm;
  }
}

/**
 * @brief Display motor state on LCD
 * 
 * @param state  Motor state (MOTOR_STOP, MOTOR_CW, MOTOR_CCW)
 */
void UI_DisplayState(MotorState state)
{
  static char line[20];
  
  switch (state) {
    case MOTOR_STOP:
      snprintf(line, sizeof(line), "State: STOP     ");
      break;
    case MOTOR_CW:
      snprintf(line, sizeof(line), "State: CW       ");
      break;
    case MOTOR_CCW:
      snprintf(line, sizeof(line), "State: CCW      ");
      break;
    default:
      snprintf(line, sizeof(line), "State: Unknown  ");
      break;
  }
  
  LCD_DisplayText(line, FIRST_LINE);
}

/**
 * @brief Trigger buzzer with specified number of beeps
 * 
 * @param beep_count  Number of beeps (1-3)
 */
void UI_TriggerBuzzer(uint8_t beep_count)
{
  if (beep_count > 0) {
    buzzer.total_beeps = beep_count;
    buzzer.beeps_done = 0;
    buzzer.beep_timer_ms = BUZZER_BEEP_MS;
    buzzer.state = BUZZER_ON;
    
    /* Start buzzing immediately */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
  }
}

/**
 * @brief Process buzzer state machine (call periodically, e.g., every 10 ms)
 * 
 * This implements the beep timing without blocking.
 * Should be called from a periodic task.
 */
void UI_ProcessBuzzer(void)
{
  switch (buzzer.state) {
    case BUZZER_IDLE:
      /* Nothing to do */
      break;
      
    case BUZZER_ON:
      /* Count down beep duration */
      if (buzzer.beep_timer_ms >= 10) {
        buzzer.beep_timer_ms -= 10;
      } else {
        /* End of beep */
        buzzer.beep_timer_ms = BUZZER_GAP_MS;
        buzzer.beeps_done++;
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        
        /* Check if all beeps done */
        if (buzzer.beeps_done >= buzzer.total_beeps) {
          buzzer.state = BUZZER_IDLE;
        } else {
          buzzer.state = BUZZER_GAP;
        }
      }
      break;
      
    case BUZZER_GAP:
      /* Count down gap between beeps */
      if (buzzer.beep_timer_ms >= 10) {
        buzzer.beep_timer_ms -= 10;
      } else {
        /* Start next beep */
        buzzer.beep_timer_ms = BUZZER_BEEP_MS;
        buzzer.state = BUZZER_ON;
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
      }
      break;
  }
}

/**
 * @brief Handle keypad interrupt (called from ISR)
 * 
 * This function should be called from the touch IRQ handler.
 * It reads the keypad and queues events for the UI task to process.
 */
void UI_HandleKeypadISR(void)
{
  /* This would typically read the I2C keypad device
   * and queue appropriate UIEvent to uiEventQueue.
   * 
   * Placeholder: actual implementation depends on keypad driver
   * (AT42QT1070 or MPR121 as detected in main.c)
   */
}
