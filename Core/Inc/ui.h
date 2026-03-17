#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "stm32l5xx_hal.h"
#include "main.h"

/* Speed preset definitions */
#define SPEED_PRESET_LOW      100    // Low speed: 100 RPM
#define SPEED_PRESET_MEDIUM   175    // Medium speed: 175 RPM
#define SPEED_PRESET_HIGH     280    // High speed: 280 RPM

/* Buzzer configuration */
#define BUZZER_PORT           GPIOA
#define BUZZER_PIN            GPIO_PIN_0   // PA0
#define BUZZER_BEEP_MS        50            // Beep duration
#define BUZZER_GAP_MS         100           // Gap between beeps

/* Function prototypes */
void UI_Init(void);
UIEvent UI_GetKeypadEvent(void);
void UI_DisplaySpeed(uint16_t target_rpm, uint16_t measured_rpm);
void UI_DisplayState(MotorState state);
void UI_TriggerBuzzer(uint8_t beep_count);
void UI_ProcessBuzzer(void);  // Call periodically (e.g., 10 ms)

/* Touch/keypad event handler */
void UI_HandleKeypadISR(void);

#endif // UI_H
