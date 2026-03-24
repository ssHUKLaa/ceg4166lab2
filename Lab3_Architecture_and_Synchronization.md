# Lab 3 Architecture and Synchronization (FreeRTOS)

## Updated Architecture Diagram

```text
+-------------------- ISR Layer --------------------+
| PC13 EXTI (Blue Button)      PB6 EXTI (Touch IRQ) |
|    |                               |              |
|    +----------- xQueueSendFromISR--+              |
+-----------------------|---------------------------+
                        v
              +---------------------+
              | Control/State Task  |  (Task2, Normal)
              | - Central state FSM |
              | - Reads touch via I2C2 after PB6 IRQ |
              | - Applies keypad mapping 0/1/2       |
              | - Multi-touch priority STOP>CCW>CW   |
              | - Starts non-blocking buzzer beep    |
              | - Updates LCD (only task allowed)    |
              +----------|---------------+
                         | motor mailbox (queue len 1)
                         v
              +---------------------+
              | Motor Task          |  (Task1, High, 10 ms)
              | - Only task that sets:
              |   * PWM (TIM1 CH1)
              |   * Direction GPIOs (PB10/PB11)
              |   * STB (PE15)
              |   * LEDs (PA9/PB7/PC7)
              | - Sends applied-state ack to Control
              +---------------------+

              +---------------------+
              | ADC Task            |  (Task3, Low, 50 ms)
              | - Reads potentiometer (ADC1 PC0)
              | - Sends duty updates to Control task
              +---------------------+
```

## Synchronization Explanation (Short)

- ISRs do **not** change motor state directly because they must stay short, non-blocking, and must not touch I2C/LCD/motor outputs.
- ISRs only enqueue events (`xQueueSendFromISR`) to the Control/State task.
- The **Motor Task has highest priority** because it is the only task allowed to control PWM/direction outputs and it must enforce safe STOP behavior deterministically.
- Determinism is preserved by:
  - centralized state machine in one task,
  - ISR-to-task event queueing,
  - single-writer motor hardware policy (Motor Task only),
  - fixed Motor Task period (10 ms),
  - deterministic multi-touch priority (STOP > CCW > CW).

## Hardware-Specific Note (Touch Keypad)

The project currently includes touch keypad protocol autodetection for:
- `AT42QT1070` (default address `0x1B`)
- `MPR121` (default address `0x5A`)

If your keypad uses a different controller/address/register map, update the touch macros/logic in `freeRTOS_first/Core/Src/main.c`.
