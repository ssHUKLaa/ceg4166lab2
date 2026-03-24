#include "i2c2_bus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t s_i2c2_mutex;

void I2c2Bus_Init(void)
{
  if (s_i2c2_mutex == NULL) {
    s_i2c2_mutex = xSemaphoreCreateRecursiveMutex();
  }
}

void I2c2Bus_Lock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return;
  }
  if (s_i2c2_mutex != NULL) {
    (void)xSemaphoreTakeRecursive(s_i2c2_mutex, portMAX_DELAY);
  }
}

void I2c2Bus_Unlock(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return;
  }
  if (s_i2c2_mutex != NULL) {
    (void)xSemaphoreGiveRecursive(s_i2c2_mutex);
  }
}
