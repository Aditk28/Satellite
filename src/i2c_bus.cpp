#include "i2c_bus.h"
#include <STM32FreeRTOS.h>
#include <task.h>
#include <semphr.h>

static SemaphoreHandle_t s_mtx      = nullptr;
static volatile uint32_t s_timeouts = 0;

void i2c_init(void) {
  // xSemaphoreCreateMutex, NOT xSemaphoreCreateBinary -- see the header. The
  // ownership this creates is what gives us priority inheritance.
  s_mtx = xSemaphoreCreateMutex();
  configASSERT(s_mtx != nullptr);
}

bool i2c_lock(uint32_t timeoutMs) {
  if (!s_mtx) return true;            // pre-init (hwSetup): no other user yet
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) return true;
  s_timeouts++;
  return false;
}

void i2c_unlock(void) {
  if (s_mtx) xSemaphoreGive(s_mtx);
}

uint32_t i2c_timeouts(void) { return s_timeouts; }
