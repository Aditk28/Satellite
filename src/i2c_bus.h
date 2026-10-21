#pragma once
#include <Arduino.h>

/*
  I2C bus mutex. Wire is not reentrant and is shared by controlTask (MPU6050) and
  safetyTask (INA219). A mutex rather than a binary semaphore so the holder gets
  priority inheritance. Hold it only across one transaction, always use a timeout
  in the control path, and never take it from an ISR.
*/

// Create the mutex. Call once, pre-scheduler.
void i2c_init(void);

// Take the bus. Returns false on timeout; the caller should degrade (reuse the
// previous sample) rather than retry.
bool i2c_lock(uint32_t timeoutMs);

// Only call after a successful i2c_lock().
void i2c_unlock(void);

// Failed locks, shown in G.
uint32_t i2c_timeouts(void);
