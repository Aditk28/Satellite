#pragma once
#include <Arduino.h>

/*
  Phase 5.2 — the I2C bus mutex.

  Two tasks now touch Wire: controlTask (MPU6050, 200 Hz) and safetyTask (INA219,
  20 Hz). Arduino's Wire is not reentrant -- concurrent transactions corrupt each
  other and can leave the bus hung with SDA held low.

  WHY A MUTEX AND NOT A BINARY SEMAPHORE. Both give mutual exclusion; only a mutex
  has an OWNER, and ownership is what enables PRIORITY INHERITANCE. The failure it
  prevents here: telemTask (prio 1) holds the bus, controlTask (3) blocks on it,
  then safetyTask (2) wakes and preempts telem. Control -- the highest-priority
  waiter -- is now stuck behind an unrelated priority-2 task. That is PRIORITY
  INVERSION, the bug that reset Mars Pathfinder on the surface in 1997. A mutex
  temporarily boosts the holder to the waiter's priority so it finishes and
  releases. A semaphore has no owner, so it cannot do this.

  ALWAYS USE A TIMEOUT IN THE CONTROL PATH, NEVER portMAX_DELAY. This bus has
  already proven marginal on this hardware (the MPU read went 2373 -> 2510 us after
  the wire rework). A control task blocked forever on a wedged peripheral, with a
  flywheel spinning, is precisely how a runaway happens. On timeout the caller must
  DEGRADE -- reuse the previous sample, count the fault -- and let safetyTask decide.

  HOLD IT ONLY ACROSS THE TRANSACTION. take -> transfer -> give. Never across a
  computation: long holds mean long inversion windows.

  NEVER FROM AN ISR. There is no FromISR variant of a mutex take, by design --
  priority inheritance is meaningless when there is no task to boost.
*/

// Create the mutex. Call once, pre-scheduler, before any task can use the bus.
void i2c_init(void);

// Take the bus. Returns false on timeout -- caller MUST degrade, not retry forever.
bool i2c_lock(uint32_t timeoutMs);

// Release. Only ever call after a successful i2c_lock().
void i2c_unlock(void);

// Count of failed locks (bus contention or a wedged transaction). Shown in G.
uint32_t i2c_timeouts(void);
