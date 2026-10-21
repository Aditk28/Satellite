#pragma once
/* The single stop path and the boot black box. faults_safeStop() kills the fans,
   then the motor driver, runs the optional graceful stop, latches the reason in a
   .noinit record that survives a warm reset, and blinks the reason code forever.
   C-compatible and dependency-free because FreeRTOSConfig.h (included by the
   kernel's C sources) names rtAssertFail and rtRunTimeCounter. */

#include <stdint.h>

/* The value is also the LED blink count. */
typedef enum {
  FAULT_NONE = 0,
  FAULT_ASSERT,             /* 1  configASSERT tripped */
  FAULT_STACK_OVERFLOW,     /* 2 */
  FAULT_MALLOC,             /* 3  pvPortMalloc / newlib malloc returned NULL */
  FAULT_SCHEDULER_RETURNED, /* 4  vTaskStartScheduler returned (heap too small) */
  FAULT_WHEEL_SAT,          /* 5  wheel speed over the saturation limit */
  FAULT_HEARTBEAT = 8       /* 8  control task stopped kicking the safety task */
} fault_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Call in setup() before motor bring-up so a fault during initFOC still kills the
   driver. hwEnablePin is the active-high driver enable (-1 to skip). ledPin is
   blinked after a fault (-1 for none); on the F446RE that is PA5, which is also
   motor phase A, which is fine because the driver is dead by then. stopHook is an
   optional graceful stop run after the hardware kill. */
void faults_init(int hwEnablePin, int ledPin, void (*stopHook)(void));

/* Kill hook run first on every fault path, before the motor enable is pulled low.
   Used for the fans, which are the bigger hazard with unguarded props. It runs with
   interrupts masked, so it must not use FreeRTOS, locks or unbounded waits. */
void faults_setHwKillHook(void (*fn)(void));

/* Never returns. */
void faults_safeStop(fault_t reason);

/* Print and clear the record left by the previous boot's fault, if any. */
void faults_reportLastBoot(void);

/* configASSERT hook. Latches file/line, then stops. */
void rtAssertFail(const char* file, int line);

/* configGENERATE_RUN_TIME_STATS counter. Returns TIM5->CNT. */
uint32_t rtRunTimeCounter(void);

#ifdef __cplusplus
}
#endif
