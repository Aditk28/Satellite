#pragma once
/* ============================================================================
   faults.h  —  the one stop path + the boot black box (Step 1.2)
   ----------------------------------------------------------------------------
   Every failure (kernel assert, stack overflow, malloc fail, scheduler return,
   wheel saturation, ...) funnels through faults_safeStop(), which ALWAYS, in
   order: (1) kills the motor at the hardware level, (2) runs the graceful
   SimpleFOC stop if one was registered, (3) latches a reason code to a .noinit
   record that survives a warm reset, (4) blinks the reason on an LED forever.

   This header is deliberately SimpleFOC-free and C-compatible: FreeRTOSConfig.h
   (a C header included by the kernel's C sources) names rtAssertFail and
   rtRunTimeCounter, so they must be extern "C".
   ============================================================================ */

#include <stdint.h>

typedef enum {
  FAULT_NONE = 0,
  FAULT_ASSERT,             /* 1  configASSERT tripped                        */
  FAULT_STACK_OVERFLOW,     /* 2  a task blew its stack                       */
  FAULT_MALLOC,             /* 3  pvPortMalloc / newlib malloc returned NULL  */
  FAULT_SCHEDULER_RETURNED, /* 4  vTaskStartScheduler returned (heap too small)*/
  FAULT_WHEEL_SAT,          /* 5  |omega_w| over WHEEL_SAT_LIMIT              */
  FAULT_UNDERVOLT,          /* 6  bus voltage collapse (Phase 5)             */
  FAULT_OVERCURRENT,        /* 7  (Phase 5)                                  */
  FAULT_HEARTBEAT,          /* 8  a task missed its deadline (Phase 5)       */
  FAULT_I2C_TIMEOUT         /* 9  bus mutex / transfer timeout (Phase 5)     */
} fault_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the safe-stop hardware. Call in setup() BEFORE motor bring-up, so a
   fault during motor.initFOC() (which spins the wheel) still kills the driver.
     hwEnablePin : DRV8313 enable, active-high. Driven LOW to kill the outputs at
                   the pin, independent of any SimpleFOC object state. Pass -1 to
                   skip the hardware kill.
     ledPin      : blinked forever after a fault. On the Nucleo-F446RE the only
                   LED (LD2) is PA5, which is ALSO motor PWM phase A — safe to
                   drive as GPIO only because the driver is already dead by then.
                   Pass -1 for no LED.
     stopHook    : optional graceful stop (e.g. motor.target=0; motor.disable();
                   driver.disable()). May be NULL. Runs AFTER the hardware kill. */
void faults_init(int hwEnablePin, int ledPin, void (*stopHook)(void));

/* Register a kill that runs at the VERY TOP of every fault path — after interrupts
   are masked, but BEFORE the motor enable pin is pulled low and before stopHook.
   Added for the translation fans (Phase 1.3): with the props unguarded (decision B7)
   they are the larger hazard, so if only one actuator kill ever completes it should
   be that one. Both are a handful of register writes, so the ordering costs
   nanoseconds either way.

   The hook MUST be safe with interrupts already disabled: no FreeRTOS API, no lock,
   no unbounded wait. fans_stopAll() is built to that contract.

   Registered as a function pointer rather than faults.cpp calling fans_stopAll()
   directly, deliberately: this header is C-compatible and dependency-free because
   FreeRTOSConfig.h (a C header, included by the kernel's C sources) names
   rtAssertFail and rtRunTimeCounter. Pulling a C++ Arduino module in here would
   break that. May be NULL. */
void faults_setHwKillHook(void (*fn)(void));

/* The ONE stop path. Never returns. */
void faults_safeStop(fault_t reason);

/* Print (and consume) the .noinit record left by the previous boot's fault, if
   any. Call at the top of setup() — with no debugger attached this is how you
   learn what killed the last run. */
void faults_reportLastBoot(void);

/* configASSERT hook (FreeRTOSConfig.h). Latches file/line, then safe-stops. */
void rtAssertFail(const char* file, int line);

/* Run-time-stats counter for configGENERATE_RUN_TIME_STATS. Returns TIM5->CNT. */
uint32_t rtRunTimeCounter(void);

#ifdef __cplusplus
}
#endif
