#pragma once
/* ============================================================================
   STM32FreeRTOSConfig.h  —  the kernel contract (Step 1.2)
   ----------------------------------------------------------------------------
   LOCATION IS LOad-BEARING. This lives in include/, NOT src/. The library's
   wrapper FreeRTOSConfig.h (in .pio/libdeps/.../src/) does
   `#if __has_include("STM32FreeRTOSConfig.h")` to decide whether to take our
   full override or fall back to FreeRTOSConfig_Default.h. That __has_include
   searches the wrapper's own directory and the -I include path. PlatformIO puts
   the project include/ dir on that path but NOT src/, so a copy in src/ is
   invisible to the kernel build and silently ignored — you get the library
   defaults (whose configASSERT is a bare for(;;) hang: no rtAssertFail, no LED,
   no black box). Keep this file in include/.

   FILENAME is also load-bearing: it must be exactly STM32FreeRTOSConfig.h (the
   name the wrapper probes), not FreeRTOSConfig.h.

   HEAP: selected by the -D configMEMMANG_HEAP_NB=3 build flag (heap_3 = a
   thread-safe wrapper around newlib malloc/free). Under heap_3, configTOTAL_
   HEAP_SIZE below is DECORATIVE — the real heap is the C-runtime heap between
   _end and the stack in the linker script. Task stacks come from malloc; a
   failed malloc fires vApplicationMallocFailedHook (see faults.cpp).

   SYSTICK: on STM32duino the Arduino core owns SysTick at 1 kHz and calls a weak
   osSystickHandler(); the FreeRTOS library overrides that to drive the kernel
   tick, guarded by INCLUDE_xTaskGetSchedulerState (which we set to 1). So
   (a) configTICK_RATE_HZ MUST be 1000 to match the core, and (b) we do NOT alias
   xPortSysTickHandler -> SysTick_Handler (the core already defines it).
   ============================================================================ */

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif

/* ---- scheduler ---------------------------------------------------------- */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configCPU_CLOCK_HZ                        (SystemCoreClock)
#define configTICK_RATE_HZ                        ((TickType_t)1000)  /* MUST be 1000 (shares core SysTick) */
#define configMAX_PRIORITIES                      6                   /* idle0 telem1 safety/comms2 ctrl3 foc4 */
#define configMINIMAL_STACK_SIZE                  ((uint16_t)128)     /* WORDS (=512 bytes), not bytes */
#define configMAX_TASK_NAME_LEN                   16
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   1
#define configUSE_TASK_NOTIFICATIONS              1                   /* Phase 4: FOC/control notify */
#define configUSE_MUTEXES                         1                   /* Phase 5: I2C bus mutex */
#define configUSE_RECURSIVE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES             1
#define configUSE_TIMERS                          1
#define configTIMER_TASK_PRIORITY                 2
#define configTIMER_QUEUE_LENGTH                  10
#define configTIMER_TASK_STACK_DEPTH              (configMINIMAL_STACK_SIZE * 2)
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS   1                   /* tracer task-index trick (Step 1.4) */
#define configQUEUE_REGISTRY_SIZE                 8

/* ---- memory ------------------------------------------------------------- */
#define configSUPPORT_DYNAMIC_ALLOCATION          1
#define configSUPPORT_STATIC_ALLOCATION           0
#define configTOTAL_HEAP_SIZE                     ((size_t)(32 * 1024)) /* decorative under heap_3 (see header) */
#define configUSE_NEWLIB_REENTRANT                1                     /* per-task _reent: String/sprintf across tasks */
#define configUSE_MALLOC_FAILED_HOOK              1
#define configUSE_MALLOC_FAILED_HOOK_BLINK        0   /* 0 => WE define the hook (faults.cpp), not the library */

/* ---- debugging / hooks -------------------------------------------------- */
#define configUSE_IDLE_HOOK                       0   /* we don't route idle -> loop() */
#define configUSE_TICK_HOOK                       0
#define configCHECK_FOR_STACK_OVERFLOW            2   /* method 2: pattern check at switch time */
#define configCHECK_FOR_STACK_OVERFLOW_BLINK      0   /* 0 => WE define the hook (faults.cpp) */
#define configUSE_TRACE_FACILITY                  1
#define configGENERATE_RUN_TIME_STATS             1   /* per-task CPU%, clocked off TIM5 */
#define configUSE_STATS_FORMATTING_FUNCTIONS      1

/* Run-time-stats clock = the free-running TIM5 µs timebase (Step 0.2). Routed
   through a function so this header needn't include the CMSIS device header just
   to name TIM5; rtRunTimeCounter() lives in faults.cpp. */
#ifdef __cplusplus
extern "C" {
#endif
uint32_t rtRunTimeCounter(void);
void     rtAssertFail(const char* file, int line);
#ifdef __cplusplus
}
#endif
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  /* TIM5 already running */
#define portGET_RUN_TIME_COUNTER_VALUE()          rtRunTimeCounter()

/* ---- scheduler tracer hooks (Step 1.4) ----------------------------------
   The kernel calls traceTASK_SWITCHED_OUT/IN inside vTaskSwitchContext on every
   context switch. We point them at traceOut()/traceIn() (defined in trace.cpp)
   to log the scheduler timeline. trace_c.h lives in include/ so this kernel-side
   include resolves (same reason this config does). Do NOT pull the C++ trace.h
   in here — it's included by plain-C kernel TUs. */
#include "trace_c.h"
#define traceTASK_SWITCHED_IN()   traceIn()
#define traceTASK_SWITCHED_OUT()  traceOut()

/* ---- the assertion: the single most important line here. Silent kernel
   corruption (priority-0 ISR calling an API, mutex-from-ISR, blocking before the
   scheduler starts) becomes a caught, motor-killed, reported halt. ---------- */
#define configASSERT(x)  if((x) == 0) { taskDISABLE_INTERRUPTS(); rtAssertFail(__FILE__, __LINE__); }

/* ---- interrupt priorities (Cortex-M4, 4 implemented priority bits) ------- */
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS                            __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS                            4
#endif
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
          ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* ---- optional API surface we actually use ------------------------------- */
#define INCLUDE_vTaskPrioritySet                  1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskDelete                       1
#define INCLUDE_vTaskSuspend                      1
#define INCLUDE_vTaskDelayUntil                   1
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_xTaskGetSchedulerState            1   /* guards the pre-scheduler SysTick */
#define INCLUDE_uxTaskGetStackHighWaterMark       1   /* G-command stack check (Step 2.1) */
#define INCLUDE_xTaskGetIdleTaskHandle            1
#define INCLUDE_eTaskGetState                     1
#define INCLUDE_xTaskGetCurrentTaskHandle         1
#define INCLUDE_xTimerPendFunctionCall            1

/* ---- handler name mapping. The port's handlers must carry the CMSIS vector
   names or the vector table never dispatches to them. Do NOT map
   xPortSysTickHandler here (see header note re: osSystickHandler). ---------- */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
