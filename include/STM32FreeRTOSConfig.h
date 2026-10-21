#pragma once
/* FreeRTOS kernel configuration.

   This file must be named STM32FreeRTOSConfig.h and live in include/: the
   library's FreeRTOSConfig.h checks __has_include("STM32FreeRTOSConfig.h"), and
   PlatformIO puts include/ on the kernel's include path but not src/. If it is not
   found, the library defaults are used silently (configASSERT becomes a bare hang).

   Heap is heap_3 (-D configMEMMANG_HEAP_NB=3, newlib malloc), so
   configTOTAL_HEAP_SIZE is unused. The Arduino core owns SysTick at 1 kHz and
   forwards it to the kernel via osSystickHandler, so the tick rate must be 1000
   and xPortSysTickHandler is not mapped to SysTick_Handler. */

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif

/* Scheduler */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configCPU_CLOCK_HZ                        (SystemCoreClock)
#define configTICK_RATE_HZ                        ((TickType_t)1000)  /* shares the core's SysTick */
#define configMAX_PRIORITIES                      6                   /* idle0 telem1 safety/comms2 ctrl3 foc4 */
#define configMINIMAL_STACK_SIZE                  ((uint16_t)128)     /* words */
#define configMAX_TASK_NAME_LEN                   16
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   1
#define configUSE_TASK_NOTIFICATIONS              1
#define configUSE_MUTEXES                         1
#define configUSE_RECURSIVE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES             1
#define configUSE_TIMERS                          1
#define configTIMER_TASK_PRIORITY                 2
#define configTIMER_QUEUE_LENGTH                  10
#define configTIMER_TASK_STACK_DEPTH              (configMINIMAL_STACK_SIZE * 2)
#define configQUEUE_REGISTRY_SIZE                 8

/* Memory */
#define configSUPPORT_DYNAMIC_ALLOCATION          1
#define configSUPPORT_STATIC_ALLOCATION           0
#define configTOTAL_HEAP_SIZE                     ((size_t)(32 * 1024)) /* unused under heap_3 */
#define configUSE_NEWLIB_REENTRANT                1                     /* per-task _reent for String/sprintf */
#define configUSE_MALLOC_FAILED_HOOK              1
#define configUSE_MALLOC_FAILED_HOOK_BLINK        0   /* hook is defined in faults.cpp */

/* Debugging and hooks */
#define configUSE_IDLE_HOOK                       0
#define configUSE_TICK_HOOK                       0
#define configCHECK_FOR_STACK_OVERFLOW            2
#define configCHECK_FOR_STACK_OVERFLOW_BLINK      0   /* hook is defined in faults.cpp */
#define configUSE_TRACE_FACILITY                  1
#define configGENERATE_RUN_TIME_STATS             1   /* per-task CPU%, clocked off TIM5 */
#define configUSE_STATS_FORMATTING_FUNCTIONS      1

/* Run-time stats use the TIM5 microsecond timebase, read through a function so
   this header does not need the CMSIS device header. */
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

/* Assertions stop the motor and record the location in the black box. */
#define configASSERT(x)  if((x) == 0) { taskDISABLE_INTERRUPTS(); rtAssertFail(__FILE__, __LINE__); }

/* Interrupt priorities (4 priority bits). An ISR that calls a FromISR API must
   have a numeric priority of 5 or more (5 is the most urgent allowed). */
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

/* Optional API */
#define INCLUDE_vTaskPrioritySet                  1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskDelete                       1
#define INCLUDE_vTaskSuspend                      1
#define INCLUDE_vTaskDelayUntil                   1
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_xTaskGetSchedulerState            1   /* guards the pre-scheduler SysTick */
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define INCLUDE_xTaskGetIdleTaskHandle            1
#define INCLUDE_eTaskGetState                     1
#define INCLUDE_xTaskGetCurrentTaskHandle         1
#define INCLUDE_xTimerPendFunctionCall            1

/* Map the port's handlers to the CMSIS vector names. SysTick is not mapped (see top). */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
