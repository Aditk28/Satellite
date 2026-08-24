#pragma once
#include <Arduino.h>

/* Free-running 32-bit microsecond counter on TIM5.
   Independent of SysTick, so it survives the FreeRTOS handoff.
   Wraps every ~71.6 minutes; unsigned subtraction handles the wrap. */
   
/* Actual clock feeding a timer's counter, accounting for the APB ×2 rule. */
uint32_t timerClkFreq(TIM_TypeDef* t);

void us_init(void);

static inline uint32_t us_now(void) { return TIM5->CNT; }

/* Elapsed microseconds since `t0`, correct across one wrap. */
static inline uint32_t us_since(uint32_t t0) { return TIM5->CNT - t0; }