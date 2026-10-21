#pragma once
#include <Arduino.h>

/* Free-running 32-bit microsecond counter on TIM5, independent of SysTick.
   Wraps every ~71.6 minutes; unsigned subtraction handles the wrap. */

void us_init(void);

static inline uint32_t us_now(void) { return TIM5->CNT; }

static inline uint32_t us_since(uint32_t t0) { return TIM5->CNT - t0; }
