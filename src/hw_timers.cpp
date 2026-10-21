#include "hw_timers.h"
#include "timebase.h"
#include <HardwareTimer.h>

static HardwareTimer*    s_focTim   = nullptr;
static volatile uint32_t s_focCount = 0;
static volatile uint32_t s_focLast  = 0;
static volatile uint32_t s_focDtMin = 0xFFFFFFFFu;
static volatile uint32_t s_focDtMax = 0;
static void (*s_focCb)(void) = nullptr;

/* Called from the core's TIM9 handler, which clears the flag. Jitter is measured
   against TIM5 so it is independent of the timer generating the tick. */
static void focTick_isr(void) {
  uint32_t now = TIM5->CNT;
  if (s_focCount) {                        /* skip the first (no prior sample) */
    uint32_t dt = now - s_focLast;
    if (dt < s_focDtMin) s_focDtMin = dt;
    if (dt > s_focDtMax) s_focDtMax = dt;
  }
  s_focLast = now;
  s_focCount++;
  if (s_focCb) s_focCb();
}

void focTick_attach(void (*cb)(void)) { s_focCb = cb; }

void focTick_init(uint32_t hz) {
  s_focTim = new HardwareTimer(TIM9);
  s_focTim->setOverflow(hz, HERTZ_FORMAT);
  s_focTim->attachInterrupt(focTick_isr);
  s_focTim->resume();
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);   /* override the API default */
}

void focTick_jitter(uint32_t* min_us, uint32_t* max_us) {
  if (min_us) *min_us = s_focDtMin;
  if (max_us) *max_us = s_focDtMax;
}
