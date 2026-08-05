#include "hw_timers.h"
#include "timebase.h"     /* for timerClkFreq(), TIM5 timebase */
#include <HardwareTimer.h>

static void dumpTimer(const char* n, TIM_TypeDef* t, uint32_t rccBit,
                      volatile uint32_t* rccReg) {
  if (!(*rccReg & rccBit)) {
    Serial.print(n); Serial.println("  [clock disabled - free]");
    return;
  }
  Serial.print(n);
  Serial.print("  CEN=");   Serial.print(t->CR1 & TIM_CR1_CEN);
  Serial.print(" PSC=");    Serial.print(t->PSC);
  Serial.print(" ARR=");    Serial.print(t->ARR);
  Serial.print(" CCER=0x"); Serial.print(t->CCER, HEX);
  Serial.print(" CCMR1=0x");Serial.print(t->CCMR1, HEX);
  Serial.print(" DIER=0x"); Serial.print(t->DIER, HEX);
  if (t->CR1 & TIM_CR1_CEN) {
    uint32_t clk = timerClkFreq(t);
    float hz = (float)clk / ((t->PSC + 1.0f) * (t->ARR + 1.0f));
    Serial.print("  -> "); Serial.print(hz, 1); Serial.print(" Hz");
  }
  Serial.println();
}

void timers_dumpAll(const char* label) {
  Serial.print("--- timer audit: "); Serial.print(label); Serial.println(" ---");
  dumpTimer("TIM1 ", TIM1,  RCC_APB2ENR_TIM1EN,  &RCC->APB2ENR);
  dumpTimer("TIM2 ", TIM2,  RCC_APB1ENR_TIM2EN,  &RCC->APB1ENR);
  dumpTimer("TIM3 ", TIM3,  RCC_APB1ENR_TIM3EN,  &RCC->APB1ENR);
  dumpTimer("TIM4 ", TIM4,  RCC_APB1ENR_TIM4EN,  &RCC->APB1ENR);
  dumpTimer("TIM5 ", TIM5,  RCC_APB1ENR_TIM5EN,  &RCC->APB1ENR);
  dumpTimer("TIM8 ", TIM8,  RCC_APB2ENR_TIM8EN,  &RCC->APB2ENR);
  dumpTimer("TIM9 ", TIM9,  RCC_APB2ENR_TIM9EN,  &RCC->APB2ENR);
  dumpTimer("TIM10", TIM10, RCC_APB2ENR_TIM10EN, &RCC->APB2ENR);
  dumpTimer("TIM11", TIM11, RCC_APB2ENR_TIM11EN, &RCC->APB2ENR);
  dumpTimer("TIM12", TIM12, RCC_APB1ENR_TIM12EN, &RCC->APB1ENR);
  Serial.println("------------------------------");
}

/* ---- Step 1.5 — FOC tick on TIM9 --------------------------------------- */
static HardwareTimer*    s_focTim   = nullptr;
static volatile uint32_t s_focCount = 0;
static volatile uint32_t s_focLast  = 0;
static volatile uint32_t s_focDtMin = 0xFFFFFFFFu;
static volatile uint32_t s_focDtMax = 0;

/* Runs inside the core's TIM9 handler (HAL_TIM_IRQHandler clears the flag for
   us). Timestamp with the INDEPENDENT TIM5 timebase so we don't measure jitter
   against the very clock that generates the tick. Unsigned subtraction is
   wrap-correct. Phase 4 replaces this body with the FOC/control task notifies. */
static void focTick_isr(void) {
  uint32_t now = TIM5->CNT;
  if (s_focCount) {                        /* skip the first (no prior sample) */
    uint32_t dt = now - s_focLast;
    if (dt < s_focDtMin) s_focDtMin = dt;
    if (dt > s_focDtMax) s_focDtMax = dt;
  }
  s_focLast = now;
  s_focCount++;
}

void focTick_init(uint32_t hz) {
  s_focTim = new HardwareTimer(TIM9);
  s_focTim->setOverflow(hz, HERTZ_FORMAT);
  s_focTim->attachInterrupt(focTick_isr);
  s_focTim->resume();
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);   /* override the API default */
}

uint32_t focTick_count(void) { return s_focCount; }

void focTick_resetStats(void) {
  s_focDtMin = 0xFFFFFFFFu;
  s_focDtMax = 0;
  s_focLast  = TIM5->CNT;
  s_focCount = 0;
}

void focTick_jitter(uint32_t* min_us, uint32_t* max_us) {
  if (min_us) *min_us = s_focDtMin;
  if (max_us) *max_us = s_focDtMax;
}