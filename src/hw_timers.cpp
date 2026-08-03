#include "hw_timers.h"
#include "timebase.h"     /* for timerClkFreq() */

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