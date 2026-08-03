#include "timebase.h"

uint32_t timerClkFreq(TIM_TypeDef* t) {
  bool onAPB2 = (t == TIM1  || t == TIM8  || t == TIM9 ||
                 t == TIM10 || t == TIM11);

  uint32_t presc = onAPB2
      ? ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos)
      : ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);

  uint32_t pclk = onAPB2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

  return (presc < 4) ? pclk : pclk * 2;
}

void us_init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1  = 0;
  TIM5->PSC  = (timerClkFreq(TIM5) / 1000000UL) - 1;
  TIM5->ARR  = 0xFFFFFFFFUL;
  TIM5->EGR  = TIM_EGR_UG;
  TIM5->SR   = 0;
  TIM5->DIER = 0;
  TIM5->CR1  = TIM_CR1_CEN;
}