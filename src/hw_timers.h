#pragma once
#include <Arduino.h>

void timers_dumpAll(const char* label);          /* Step 0.3 — timer audit */

/* Step 1.5 — FOC tick on TIM9. The STM32duino core strongly defines the
   TIM1_BRK_TIM9 vector (HardwareTimer.cpp, #if TIM9_BASE), so a raw handler
   collides at link — same as TIM7. focTick_init() uses the HardwareTimer API and
   forces the IRQ to priority 5 (the configMAX_SYSCALL boundary, so a Phase-4
   FromISR notify is legal). ISR body for now: count + measure inter-fire jitter
   against the independent TIM5 timebase. */
void     focTick_init(uint32_t hz);              /* start TIM9 @ hz, IRQ prio 5 */
uint32_t focTick_count(void);                    /* total ISR fires since reset */
void     focTick_resetStats(void);               /* zero count + jitter accumulators */
void     focTick_jitter(uint32_t* min_us, uint32_t* max_us);  /* inter-fire delta */