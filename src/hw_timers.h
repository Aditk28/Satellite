#pragma once
#include <Arduino.h>

/* FOC tick on TIM9. Uses the HardwareTimer API because the STM32duino core already
   defines the TIM9 vector. The IRQ is set to priority 5 (configMAX_SYSCALL), so the
   callback may use FreeRTOS FromISR calls. */
void     focTick_init(uint32_t hz);
void     focTick_jitter(uint32_t* min_us, uint32_t* max_us);  /* inter-fire interval, TIM5 */

/* Callback run inside the TIM9 ISR on every tick. Attach before focTick_init(). */
void     focTick_attach(void (*cb)(void));
