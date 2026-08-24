/* ============================================================================
   p1_test.cpp — Phase 1 test harness.  THROWAWAY.
   ----------------------------------------------------------------------------
   Build/flash:  pio run -e p1test -t upload      (NOT the default env)
   NO MOTOR is initialised or driven — safe to run with nothing spinning.

   STEP 1.5 — verify the FOC tick timer (TIM9 @ 4 kHz, IRQ prio 5).
     No 'Y' needed: after boot it measures for 5 s and auto-prints.
     Expect:
       - rate  = 4000 Hz within ~0.1% (target 3996..4004),
       - inter-fire dt ~250 us, jitter (max-min) under ~3 us,
       - the timer dump shows TIM9 running at 4000 Hz and TIM2/TIM3 free
         (no SimpleFOC here, so they're clock-disabled — confirms the
         HardwareTimer(TIM9) setup touched only TIM9).
   ============================================================================ */
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "timebase.h"
#include "faults.h"
#include "hw_timers.h"

#define PIN_ENABLE   10
#define FAULT_LED    LED_BUILTIN
#define FOC_HZ       4000u

static void reportTask(void*) {
  Serial.println("[p1test] measuring FOC tick for 5 s...");
  Serial.flush();

  focTick_resetStats();
  uint32_t c0 = focTick_count();
  uint32_t t0 = us_now();
  vTaskDelay(pdMS_TO_TICKS(5000));
  uint32_t c1 = focTick_count();
  uint32_t t1 = us_now();

  uint32_t window = (uint32_t)(t1 - t0);
  float    rate   = (c1 - c0) * 1e6f / (float)window;
  uint32_t mn, mx;
  focTick_jitter(&mn, &mx);

  Serial.print("ticks=");     Serial.print(c1 - c0);
  Serial.print("  window_us="); Serial.println(window);
  Serial.print("rate = ");    Serial.print(rate, 2);
  Serial.print(" Hz   (target "); Serial.print(FOC_HZ); Serial.println(")");
  Serial.print("inter-fire dt(us): min="); Serial.print(mn);
  Serial.print(" max=");      Serial.print(mx);
  Serial.print("  ideal=");   Serial.print(1e6f / FOC_HZ, 1);
  Serial.print("  jitter(max-min)="); Serial.print(mx - mn); Serial.println(" us");
  Serial.flush();

  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  us_init();

  Serial.println();
  Serial.println("=== Phase 1 FOC-tick verify (p1test, Step 1.5) ===");
  faults_reportLastBoot();
  faults_init(PIN_ENABLE, FAULT_LED, NULL);

  focTick_init(FOC_HZ);                 /* TIM9 @ 4 kHz, IRQ prio 5 */
  timers_dumpAll("after focTick_init"); /* TIM9 running; TIM2/TIM3 must be free */

  configASSERT(xTaskCreate(reportTask, "report", 512, NULL, 3, NULL) == pdPASS);

  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);   /* only if the heap is too small */
}

void loop() {}
