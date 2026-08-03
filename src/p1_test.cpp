/* ============================================================================
   p1_test.cpp — Phase 1 test harness (Steps 1.3–1.5).  THROWAWAY.
   ----------------------------------------------------------------------------
   Build/flash:  pio run -e p1test -t upload      (NOT the default env)
   NO MOTOR is initialised or driven — safe to run with nothing spinning.

   DIAGNOSTIC MODE (Step 1.3 debug). Starts at the LEGAL priority (5) to prove
   the timer + notification + scheduler machinery, then you flip TEST_IRQ_PRIO to
   0 to prove the illegal-priority assert. The serial output tells you which
   stage works:
     * "LED sanity: 3 blinks" — you should SEE LD2 blink 3x. Proves the LED/pin.
     * "testTask running"     — the scheduler started and the task runs.
     * "notification N (isr=X)" streaming — the TIM7 ISR fires and notifies.
     * "heartbeat N (isr=0)"  — task runs but the TIM7 ISR is NOT firing.
   ============================================================================ */
#include <Arduino.h>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include "timebase.h"
#include "faults.h"

#define PIN_ENABLE   10
#define FAULT_LED    LED_BUILTIN

/* 5 = legal boundary (no assert; notifications stream).  0 = illegal (assert
   must fire). Part 1 (0) proven; this is the Part 2 positive control (5). */
#define TEST_IRQ_PRIO  5

static TaskHandle_t hTestTask;
static HardwareTimer* testTim = nullptr;
static volatile uint32_t g_isrCount = 0;

static void onTick(void) {
  g_isrCount++;                                 /* did the ISR actually fire? */
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(hTestTask, &woken);    /* validates IRQ priority */
  portYIELD_FROM_ISR(woken);
}

static void startTestTimer(uint32_t hz, uint32_t nvicPrio) {
  testTim = new HardwareTimer(TIM7);
  testTim->setOverflow(hz, HERTZ_FORMAT);
  testTim->attachInterrupt(onTick);
  testTim->resume();
  HAL_NVIC_SetPriority(TIM7_IRQn, nvicPrio, 0);  /* force the test priority */
}

static void ledBlink(int n, int ms) {
  pinMode(FAULT_LED, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(FAULT_LED, HIGH); delay(ms);
    digitalWrite(FAULT_LED, LOW);  delay(ms);
  }
}

static void testTask(void*) {
  Serial.println("[p1test] testTask running");
  Serial.flush();
  uint32_t n = 0, beats = 0;
  for (;;) {
    /* Wait up to 1 s for a notification. Got one -> timer/ISR works. Timed out
       -> print a heartbeat with the ISR count so we can see if it ever fired. */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
      n++;
      if (n <= 8 || (n % 8 == 0)) {
        Serial.print("[p1test] notification "); Serial.print(n);
        Serial.print("  (isr="); Serial.print((uint32_t)g_isrCount); Serial.println(")");
        Serial.flush();
      }
    } else {
      beats++;
      Serial.print("[p1test] heartbeat "); Serial.print(beats);
      Serial.print("  (isr="); Serial.print((uint32_t)g_isrCount); Serial.println(")");
      Serial.flush();
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  us_init();

  Serial.println();
  Serial.println("=== Phase 1 test harness (p1test) ===");
  faults_reportLastBoot();

  Serial.print("PRIGROUP before = "); Serial.println((SCB->AIRCR >> 8) & 7u);
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  Serial.print("PRIGROUP after  = "); Serial.println((SCB->AIRCR >> 8) & 7u);

  Serial.println("LED sanity: LD2 should blink 3x now...");
  Serial.flush();
  ledBlink(3, 200);

  faults_init(PIN_ENABLE, FAULT_LED, NULL);

  configASSERT(xTaskCreate(testTask, "test", 256, NULL, 3, &hTestTask) == pdPASS);

  Serial.print("TEST_IRQ_PRIO = "); Serial.println(TEST_IRQ_PRIO);
  Serial.println(TEST_IRQ_PRIO < 5
    ? "  -> ILLEGAL: expect assert (LED 1-pulse blink + BLACKBOX on reset)."
    : "  -> legal: expect 'notification N' streaming at 4 Hz.");
  Serial.flush();

  startTestTimer(4, TEST_IRQ_PRIO);             /* 4 Hz */

  /* Read back what the ISR priority ACTUALLY is, vs the syscall threshold.
     For the assert to fire, TIM7's priority must be numerically LESS than
     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5). If it's >= 5, the priority
     never took and the ISR is "legal" -> no assert, notifications stream. */
  uint32_t tim7prio = NVIC_GetPriority(TIM7_IRQn);
  Serial.print("TIM7 NVIC priority   = "); Serial.println(tim7prio);
  Serial.print("max-syscall priority = "); Serial.println((uint32_t)configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  Serial.println((tim7prio < (uint32_t)configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
    ? "  -> ILLEGAL (prio < max-syscall): assert SHOULD fire on first ISR"
    : "  -> legal (prio >= max-syscall): no assert; notifications will stream");
  Serial.flush();

  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);    /* only if the heap is too small */
}

void loop() {}
