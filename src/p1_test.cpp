/* ============================================================================
   p1_test.cpp — Phase 1 test harness.  THROWAWAY.
   ----------------------------------------------------------------------------
   Build/flash:  pio run -e p1test -t upload      (NOT the default env)
   NO MOTOR is initialised or driven — safe to run with nothing spinning.

   STEP 1.4 — verify the scheduler tracer.
     Two tasks make preemption visible:
       - taskPeriodic (prio 3): ~100 us of work every 1 ms (vTaskDelayUntil).
       - taskSpinner  (prio 1): never blocks; fills the gaps.
     Trace runs from scheduler start. Let it run ~1 s, then send 'Y' to dump the
     ring buffer as CSV. Save that to a file and run tools/plot_trace.py.

     PREDICTION (this is what makes the tracer trustworthy as ground truth):
       - TEST  (id 6) appears as a clean bar every 1000 us,
       - TEST2 (id 7) bars are CHOPPED at those instants (preemption),
       - IDLE  (id 0) is ABSENT (the spinner is always Ready, so idle never runs).
   ============================================================================ */
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "timebase.h"
#include "faults.h"
#include "trace.h"

#define PIN_ENABLE   10
#define FAULT_LED    LED_BUILTIN

static TaskHandle_t hPeriodic, hSpinner;

/* prio 3: ~100 us of work every 1 ms. Also polls Serial for 'Y' to dump. */
static void taskPeriodic(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));            /* wake every 1 ms */
    uint32_t t0 = us_now();
    while (us_since(t0) < 100) { __asm__ volatile(""); } /* ~100 us of "work" */
    while (Serial.available()) {                         /* dump on demand */
      char c = Serial.read();
      if (c == 'Y' || c == 'y') trace_dump(Serial);
    }
  }
}

/* prio 1: never blocks. Gets chopped by taskPeriodic every 1 ms. */
static void taskSpinner(void*) {
  volatile uint32_t x = 0;
  for (;;) { x++; }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  us_init();

  Serial.println();
  Serial.println("=== Phase 1 tracer verify (p1test, Step 1.4) ===");
  faults_reportLastBoot();
  faults_init(PIN_ENABLE, FAULT_LED, NULL);

  configASSERT(xTaskCreate(taskPeriodic, "period", 512, NULL, 3, &hPeriodic) == pdPASS);
  configASSERT(xTaskCreate(taskSpinner,  "spin",   256, NULL, 1, &hSpinner)  == pdPASS);

  /* stash each task's trace index in TLS slot 0. Idle stays NULL=0=IDLE. */
  vTaskSetThreadLocalStoragePointer(hPeriodic, 0, (void*)(uintptr_t)TRACE_ID_TEST);
  vTaskSetThreadLocalStoragePointer(hSpinner,  0, (void*)(uintptr_t)TRACE_ID_TEST2);

  Serial.println("tracing... let it run ~1 s, then send 'Y' to dump the CSV.");
  Serial.println("expect: TEST(6) every ~1000us, TEST2(7) chopped, IDLE(0) absent.");
  Serial.flush();

  trace_start();
  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);   /* only if the heap is too small */
}

void loop() {}
