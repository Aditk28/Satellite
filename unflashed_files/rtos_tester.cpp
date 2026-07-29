/* ============================================================================
 * STEP 2 — Timer-driven semaphore pattern (deterministic tick), no sensors yet.
 *
 * This replaces TaskBlink's vTaskDelay loop with the actual mechanism the
 * real Control Loop task will use: a hardware timer ISR gives a binary
 * semaphore at a fixed rate, and the task blocks on that semaphore rather
 * than sleeping on its own schedule.
 *
 * Goal: prove the tick is actually regular and fast, before any sensor
 * reads or math get added on top.
 *
 * How to verify this actually worked (do this, don't just trust it "looks"
 * fine — jitter is invisible to the eye on an LED):
 *   - Ideally: probe PIN_TOGGLE (D2 here) with an oscilloscope or logic
 *     analyzer. You should see a clean, regular square wave at exactly
 *     half your CONTROL_LOOP_HZ (it toggles once per tick, so one full
 *     on/off cycle = 2 ticks).
 *   - No scope handy yet: the serial heartbeat below also prints how many
 *     control ticks happened in the last second. At CONTROL_LOOP_HZ = 1000,
 *     it should read consistently ~1000, not drifting or noisy.
 * ==========================================================================*/

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// --- Config ---
#define CONTROL_LOOP_HZ   1000     // target control loop rate
#define PIN_TOGGLE         D2      // scope/logic analyzer probe point
                                   // (separate from LED_BUILTIN so the
                                   // heartbeat task can still use the LED
                                   // independently if you want later)

// --- FreeRTOS handles ---
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xHeartbeatTaskHandle = NULL;
SemaphoreHandle_t xControlSemaphore = NULL;
HardwareTimer *controlTimer = NULL;

// --- Shared counter for the heartbeat task to report tick rate.
//     'volatile' matters here: it tells the compiler this value can change
//     at any time from outside the normal flow (from the ISR / another
//     task), so it must not cache it in a register and skip re-reading it. ---
volatile uint32_t controlTickCount = 0;

/* ----------------------------------------------------------------------
 * Timer ISR — runs in interrupt context. Keep this as close to nothing
 * as possible: no Serial prints, no delays, no floating point you don't
 * have to do. Its only job is to wake the Control task.
 * ------------------------------------------------------------------- */
void onControlTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Give the semaphore FROM AN ISR — note the FromISR variant. Regular
  // xSemaphoreGive() is not safe to call from interrupt context.
  xSemaphoreGiveFromISR(xControlSemaphore, &xHigherPriorityTaskWoken);

  // If giving the semaphore just woke a task of higher priority than
  // whatever was running before this interrupt, this tells FreeRTOS to
  // switch to it immediately after the ISR returns, rather than waiting
  // for the next scheduler tick. Boilerplate you always want here.
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ----------------------------------------------------------------------
 * Control Loop task — blocks on the semaphore, wakes exactly once per
 * timer tick. This is the task real sensor reads / FOC / control math
 * will go inside of later — for now it just toggles a pin and counts.
 * ------------------------------------------------------------------- */
void TaskControlLoop(void *pvParameters) {
  pinMode(PIN_TOGGLE, OUTPUT);
  bool pinState = false;

  for (;;) {
    // Block here — uses ~0 CPU — until the ISR gives the semaphore.
    // portMAX_DELAY means "wait forever," which is correct: if this
    // never wakes, something is wrong with the timer, and we want that
    // to be obvious (nothing toggling) rather than silently falling
    // back to some other rate.
    xSemaphoreTake(xControlSemaphore, portMAX_DELAY);

    pinState = !pinState;
    digitalWrite(PIN_TOGGLE, pinState);
    controlTickCount++;

    // --- Real work will eventually go here, in this exact spot:
    //     read IMU, read MT6701, run fusion filter, run PID/LQR,
    //     call motor.loopFOC() + motor.move(). All of it needs to fit
    //     comfortably inside one tick period (1ms at 1kHz) — this is
    //     exactly why we're proving the empty-loop rate first. ---
  }
}

/* ----------------------------------------------------------------------
 * Heartbeat task — unchanged in spirit from Step 1, but now also reports
 * the control tick count so you can sanity-check the rate from the
 * serial monitor even without a scope.
 * ------------------------------------------------------------------- */
void TaskHeartbeat(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(1000);
  uint32_t lastCount = 0;

  for (;;) {
    // Snapshot the volatile counter and compute ticks-in-the-last-second.
    // Not mutex-protected: a single uint32_t read/write is atomic on
    // Cortex-M4, so this is safe without one. (This is the one case
    // where skipping a mutex is fine — anything more than a single
    // primitive read/write would need one.)
    uint32_t currentCount = controlTickCount;
    uint32_t ticksThisSecond = currentCount - lastCount;
    lastCount = currentCount;

    Serial.print("heartbeat | control ticks last second: ");
    Serial.println(ticksThisSecond);

    vTaskDelay(xDelay);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Step 2: timer + semaphore bring-up...");

  // Create the semaphore BEFORE starting the timer — if the timer somehow
  // fired before this exists, xSemaphoreGiveFromISR would be handed a
  // null handle.
  xControlSemaphore = xSemaphoreCreateBinary();
  if (xControlSemaphore == NULL) {
    Serial.println("!! Failed to create semaphore !!");
    while (1) {}
  }

  // TIM2 is a general-purpose timer available on the Nucleo-F446RE and
  // not claimed by the Arduino core for PWM/millis() by default — a safe
  // first choice. If you later find a conflict (e.g. once fan ESCs need
  // PWM timers too), this is the line to revisit.
  controlTimer = new HardwareTimer(TIM2);
  controlTimer->setOverflow(CONTROL_LOOP_HZ, HERTZ_FORMAT);
  controlTimer->attachInterrupt(onControlTimer);
  controlTimer->resume();

  xTaskCreate(TaskControlLoop, "Control", 512, NULL, 3, &xControlTaskHandle);
  xTaskCreate(TaskHeartbeat,   "Heartbeat", 256, NULL, 1, &xHeartbeatTaskHandle);

  vTaskStartScheduler();

  Serial.println("!! Scheduler failed to start !!");
}

void loop() {
  // Empty — everything happens in tasks + the timer ISR above.
}