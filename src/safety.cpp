#include "safety.h"
#include "faults.h"
#include "telemetry.h"
#include <STM32FreeRTOS.h>
#include <task.h>

#define SAFETY_PERIOD_MS      50    /* 20 Hz */
#define HEARTBEAT_TIMEOUT_MS  300   /* no progress for this long = control is dead */
#define SUPPLY_ABSENT_V       1.0f  /* below this the motor supply is simply OFF,
                                       not sagging -- do not trip undervoltage */

static float (*s_wheelVel)(void)          = nullptr;
static void  (*s_safeStop)(const char*)   = nullptr;
static float          s_satLimit          = 0.0f;
static TaskHandle_t   s_task              = nullptr;
static volatile bool  s_armed             = false;
static volatile uint32_t s_kicks          = 0;
static volatile uint32_t s_checks         = 0;

/* Phase 5.2 power monitoring (monitor only -- no trip yet, see safety.h). */
static bool (*s_readPower)(float*, float*) = nullptr;
static volatile float    s_minV     = 1e9f;
static volatile float    s_maxV     = 0.0f;
static volatile float    s_maxA     = 0.0f;
static volatile uint32_t s_pwrFails = 0;
static volatile bool     s_pwrSeen  = false;

void safety_kick(void) { s_kicks++; }
void safety_arm(void)  { s_armed = true; }
static float s_minBusV = 0.0f;      /* 0 = trip disabled */
static float s_maxMilliAmps = 0.0f; /* 0 = trip disabled */

void safety_setPowerMonitor(bool (*readPower)(float*, float*)) { s_readPower = readPower; }
void safety_setPowerLimits(float minBusV, float maxMilliAmps) {
  s_minBusV = minBusV; s_maxMilliAmps = maxMilliAmps;
}

bool safety_powerStats(float* minV, float* maxV, float* maxA, uint32_t* fails) {
  if (minV)  *minV  = s_minV;
  if (maxV)  *maxV  = s_maxV;
  if (maxA)  *maxA  = s_maxA;
  if (fails) *fails = s_pwrFails;
  return s_pwrSeen;
}

static void safetyTask(void*) {
  TickType_t last         = xTaskGetTickCount();
  uint32_t   lastKick     = 0;
  TickType_t lastKickTick = last;

  for (;;) {
    /* If we were starved long enough to fall behind, RESYNC instead of letting
       vTaskDelayUntil return immediately once per missed period. Without this the
       checks below fire back-to-back in microseconds after any long starvation --
       which is exactly how the first version false-tripped the heartbeat after B
       and E (Arduino delay() busy-spins at prio 3 and starves this task). */
    TickType_t nowT = xTaskGetTickCount();
    if ((int32_t)(nowT - last) > (int32_t)pdMS_TO_TICKS(2 * SAFETY_PERIOD_MS))
      last = nowT;

    /* Absolute-time wake: execution time does NOT accumulate as period drift,
       which vTaskDelay would cause. */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAFETY_PERIOD_MS));

    if (!s_armed) {                                   /* hwSetup still running */
      lastKick     = s_kicks;
      lastKickTick = xTaskGetTickCount();
      continue;
    }
    s_checks++;

    /* ---- 1. wheel overspeed, independent of the control task ---------------
       Normally the inline 200 Hz check in controlUpdate fires first; this is the
       backstop for when the control task is not checking at all. */
    if (s_wheelVel) {
      float w = s_wheelVel();
      if (fabsf(w) > s_satLimit) {
        if (s_safeStop) s_safeStop("safety task: wheel overspeed");
        else            faults_safeStop(FAULT_WHEEL_SAT);
      }
    }

    /* ---- 1b. power rail (INA219, the second I2C user) ---------------------
       MONITOR ONLY: record extremes so real thresholds can be chosen from data.
       The read itself takes the I2C mutex inside the sketch-provided callback --
       this task must NOT hold the bus across anything else. */
    if (s_readPower) {
      float busV = 0.0f, mA = 0.0f;
      if (s_readPower(&busV, &mA)) {
        s_pwrSeen = true;
        if (busV < s_minV) s_minV = busV;
        if (busV > s_maxV) s_maxV = busV;
        if (fabsf(mA) > s_maxA) s_maxA = fabsf(mA);

        /* DEBOUNCED: a threshold must hold for 2 consecutive checks (100 ms).
           A single noisy INA219 sample must never kill a run.

           LATCHED: fire ONCE per excursion, then stay quiet until the condition
           clears. Without this, every 50 ms check re-fired the stop and the
           operator got 20 identical messages per second, burying everything else.

           SUPPLY-ABSENT GUARD: a reading below SUPPLY_ABSENT_V means the motor
           supply simply is not connected (bench-testing on USB), not a collapsing
           battery. Tripping there is pointless -- there is no power reaching the
           driver to cut -- and it made the board unusable without the battery. */
        static int  uvStrikes = 0,     ocStrikes = 0;
        static bool uvLatched = false, ocLatched = false;

        if (s_minBusV > 0.0f && busV < s_minBusV && busV > SUPPLY_ABSENT_V) {
          if (++uvStrikes >= 2 && !uvLatched) {
            uvLatched = true;
            if (s_safeStop) s_safeStop("safety: bus undervoltage");
          }
        } else { uvStrikes = 0; uvLatched = false; }

        if (s_maxMilliAmps > 0.0f && fabsf(mA) > s_maxMilliAmps) {
          if (++ocStrikes >= 2 && !ocLatched) {
            ocLatched = true;
            if (s_safeStop) s_safeStop("safety: overcurrent");
          }
        } else { ocStrikes = 0; ocLatched = false; }
      } else {
        s_pwrFails++;
      }
    }

    /* ---- 2. control-task heartbeat ----------------------------------------
       Measured in TIME, not in iterations. An iteration counter assumes this task
       runs on schedule -- but it can be starved (see the resync above), and then N
       consecutive checks span microseconds rather than N periods, so a perfectly
       healthy control task looks dead. Wall-clock elapsed since the counter last
       moved is immune to how often we get to look.

       No progress for HEARTBEAT_TIMEOUT_MS means the control task is wedged (hung
       peripheral, deadlock, crash). It cannot be recovered by clearing flags it
       will never read, so this is terminal: hardware kill, latch the reason to the
       .noinit black box, blink it forever. */
    uint32_t k = s_kicks;
    nowT = xTaskGetTickCount();
    if (k != lastKick) {
      lastKick     = k;
      lastKickTick = nowT;
    } else if ((nowT - lastKickTick) > pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS)) {
      faults_safeStop(FAULT_HEARTBEAT);
    }
  }
}

void safety_init(float (*wheelVel)(void), float satLimit,
                 void (*safeStopFn)(const char*)) {
  s_wheelVel = wheelVel;
  s_satLimit = satLimit;
  s_safeStop = safeStopFn;
  configASSERT(xTaskCreate(safetyTask, "safety", 384, nullptr, 2, &s_task) == pdPASS);
}

uint32_t safety_checks(void) { return s_checks; }

uint32_t safety_stackFreeWords(void) {
  return s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) : 0;
}
