#include "safety.h"
#include "faults.h"
#include "telemetry.h"
#include <STM32FreeRTOS.h>
#include <task.h>

#define SAFETY_PERIOD_MS      50    /* 20 Hz */
#define HEARTBEAT_TIMEOUT_MS  300   /* no kick for this long = control task is dead */
#define FAN_STALL_TIMEOUT_MS  200   /* ~100 missed frames at 500 Hz */
#define SUPPLY_ABSENT_V       1.0f  /* below this the motor supply is off, not sagging */

static float (*s_wheelVel)(void)          = nullptr;
static void  (*s_safeStop)(const char*)   = nullptr;
static float          s_satLimit          = 0.0f;
static TaskHandle_t   s_task              = nullptr;
static volatile bool  s_armed             = false;
static volatile uint32_t s_kicks          = 0;
static volatile uint32_t s_checks         = 0;

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

static uint32_t (*s_fanFrames)(void)  = nullptr;
static bool     (*s_fanRunning)(void) = nullptr;

void safety_setFanMonitor(uint32_t (*fanFrames)(void), bool (*fanRunning)(void)) {
  s_fanFrames = fanFrames; s_fanRunning = fanRunning;
}

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
  uint32_t   lastFanFrame = 0;
  TickType_t lastFanTick  = last;
  bool       fanLatched   = false;

  for (;;) {
    /* After a long starvation, resync instead of letting vTaskDelayUntil return
       immediately once per missed period and run the checks back-to-back. */
    TickType_t nowT = xTaskGetTickCount();
    if ((int32_t)(nowT - last) > (int32_t)pdMS_TO_TICKS(2 * SAFETY_PERIOD_MS))
      last = nowT;

    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAFETY_PERIOD_MS));

    if (!s_armed) {                                   /* hwSetup still running */
      lastKick     = s_kicks;
      lastKickTick = xTaskGetTickCount();
      continue;
    }
    s_checks++;

    /* Wheel overspeed. Backstop for when the control task is not checking. */
    if (s_wheelVel) {
      float w = s_wheelVel();
      if (fabsf(w) > s_satLimit) {
        if (s_safeStop) s_safeStop("safety task: wheel overspeed");
        else            faults_safeStop(FAULT_WHEEL_SAT);
      }
    }

    /* Power rail. Each trip is debounced over 2 checks and latched so it fires
       once per excursion. Readings below SUPPLY_ABSENT_V mean the board is on USB
       with no motor supply, so they never trip undervoltage. */
    if (s_readPower) {
      float busV = 0.0f, mA = 0.0f;
      if (s_readPower(&busV, &mA)) {
        s_pwrSeen = true;
        if (busV < s_minV) s_minV = busV;
        if (busV > s_maxV) s_maxV = busV;
        if (fabsf(mA) > s_maxA) s_maxA = fabsf(mA);

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

    /* fanTask liveness. Only checked while fanRunning() is true (not during the
       arming ramp or after a hard kill). Wall-clock based and latched. */
    if (s_fanFrames && s_fanRunning) {
      uint32_t fr = s_fanFrames();
      TickType_t tf = xTaskGetTickCount();
      if (!s_fanRunning()) {
        lastFanFrame = fr; lastFanTick = tf; fanLatched = false;
      } else if (fr != lastFanFrame) {
        lastFanFrame = fr; lastFanTick = tf; fanLatched = false;
      } else if (!fanLatched &&
                 (tf - lastFanTick) > pdMS_TO_TICKS(FAN_STALL_TIMEOUT_MS)) {
        fanLatched = true;
        if (s_safeStop) s_safeStop("safety: fanTask stalled (DSHOT frames stopped)");
      }
    }

    /* Control-task heartbeat, measured in wall-clock time since the counter last
       moved (an iteration count would false-trip if this task were starved).
       A wedged control task cannot be recovered, so this is a terminal fault. */
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
