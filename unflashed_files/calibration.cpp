/*
  PURPOSE: Automated system-ID sweep for the reaction-wheel subsystem.
  NOT the run-time control sketch (that's full.cpp) -- flash this instead of
  it, run once per reset, re-run any time the control algorithm needs fresh
  data to retune against.

  ============================================================================
  REVISION NOTES -- what changed since the 2026-07-29 run, and why
  ============================================================================

  That run was usable (clean ~320 Hz logging, all tests ended on
  platform_settled, wheel response linear at 7.71 rad/s per volt), but
  analysis of it exposed one structural limit and several fixable gaps.
  All are addressed below.

  1. INA219 IS NOW LOGGED (bus voltage + current). THIS IS THE BIG ONE.
     Velocity-only data cannot separate R, Kt, Kv and friction b -- at
     steady state Uq = omega*(R*b/Kt + Kv), so the measured 7.71 rad/s per
     volt slope is ONE number produced by FOUR unknowns. No amount of extra
     voltage-step data fixes that; the information simply isn't in the
     velocity signal.

     The INA219 sits on the DC bus, not the phase wires, so it does NOT
     measure Iq directly. It helps via power balance instead: in voltage
     mode SimpleFOC drives Ud ~= 0, so essentially all electrical power is
     Uq*Iq, which must come from the bus:
           V_bus * I_bus  ~=  Uq * Iq     ->     Iq ~= V_bus*I_bus / Uq
     Uq is known (it's the commanded target) and V_bus/I_bus are now
     logged, so Iq is recoverable in post-processing. With Iq in hand,
     Uq = R*Iq + Kv*omega becomes one equation in two unknowns per test,
     and several voltage levels solve R and Kv separately by regression.
     Caveats worth remembering when fitting: this assumes Ud~=0 and
     neglects switching/conduction losses (a few percent), and it blows up
     near Uq=0 (division by ~zero) -- fine here since targetV is held at a
     constant nonzero value through phase B, but do not trust it at a
     reversal's zero crossing.

  2. WHEEL ANGLE IS NOW LOGGED DIRECTLY (sensor.getAngle()). Previously
     wheel angle was integrated from velocity in post-processing, which
     accumulates error. The encoder already provides absolute angle and is
     already read every FOC iteration, so this costs nothing. It also gives
     a better basis for computing wheel ACCELERATION, which is what sets
     the reaction torque on the platform (tau = -J_w * domega/dt).

  3. PHASE A NOW ENDS ON A SETTLE CONDITION, NOT A FIXED HOLD TIME.
     In the previous run, test 9's phase-A tail averaged +0.785 dps -- the
     platform was still ringing when the step fired, so that test's phase B
     started from a contaminated initial condition (tests 6 and 7 showed
     elevated noise too, std ~0.4 vs ~0.03 for the at-rest tests). Phase A
     now requires ALL THREE of: minimum hold elapsed, wheel velocity steady
     (its change between logged samples below a threshold, debounced), and
     platform gyro rate settled (debounced) -- with a timeout backstop so a
     never-settling rig can't hang the sweep.

  4. REPEATS. Each condition now runs N_REPEATS times, so the fit can carry
     a variance/repeatability figure instead of a single unqualified number.
     IMPORTANT ORDERING DETAIL: repeats are the OUTER loop, not the inner
     one -- the whole ordered condition list runs start to finish, then
     repeats. Running rep 1,2,3 of a condition back-to-back would confound
     any thermal/mechanical drift with that specific condition; spreading
     reps across the sweep averages drift over all conditions instead.

  5. SIGN-INTERLEAVED ORDERING. In the previous run the platform accumulated
     -365 degrees (a full turn) by test 4 before later tests unwound it back
     to -7.67 degrees net. Any tether attached to the platform therefore had
     DIFFERENT drag in early vs late tests, quietly confounding the
     comparison. Conditions are now ordered so consecutive tests alternate
     direction, which cancels rotation continuously rather than at the end.
     Net rotation per test and a running sweep total are printed for
     confirmation -- informational only, no threshold and no mid-sweep pause.

  6. MIRRORED NEGATIVE-DIRECTION TESTS. Previously there were four positive
     steps and only one negative, which showed ~5% asymmetry (-7.39 vs
     +7.76 rad/s at 1 V) that n=1 can't confirm. Every condition now has a
     sign-mirrored counterpart, which both tests that asymmetry properly
     and provides the alternating order item 5 needs.

  7. STICTION MEASUREMENT -- TWO MODES, ONE SUPERSEDED.
     MODE_STAIRCASE (voltage-level staircase) was the first attempt and does
     NOT work for platform breakaway. Reaction torque on the platform is
     proportional to WHEEL ACCELERATION; at a constant commanded voltage the
     wheel accelerates briefly then plateaus, so each tread delivers a torque
     impulse set by the STEP SIZE (a constant 0.05 V), not by the absolute
     voltage. Climbing the staircase applies the identical impulse at every
     rung. The 2026-07-30 run confirmed it: platform peak rate stayed flat at
     1.0-2.2 dps from 0.35 V to 0.80 V while wheel speed climbed 1.7 -> 6.4
     rad/s. That run also overran the sample buffer (~6700 samples needed vs
     2400 available) and stopped at 0.85 V without descending.

     It did find something real, though: the MOTOR's own deadband. The wheel
     does not turn at all below ~0.35 V (0.001-0.056 rad/s, i.e. noise), then
     jumps to 1.72 rad/s. Below that command the controller has NO authority,
     which is a genuine limit-cycle risk for any integral term.

     MODE_BREAKAWAY (new) sweeps STEP SIZE from rest instead, which is what
     actually scales torque delivered to the platform. Each trial dumps
     before the next starts, so the buffer holds one trial (~580 samples)
     rather than a whole sweep -- that's the structural fix for the overrun.

  8. SPLIT LOG RATE BY PHASE (LOG_DECIM_A vs LOG_DECIM_B). Phase A is now
     settle-gated and can run longer, but it's mostly steady-state holding
     and only needs enough resolution to confirm settling -- so it logs at
     ~107 Hz while phase B keeps the full ~320 Hz that proved adequate last
     run (54-63 samples on the rise). This is what pays for the two new
     columns: despite 28 bytes/sample vs the old 20, total SRAM use goes
     DOWN because phase A no longer burns samples at full rate.

  NOTE ON A RESIDUAL NONLINEARITY (not "fixed", just known): measured wheel
  time constant fell monotonically with voltage last run -- 196.7, 190.6,
  184.2, 177.9 ms at 1.0/1.5/2.5/4.0 V. A truly linear first-order system
  has constant tau. That ~10% drift is the signature of Coulomb (dry)
  friction in the wheel bearings, proportionally more significant at low
  speed. It doesn't invalidate a linear model near an operating point, but
  one global linear fit will be slightly wrong at the extremes. The new
  current data (item 1) is what will let you model it properly if you want
  to.

  ============================================================================

  WHAT THE DATA IS FOR:
    - Wheel-side parameters: with current now logged, R and Kv separate by
      regression across voltage levels (see item 1), and J_w/damping fit
      from the transient with Kt*Iq(t) as a MEASURED forcing term rather
      than an assumed step shape. The motor's electrical time constant
      (L/R, typically microseconds to a few ms) is far faster than the
      ~180-200 ms mechanical tau, so Iq(t) tracks (Uq - Kv*omega(t))/R
      essentially instantly -- the electrical dynamics can be treated as
      quasi-static in the fit.
    - Reversal tests: check for asymmetry/deadband crossing zero, now with
      mirrored conditions so the check is symmetric.
    - Staircase mode: breakaway (static friction) threshold for the
      platform -- the controller deadband.
    - Platform inertia (J_p): still do NOT derive this from momentum
      conservation in these transients -- friction is higher than the
      ball-transfer-unit spec implied, so friction losses show up as error
      in a momentum-derived J_p. Measure it geometrically instead
      (mass + radius, I ~= 1/2*m*r^2, or a physical pendulum test).

  SAFETY / ABORT: type ANYTHING into either serial link (USB or HC-05) at
  any point and the sweep aborts immediately -- motor forced to 0V, sweep
  halted, requires a physical reset to run again.

  PROMPTS AND ABORT DON'T MIX WELL -- READ THIS BEFORE ADDING ONE: because
  any stray byte means abort, a mid-sweep prompt that waits for input must
  drain the link completely before returning, or the trailing Enter from the
  keypress that answered the prompt lands in the buffer a few ms later and
  aborts the sweep it just resumed. A previous revision hit exactly this.
  ALWAYS use waitForKeypress() for prompts -- it waits, then drains until
  the link has been quiet for START_FLUSH_MS. Never hand-roll a
  wait-then-drain-once.

  START GATE: WAIT_FOR_START_SIGNAL (on by default) makes the sweep pause
  after hardware bring-up and wait for any byte on either serial link
  before starting. That byte is consumed as "go", NOT an abort -- once the
  sweep is running, incoming bytes go back to meaning abort.

  HOW TO FLASH THIS: move full.cpp out of src/ into unflashed_files/, drop
  this file into src/ in its place, flash. MagneticSensorMT6701SSI.h/.cpp
  stay in src/ either way -- both sketches need them.

  SRAM NOTE: MAX_LOG_SAMPLES * 28 bytes/sample must fit in the F446RE's
  128KB SRAM alongside SimpleFOC, the Adafruit libs, and stack. The default
  2400 samples is ~66KB (~51%), leaving real margin. If you raise it and
  the build warns on RAM, busV is the first field worth dropping -- on a
  bench supply it barely moves, and Iq recovery is far more sensitive to
  current than to bus voltage.

  BOARD: STM32 Nucleo-F446RE. Pin mapping identical to full.cpp -- see that
  file's header for the full wiring writeup.
*/

#include <SimpleFOC.h>
#include "MagneticSensorMT6701SSI.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>

#define POLE_PAIRS      11

#define PIN_PWM_A       13
#define PIN_PWM_B       12
#define PIN_PWM_C       11
#define PIN_ENABLE      10
#define PIN_ENCODER_CS  PB1

#define HC05_EN_PIN     PC12
#define HC05_STATE_PIN  PC0
#define HC05_BAUD       115200
HardwareSerial hc05Serial(PC11, PC10);

#define VOLTAGE_LIMIT   10.0f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);

Adafruit_MPU6050 mpu;
Adafruit_INA219 ina219;

// ======================= SWEEP MODE =======================
// MODE_STEP      -- voltage-step / reversal system ID (the main sweep)
// MODE_STAIRCASE -- slow voltage staircase. KEPT FOR REFERENCE BUT SUPERSEDED
//                   BY MODE_BREAKAWAY: it cannot measure platform breakaway.
//                   Reaction torque on the platform is proportional to WHEEL
//                   ACCELERATION, and at a constant commanded voltage the
//                   wheel accelerates briefly then plateaus. Each tread
//                   therefore delivers a torque impulse proportional to the
//                   STEP SIZE (a constant 0.05 V), not to the absolute
//                   voltage -- so climbing the staircase applies the same
//                   impulse at every rung and nothing changes. The
//                   2026-07-30 run confirmed this directly: platform peak
//                   rate stayed flat at 1.0-2.2 dps from 0.35 V all the way
//                   to 0.80 V while wheel speed climbed 1.7 -> 6.4 rad/s.
//                   It IS still useful for the motor's own deadband: that
//                   run found the wheel does not turn at all below ~0.35 V.
// MODE_BREAKAWAY -- sweeps STEP SIZE from rest, which is what actually scales
//                   the torque delivered to the platform. This is the mode to
//                   use for platform stiction.
#define MODE_STEP       0
#define MODE_STAIRCASE  1
#define MODE_BREAKAWAY  2
#define SWEEP_MODE      MODE_BREAKAWAY
// ==========================================================

// ======================= TUNABLE TEST PARAMETERS =======================

struct TestStep {
  float fromVoltage;
  float toVoltage;
  unsigned long minHoldMs;   // floor on phase A; it then waits for settle
  const char* label;
};

// Ordered so consecutive entries alternate direction -- this is what keeps
// accumulated platform rotation (and therefore cable twist) near zero
// throughout the sweep instead of only at the end. Keep that alternation if
// you edit this list.
const TestStep TEST_SEQUENCE[] = {
  // --- From-rest steps, sign-mirrored pairs ---
  {0.0f,  1.0f, 1200, "step 0 -> +1.0V"},
  {0.0f, -1.0f, 1200, "step 0 -> -1.0V"},
  {0.0f,  1.5f, 1200, "step 0 -> +1.5V"},
  {0.0f, -1.5f, 1200, "step 0 -> -1.5V"},
  {0.0f,  2.5f, 1200, "step 0 -> +2.5V"},
  {0.0f, -2.5f, 1200, "step 0 -> -2.5V"},
  {0.0f,  4.0f, 1200, "step 0 -> +4.0V"},
  {0.0f, -4.0f, 1200, "step 0 -> -4.0V"},

  // --- Release from speed, sign-mirrored ---
  { 1.0f, 0.0f, 1500, "spin +1.0V -> release to 0"},
  {-1.0f, 0.0f, 1500, "spin -1.0V -> release to 0"},

  // --- Reversals through zero, sign-mirrored ---
  { 1.0f, -1.0f, 1500, "spin +1.0V -> reverse to -1.0V"},
  {-1.0f,  1.0f, 1500, "spin -1.0V -> reverse to +1.0V"},
  { 1.5f, -0.8f, 1500, "spin +1.5V -> reverse to -0.8V"},
  {-1.5f,  0.8f, 1500, "spin -1.5V -> reverse to +0.8V"},
  { 3.0f, -2.0f, 2000, "spin +3.0V -> reverse to -2.0V"},
  {-3.0f,  2.0f, 2000, "spin -3.0V -> reverse to +2.0V"},
};
const int NUM_CONDITIONS = sizeof(TEST_SEQUENCE) / sizeof(TEST_SEQUENCE[0]);

// Repeats run as the OUTER loop (whole list, then repeat) so drift is
// spread across conditions rather than confounded with any single one.
#define N_REPEATS               3

// --- Logging rates. The FOC loop itself is NEVER decimated -- only what
// gets stored is. Phase A logs slower because it's mostly steady-state
// holding and only needs enough resolution to confirm settling; phase B
// keeps the rate that proved adequate last run. ---
#define LOG_DECIM_A            60      // ~107 Hz  (phase A / hold)
#define LOG_DECIM_B            20      // ~320 Hz  (phase B / transient)

#define MAX_LOG_SAMPLES      2400      // ~66KB at 28 bytes/sample

// --- Phase A exit conditions: ALL must hold (plus minHoldMs elapsed) ---
#define WHEEL_STEADY_DELTA     0.15f   // rad/s change between logged samples
#define WHEEL_STEADY_N            8    // consecutive samples below that
#define PHASE_A_TIMEOUT_MS     9000    // backstop if it never settles

// --- Phase B stop conditions ---
#define MIN_CAPTURE_MS          500    // settle not even evaluated before this
#define MAX_CAPTURE_MS        12000    // wall-clock backstop
#define GYRO_SETTLE_DPS         3.0f   // platform "settled" below this
#define SETTLE_DEBOUNCE_N          5   // consecutive samples required

// --- Between-test cleanup (wheel AND platform must both come to rest) ---
#define SETTLE_VEL_THRESH       1.0f   // rad/s
#define SETTLE_TIMEOUT_MS      15000

#define PRE_TEST_DELAY_MS       1500

// --- Cable twist guard. Sign-alternating order should keep this near zero;
// this catches the case where it doesn't (e.g. a condition list edited out
// of alternation, or badly asymmetric response). ---

// --- Start gate ---
#define WAIT_FOR_START_SIGNAL   true
#define START_FLUSH_MS           400   // quiet window before any prompt
                                        // returns -- see waitForKeypress()

// --- Staircase mode parameters (only used when SWEEP_MODE==MODE_STAIRCASE) ---
// NOTE: this mode is superseded for platform stiction -- see the mode notes
// above. If you do run it, be aware the full up+down sweep is ~50 treads and
// at the MEASURED 168 Hz log rate that's ~6700 samples, which overruns
// MAX_LOG_SAMPLES by ~2.8x (the 2026-07-30 run stopped at 0.85 V and never
// descended). Shorten the range or the dwell if you need it.
#define STAIR_STEP_V           0.05f   // increment per tread
#define STAIR_MAX_V            1.20f   // climb to here, then back down
#define STAIR_DWELL_MS           800   // hold at each tread
#define STAIR_LOG_DECIM           60

// --- Breakaway mode parameters (SWEEP_MODE==MODE_BREAKAWAY) ---
// Sweeps STEP SIZE from rest. Each trial: settle at 0 V, step to S, watch for
// a fixed window, return to 0, wait for rest, dump. Because each trial dumps
// before the next begins, the buffer only ever holds ONE trial (~580 samples
// at the measured 168/224 Hz rates) -- comfortably inside MAX_LOG_SAMPLES.
// That is the structural difference from the staircase, which tried to hold
// an entire 40 s sweep in one buffer.
//
// Range rationale: the motor's own deadband is ~0.35 V (below that the wheel
// doesn't turn, so no torque reaches the platform at all), and a 1.0 V step
// visibly moves the platform (~28 dps peak in the main sweep). So platform
// breakaway is bracketed between those. Starting at 0.05 V re-confirms the
// motor deadband in the same run for free.
#define BREAK_MIN_V            0.05f
#define BREAK_MAX_V            1.20f
#define BREAK_STEP_V           0.05f
#define BREAK_REPEATS              2   // near threshold this is stochastic
#define BREAK_MIN_HOLD_MS        800   // floor on the at-rest hold before stepping
#define BREAK_CAPTURE_MS        1500   // FIXED capture window after the step
// Fixed, not settle-stopped, on purpose: below breakaway the platform never
// moves, so a settle-stop would fire immediately and capture nothing. A fixed
// window gives every trial the same observation period, which is what makes
// "did it move" comparable across step sizes.
// =========================================================================

unsigned long log_time_us[MAX_LOG_SAMPLES];
float log_targetV[MAX_LOG_SAMPLES];
float log_wheel_vel[MAX_LOG_SAMPLES];
float log_wheel_angle[MAX_LOG_SAMPLES];
float log_gyroZ_dps[MAX_LOG_SAMPLES];
float log_current_mA[MAX_LOG_SAMPLES];
float log_busV[MAX_LOG_SAMPLES];
int samplesLogged = 0;
int phaseBStartSample = 0;   // index of the first phase-B sample in the log

volatile bool aborted = false;
float sweepNetRotationDeg = 0.0f;   // running cable-twist estimate
bool mpuOK = false, inaOK = false;

// Measured once at startup with the platform at rest. Used ONLY for the
// cable-twist accumulator below -- the logged gyroZ_dps column stays raw,
// so post-processing can measure and remove bias itself (and cross-check
// against this value, which is reported in the sweep header).
// This matters: the bias is small (~0.4 dps) but it integrates. Left in,
// it would add roughly two degrees of phantom rotation per five-second
// capture, which across a full sweep is enough to trip the twist warning
// on a rig whose cable is actually fine.
float gyroBiasDps = 0.0f;

void printBoth(const String &s) {
  Serial.println(s);
  hc05Serial.println(s);
}

// Safety check -- call every loop iteration, including inside tight capture
// loops. Serial.available() is a cheap non-blocking flag read. Typing
// ANYTHING into either link is the kill switch; content doesn't matter.
bool checkAbort() {
  if (aborted) return true;
  if (Serial.available() || hc05Serial.available()) {
    while (Serial.available())      Serial.read();
    while (hc05Serial.available())  hc05Serial.read();
    aborted = true;
    motor.target = 0.0f;
    motor.loopFOC();
    motor.move();
    printBoth("!!! ABORTED by serial input -- motor set to 0V. Reset the board to run again. !!!");
  }
  return aborted;
}

// One decimated sensor read + log-row write. Returns the gyro rate so
// callers can use it for settle detection without re-reading I2C.
// Both the MPU6050 and the INA219 are read here, in the same decimated
// slot -- these are the slow I2C transactions, which is exactly why they're
// decimated rather than run every FOC iteration.
float logSample() {
  float gyroZ_dps = 0.0f;
  float current_mA = 0.0f, busV = 0.0f;

  if (mpuOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ_dps = g.gyro.z * 180.0f / PI;
  }
  if (inaOK) {
    busV = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
  }

  if (samplesLogged < MAX_LOG_SAMPLES) {
    int i = samplesLogged;
    log_time_us[i]     = micros();
    log_targetV[i]     = motor.target;
    log_wheel_vel[i]   = motor.shaft_velocity;
    log_wheel_angle[i] = sensor.getAngle();
    log_gyroZ_dps[i]   = gyroZ_dps;
    log_current_mA[i]  = current_mA;
    log_busV[i]        = busV;
    samplesLogged++;
  }
  return gyroZ_dps;
}

void dumpTo(Print &out, const String &header, const String &meta) {
  out.println(header);
  out.println(meta);
  out.println("t_us,targetV,wheel_vel,wheel_angle_rad,gyroZ_dps,current_mA,busV");
  for (int i = 0; i < samplesLogged; i++) {
    out.print(log_time_us[i]);          out.print(",");
    out.print(log_targetV[i], 3);       out.print(",");
    out.print(log_wheel_vel[i], 3);     out.print(",");
    out.print(log_wheel_angle[i], 4);   out.print(",");
    out.print(log_gyroZ_dps[i], 3);     out.print(",");
    out.print(log_current_mA[i], 2);    out.print(",");
    out.println(log_busV[i], 3);
  }
  out.println("--- capture end ---");
}

void dumpBoth(const String &header, const String &meta) {
  dumpTo(Serial, header, meta);
  dumpTo(hc05Serial, header, meta);
}

// Measure the gyro's at-rest bias. Call once at startup with the platform
// genuinely stationary and the motor idle. Takes about a second.
void measureGyroBias() {
  if (!mpuOK) return;
  const int N = 200;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z * 180.0f / PI;
    motor.loopFOC();
    motor.move();
    delay(5);
  }
  gyroBiasDps = sum / N;
  printBoth("[GYRO] at-rest bias measured: " + String(gyroBiasDps, 4)
            + " dps (removed from the twist accumulator; logged data stays raw)");
}

// Report the net platform rotation of the capture just finished, and keep a
// running total for the sweep. Informational only -- no threshold, no
// warning, no pause. Sign-alternating condition order (see the test list) is
// what actually keeps accumulated rotation near zero; this just reports it
// so you can confirm that's working.
void reportNetRotation() {
  float deg = 0.0f;
  for (int i = 1; i < samplesLogged; i++) {
    float dt = (log_time_us[i] - log_time_us[i-1]) / 1e6f;
    float g0 = log_gyroZ_dps[i-1] - gyroBiasDps;
    float g1 = log_gyroZ_dps[i]   - gyroBiasDps;
    deg += (g1 + g0) * 0.5f * dt;
  }
  sweepNetRotationDeg += deg;
  printBoth("  net rotation this test: " + String(deg, 1)
            + " deg | sweep total: " + String(sweepNetRotationDeg, 1) + " deg");
}

// Between-test cleanup: wheel velocity AND platform gyro rate must both be
// at rest before the next test starts. The wheel reaching ~0 does NOT mean
// the platform has stopped -- it can still be coasting on residual
// momentum. Not logged; this is administrative, not an experiment.
bool waitForRest() {
  unsigned long start = millis();
  unsigned long iterCount = 0;
  bool wheelRest = false;
  int platformCount = 0;

  while (millis() - start < SETTLE_TIMEOUT_MS) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    if (fabs(motor.shaft_velocity) < SETTLE_VEL_THRESH) wheelRest = true;

    iterCount++;
    if (iterCount % LOG_DECIM_A == 0 && mpuOK) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float gyroZ = g.gyro.z * 180.0f / PI;
      if (fabs(gyroZ) < GYRO_SETTLE_DPS) platformCount++;
      else platformCount = 0;
    }

    if (wheelRest && platformCount >= SETTLE_DEBOUNCE_N) {
      printBoth("  wheel + platform at rest after " + String(millis() - start) + " ms.");
      return true;
    }
  }
  printBoth("  !!! Rest timeout -- wheel " + String(wheelRest ? "ok" : "still moving")
            + ", platform " + String(platformCount >= SETTLE_DEBOUNCE_N ? "ok" : "still moving")
            + " -- proceeding anyway. !!!");
  return true;
}

// ===================== STEP / REVERSAL TEST =====================
// Phase A: hold at fromVoltage until minHoldMs elapsed AND the wheel is
// steady AND the platform has settled (all three), then step.
// Phase B: capture through the transient until the platform settles again.
bool runStepTest(int condIdx, int rep, int testNum, int testTotal) {
  const TestStep &t = TEST_SEQUENCE[condIdx];

  printBoth("");
  printBoth("=== Test " + String(testNum) + "/" + String(testTotal)
            + " (rep " + String(rep) + "/" + String(N_REPEATS) + "): " + String(t.label) + " ===");
  printBoth("  hold " + String(t.fromVoltage, 2) + "V (min " + String(t.minHoldMs)
            + " ms, then until steady), step to " + String(t.toVoltage, 2) + "V");
  delay(PRE_TEST_DELAY_MS);
  if (checkAbort()) return false;

  samplesLogged = 0;
  phaseBStartSample = 0;
  unsigned long iterCount = 0;

  // ---------------- PHASE A ----------------
  motor.target = t.fromVoltage;
  unsigned long phaseAStart = millis();
  float lastWheelVel = motor.shaft_velocity;
  int wheelSteadyCount = 0, platformSettleCount = 0;
  bool phaseAClean = false;

  while (true) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    iterCount++;
    if (iterCount % LOG_DECIM_A == 0) {
      if (samplesLogged >= MAX_LOG_SAMPLES) break;
      float gyroZ = logSample();

      float v = motor.shaft_velocity;
      if (fabs(v - lastWheelVel) < WHEEL_STEADY_DELTA) wheelSteadyCount++;
      else wheelSteadyCount = 0;
      lastWheelVel = v;

      if (fabs(gyroZ) < GYRO_SETTLE_DPS) platformSettleCount++;
      else platformSettleCount = 0;

      bool minElapsed = (millis() - phaseAStart >= t.minHoldMs);
      if (minElapsed && wheelSteadyCount >= WHEEL_STEADY_N
                     && platformSettleCount >= SETTLE_DEBOUNCE_N) {
        phaseAClean = true;
        break;
      }
    }

    if (millis() - phaseAStart >= PHASE_A_TIMEOUT_MS) break;
  }

  if (!phaseAClean) {
    printBoth("  !!! Phase A did not fully settle before timeout -- phase B's initial "
              "condition is contaminated for this test. !!!");
  }
  phaseBStartSample = samplesLogged;

  // ---------------- PHASE B ----------------
  motor.target = t.toVoltage;
  unsigned long phaseBStart = millis();
  int settleCount = 0;
  bool stoppedBySettle = false, stoppedByCeiling = false, stoppedByTimeout = false;
  printBoth("  -> stepped to " + String(t.toVoltage, 2) + "V, capturing transient...");

  while (true) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    iterCount++;
    if (iterCount % LOG_DECIM_B == 0) {
      if (samplesLogged >= MAX_LOG_SAMPLES) { stoppedByCeiling = true; break; }
      float gyroZ = logSample();

      if (millis() - phaseBStart >= MIN_CAPTURE_MS) {
        if (fabs(gyroZ) < GYRO_SETTLE_DPS) {
          settleCount++;
          if (settleCount >= SETTLE_DEBOUNCE_N) { stoppedBySettle = true; break; }
        } else settleCount = 0;
      }
      if (millis() - phaseBStart >= MAX_CAPTURE_MS) { stoppedByTimeout = true; break; }
    }
  }

  String stopReason = stoppedBySettle ? "platform_settled"
                     : stoppedByCeiling ? "sample_buffer_full" : "capture_timeout";

  float durMs = samplesLogged >= 2
              ? (log_time_us[samplesLogged-1] - log_time_us[0]) / 1000.0f : 0.0f;
  printBoth("  stopped: " + stopReason + " -- " + String(samplesLogged)
            + " samples (A=" + String(phaseBStartSample)
            + " B=" + String(samplesLogged - phaseBStartSample) + ") over "
            + String(durMs, 0) + " ms");
  if (stoppedByCeiling)
    printBoth("  !!! Hit MAX_LOG_SAMPLES before settling -- raise it (watch SRAM). !!!");
  if (stoppedByTimeout)
    printBoth("  !!! Hit MAX_CAPTURE_MS before settling -- decay may be truncated. !!!");

  String header = "--- capture start (test " + String(testNum) + "/" + String(testTotal)
                + ": " + String(t.label) + " [rep " + String(rep) + "/" + String(N_REPEATS) + "]) ---";
  String meta = "from=" + String(t.fromVoltage, 2) + "V to=" + String(t.toVoltage, 2)
              + "V rep=" + String(rep) + " phaseB_start_sample=" + String(phaseBStartSample)
              + " phaseA_clean=" + String(phaseAClean ? "yes" : "no")
              + " gyro_bias_dps=" + String(gyroBiasDps, 4)
              + " stop_reason=" + stopReason;
  dumpBoth(header, meta);

  reportNetRotation();

  motor.target = 0.0f;
  printBoth("  target 0V -- waiting for wheel and platform to rest...");
  return waitForRest();
}

// ===================== STICTION STAIRCASE =====================
// Climbs in small voltage treads to find where the platform first breaks
// free of static friction, then descends to find where it re-sticks (these
// are not the same voltage -- that gap is the hysteresis you care about).
// Logged as one continuous capture; find the breakaway in post-processing
// by looking for where gyro rate first leaves the noise floor.
bool runStaircase() {
  printBoth("");
  printBoth("=== STICTION STAIRCASE ===");
  printBoth("  " + String(STAIR_STEP_V, 3) + "V treads, " + String(STAIR_DWELL_MS)
            + " ms dwell, up to " + String(STAIR_MAX_V, 2) + "V and back down");
  printBoth("  Find breakaway in post: first tread where gyro rate leaves the noise floor.");
  delay(PRE_TEST_DELAY_MS);
  if (checkAbort()) return false;

  samplesLogged = 0;
  phaseBStartSample = 0;
  unsigned long iterCount = 0;

  int nUp = (int)(STAIR_MAX_V / STAIR_STEP_V + 0.5f);
  // Climb, then descend back through the same treads.
  for (int dir = 0; dir < 2; dir++) {
    for (int k = 0; k <= nUp; k++) {
      int tread = (dir == 0) ? k : (nUp - k);
      motor.target = tread * STAIR_STEP_V;
      unsigned long dwellStart = millis();

      while (millis() - dwellStart < STAIR_DWELL_MS) {
        motor.loopFOC();
        motor.move();
        if (checkAbort()) return false;
        iterCount++;
        if (iterCount % STAIR_LOG_DECIM == 0) {
          if (samplesLogged >= MAX_LOG_SAMPLES) {
            printBoth("  !!! Sample buffer full mid-staircase -- raise MAX_LOG_SAMPLES, "
                      "or widen STAIR_STEP_V / shorten STAIR_DWELL_MS. !!!");
            goto done;
          }
          logSample();
        }
      }
    }
  }
done:
  motor.target = 0.0f;

  float durMs = samplesLogged >= 2
              ? (log_time_us[samplesLogged-1] - log_time_us[0]) / 1000.0f : 0.0f;
  printBoth("  staircase complete -- " + String(samplesLogged) + " samples over "
            + String(durMs, 0) + " ms");

  String header = "--- capture start (test 1/1: stiction staircase) ---";
  String meta = "mode=staircase step_v=" + String(STAIR_STEP_V, 3)
              + " max_v=" + String(STAIR_MAX_V, 2)
              + " dwell_ms=" + String(STAIR_DWELL_MS)
              + " stop_reason=staircase_complete";
  dumpBoth(header, meta);

  reportNetRotation();
  return waitForRest();
}

// ===================== BREAKAWAY (STEP-SIZE SWEEP) =====================
// One trial: settle at 0 V, step to stepV, observe for a FIXED window,
// return to 0, wait for rest. Records the peak platform rate and peak wheel
// speed reached, which is what makes breakaway visible without any
// post-processing.
//
// Why step size rather than voltage level: reaction torque on the platform
// is proportional to wheel ACCELERATION, and a step from rest to stepV
// produces an acceleration proportional to stepV. Holding a constant voltage
// (what MODE_STAIRCASE does) produces acceleration only transiently, and its
// magnitude depends on the change in command, not its absolute value -- which
// is why that mode's platform response was flat across the whole sweep.
bool runBreakawayTrial(float stepV, int rep, int trialNum, int trialTotal,
                        float &peakGyroOut, float &peakWheelOut) {
  printBoth("");
  printBoth("=== Breakaway trial " + String(trialNum) + "/" + String(trialTotal)
            + ": step 0 -> " + String(stepV, 2) + "V  [rep " + String(rep)
            + "/" + String(BREAK_REPEATS) + "] ===");
  delay(PRE_TEST_DELAY_MS);
  if (checkAbort()) return false;

  samplesLogged = 0;
  phaseBStartSample = 0;
  unsigned long iterCount = 0;

  // ---- Phase A: sit at 0 V until the platform is genuinely still ----
  motor.target = 0.0f;
  unsigned long phaseAStart = millis();
  int platformSettleCount = 0;
  bool phaseAClean = false;

  while (true) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    iterCount++;
    if (iterCount % LOG_DECIM_A == 0) {
      if (samplesLogged >= MAX_LOG_SAMPLES) break;
      float gyroZ = logSample();
      if (fabs(gyroZ) < GYRO_SETTLE_DPS) platformSettleCount++;
      else platformSettleCount = 0;

      if ((millis() - phaseAStart >= BREAK_MIN_HOLD_MS)
          && platformSettleCount >= SETTLE_DEBOUNCE_N) {
        phaseAClean = true;
        break;
      }
    }
    if (millis() - phaseAStart >= PHASE_A_TIMEOUT_MS) break;
  }
  if (!phaseAClean)
    printBoth("  !!! Platform not settled before the step -- this trial's result is suspect. !!!");

  phaseBStartSample = samplesLogged;

  // ---- Phase B: step, observe for a fixed window ----
  motor.target = stepV;
  unsigned long phaseBStart = millis();
  float peakGyro = 0.0f, peakWheel = 0.0f;

  while (millis() - phaseBStart < BREAK_CAPTURE_MS) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    iterCount++;
    if (iterCount % LOG_DECIM_B == 0) {
      if (samplesLogged >= MAX_LOG_SAMPLES) break;
      float gyroZ = logSample();
      if (fabs(gyroZ) > fabs(peakGyro)) peakGyro = gyroZ;
      if (fabs(motor.shaft_velocity) > fabs(peakWheel)) peakWheel = motor.shaft_velocity;
    }
  }

  peakGyroOut = peakGyro;
  peakWheelOut = peakWheel;

  printBoth("  peak platform rate " + String(peakGyro, 2) + " dps"
            + "   peak wheel " + String(peakWheel, 2) + " rad/s"
            + (fabs(peakGyro) > GYRO_SETTLE_DPS ? "   <-- PLATFORM MOVED" : ""));

  String header = "--- capture start (test " + String(trialNum) + "/" + String(trialTotal)
                + ": breakaway step 0 -> " + String(stepV, 2) + "V [rep " + String(rep)
                + "/" + String(BREAK_REPEATS) + "]) ---";
  String meta = "mode=breakaway from=0.00V to=" + String(stepV, 2)
              + "V rep=" + String(rep)
              + " phaseB_start_sample=" + String(phaseBStartSample)
              + " phaseA_clean=" + String(phaseAClean ? "yes" : "no")
              + " peak_gyro_dps=" + String(peakGyro, 3)
              + " peak_wheel_rads=" + String(peakWheel, 3)
              + " gyro_bias_dps=" + String(gyroBiasDps, 4)
              + " stop_reason=fixed_window";
  dumpBoth(header, meta);

  reportNetRotation();

  motor.target = 0.0f;
  return waitForRest();
}

bool runBreakawaySweep() {
  int nSteps = (int)((BREAK_MAX_V - BREAK_MIN_V) / BREAK_STEP_V + 0.5f) + 1;
  int total = nSteps * BREAK_REPEATS;

  printBoth("=== Breakaway sweep (step-size) ===");
  printBoth("  " + String(nSteps) + " step sizes from " + String(BREAK_MIN_V, 2)
            + "V to " + String(BREAK_MAX_V, 2) + "V x " + String(BREAK_REPEATS)
            + " repeats = " + String(total) + " trials");
  printBoth("  Watch for the first step size where platform rate clears "
            + String(GYRO_SETTLE_DPS, 1) + " dps.");
  printBoth("  Type anything into either serial link at any time to abort.");

  // Summary is accumulated and printed at the end so the breakaway point is
  // readable in one place, without scrolling back through every trial dump.
  static float sumPeak[64];
  static int   sumCount[64];
  for (int i = 0; i < 64; i++) { sumPeak[i] = 0.0f; sumCount[i] = 0; }

  int trial = 0;
  for (int rep = 1; rep <= BREAK_REPEATS; rep++) {
    for (int k = 0; k < nSteps; k++) {
      float stepV = BREAK_MIN_V + k * BREAK_STEP_V;
      trial++;
      float pg = 0.0f, pw = 0.0f;
      if (!runBreakawayTrial(stepV, rep, trial, total, pg, pw)) return false;
      if (k < 64) { sumPeak[k] += fabs(pg); sumCount[k]++; }
    }
  }

  printBoth("");
  printBoth("=== BREAKAWAY SUMMARY (mean peak platform rate per step size) ===");
  printBoth("  step_V, mean_peak_dps, moved");
  float firstMoved = -1.0f;
  for (int k = 0; k < nSteps && k < 64; k++) {
    if (!sumCount[k]) continue;
    float mean = sumPeak[k] / sumCount[k];
    bool moved = mean > GYRO_SETTLE_DPS;
    if (moved && firstMoved < 0) firstMoved = BREAK_MIN_V + k * BREAK_STEP_V;
    printBoth("  " + String(BREAK_MIN_V + k * BREAK_STEP_V, 2) + ", "
              + String(mean, 2) + ", " + String(moved ? "yes" : "no"));
  }
  if (firstMoved >= 0)
    printBoth("  -> platform breakaway step size ~ " + String(firstMoved, 2) + " V");
  else
    printBoth("  -> platform never moved. Raise BREAK_MAX_V and rerun.");
  return true;
}

// ===================== START GATE =====================
// Wait for any byte on either serial link, then drain the link COMPLETELY
// before returning. Use this for EVERY mid-sweep prompt -- never hand-roll a
// wait-then-drain-once, which is the bug this exists to prevent.
//
// Why the settle window: a typed character's trailing Enter (\r and/or \n)
// arrives a few ms AFTER the character itself. Draining once and returning
// immediately misses it, so it lands in the buffer moments later and
// checkAbort() reads it as an unintended abort -- the sweep dies right after
// the prompt that was supposed to resume it. So: keep draining, and reset
// the quiet window on every new byte, until the link has actually gone
// silent for START_FLUSH_MS.
//
// The FOC loop is kept serviced (whatever target is currently set) the whole
// time, since the driver stays live throughout.
void waitForKeypress() {
  while (!Serial.available() && !hc05Serial.available()) {
    motor.loopFOC();
    motor.move();
  }
  unsigned long lastByteMs = millis();
  while (millis() - lastByteMs < START_FLUSH_MS) {
    motor.loopFOC();
    motor.move();
    if (Serial.available() || hc05Serial.available()) {
      while (Serial.available())      Serial.read();
      while (hc05Serial.available())  hc05Serial.read();
      lastByteMs = millis();
    }
  }
}

void waitForStartSignal() {
  if (!WAIT_FOR_START_SIGNAL) return;

  while (Serial.available())      Serial.read();
  while (hc05Serial.available())  hc05Serial.read();

  Serial.println("Hardware bring-up done. Send any character on Serial or HC-05 to start the sweep...");
  hc05Serial.println("Hardware bring-up done. Send any character to start the sweep...");

  motor.target = 0.0f;
  waitForKeypress();

  printBoth("Starting sweep now.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(HC05_STATE_PIN, INPUT);
  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  Wire.begin();
  // 400 kHz fast mode. Measured on the 2026-07-30 runs: each logged sample
  // costs ~3.7 ms of I2C (MPU6050 + two INA219 reads) against a ~27 kHz bare
  // FOC loop, making I2C by far the dominant per-sample expense -- it's why
  // phase B logged at 224 Hz rather than the 320 Hz its decimation implies.
  // Both sensors support 400 kHz. The same reads will sit inside the
  // real-time control task later, so the headroom matters there too.
  Wire.setClock(400000);

  mpuOK = mpu.begin();
  if (!mpuOK) {
    printBoth("[MPU6050] NOT FOUND -- check wiring / address (0x68). Settle detection "
              "and platform data will not work.");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    printBoth("[MPU6050] OK");
  }

  inaOK = ina219.begin();
  if (!inaOK) {
    printBoth("[INA219] NOT FOUND -- current/voltage will log as 0. R and Kv cannot be "
              "separated without it (see header note 1).");
  } else {
    ina219.setCalibration_32V_2A();
    printBoth("[INA219] OK -- logging bus voltage and current");
  }

  sensor.init(&encoderSPI);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = VOLTAGE_PSU;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) {
    printBoth("Driver init FAILED -- check pin/timer mapping.");
    while (1) { delay(1000); }
  }
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = VOLTAGE_LIMIT;

  motor.init();
  motor.initFOC();
  motor.target = 0.0f;

  // Platform must be stationary here -- this runs before the start gate, so
  // it happens while you're still setting up rather than mid-sweep.
  measureGyroBias();

  waitForStartSignal();

  if (!aborted) {
#if SWEEP_MODE == MODE_BREAKAWAY
    runBreakawaySweep();
#elif SWEEP_MODE == MODE_STAIRCASE
    printBoth("=== Stiction staircase mode ===");
    printBoth("NOTE: this mode cannot measure PLATFORM breakaway (see the mode");
    printBoth("notes at the top of this file). Use MODE_BREAKAWAY for that.");
    printBoth("Type anything into either serial link at any time to abort.");
    runStaircase();
#else
    int total = NUM_CONDITIONS * N_REPEATS;
    printBoth("=== Voltage-step system-ID sweep ===");
    printBoth(String(NUM_CONDITIONS) + " conditions x " + String(N_REPEATS)
              + " repeats = " + String(total) + " tests.");
    printBoth("Repeats run as the outer loop (full list, then repeat) so drift spreads "
              "across conditions instead of confounding one.");
    printBoth("Type anything into either serial link at any time to abort.");
    printBoth("");

    int testNum = 0;
    bool ok = true;
    for (int rep = 1; rep <= N_REPEATS && ok; rep++) {
      for (int c = 0; c < NUM_CONDITIONS && ok; c++) {
        testNum++;
        ok = runStepTest(c, rep, testNum, total);
      }
    }
#endif
  }

  if (!aborted) {
    printBoth("");
    printBoth("=== Sweep complete. Reset the board to run again. ===");
    printBoth("Final accumulated platform rotation: " + String(sweepNetRotationDeg, 1) + " deg");
  }
}

void loop() {
  // The whole sweep runs once, inside setup(). Idle the FOC loop at 0V
  // afterward (also the state the abort path leaves things in).
  motor.target = 0.0f;
  motor.loopFOC();
  motor.move();
}