/*
  PURPOSE: Fully automated system-ID sweep for the reaction-wheel subsystem.
  NOT the run-time control sketch (that's full.cpp) -- flash this instead of
  it, run once per reset, re-run any time the control algorithm needs fresh
  data to retune against.

  REVISION NOTES (what changed from the fixed-window version, and why):

    1. CAPTURE UNTIL THE PLATFORM SETTLES, not a fixed sample count.
       Each test now runs in two phases: hold at `fromVoltage` for
       `holdBeforeStepMs`, then step to `toVoltage` -- and capture continues
       through BOTH phases and the decay afterward, stopping only once the
       platform's own gyro rate has been below GYRO_SETTLE_DPS for
       SETTLE_DEBOUNCE_N consecutive logged samples in a row (debounced so a
       single noisy near-zero reading mid-motion doesn't end the capture
       early). This is deliberately checking the PLATFORM's rate, not the
       wheel's -- the wheel can sit at a nonzero steady-state velocity
       indefinitely once torque disappears (holding a constant voltage
       produces no more torque once the wheel stops accelerating), while the
       platform is what actually decays back to zero under real friction.
       Conflating the two would either cut capture short (waiting on a wheel
       velocity that may never hit zero) or never stop. Two independent
       backstops still apply so a test can't run away: MAX_LOG_SAMPLES (hard
       buffer ceiling) and MAX_CAPTURE_MS (wall-clock cap on phase B).
       This is also what "keeps more friction information" -- the decay
       tail, which the old fixed 800/2000-sample window often cut off, is
       now captured in full.

    2. LOGGING IS NOW DECIMATED UNIFORMLY (LOG_DECIM), not "wheel fields at
       full rate, gyro decimated separately." The FOC loop itself
       (motor.loopFOC()/motor.move()) still runs every single iteration,
       full rate, undecimated -- only what gets STORED is decimated. This
       had to change because captures can now run for several seconds
       (through a full settle), and storing every field at full loop rate
       for that long would overflow the F446RE's 128KB SRAM. The trade-off:
       wheel-velocity samples are no longer at full loop rate. This costs
       essentially nothing here -- the motor's electrical time constant
       isn't observable through velocity alone anyway (no current sensing
       on this board), and the MECHANICAL time constants that are
       observable through wheel_velocity are almost certainly much slower
       than a few-hundred-Hz logging rate. Watch the realized log rate
       printed after each test and adjust LOG_DECIM/MAX_LOG_SAMPLES if it
       looks wrong for what you're trying to resolve.

    3. RELEASE AND REVERSAL TESTS are now first-class entries in
       TEST_SEQUENCE, not just an incidental thing that happens after a
       step test. Each entry is (fromVoltage, toVoltage, holdBeforeStepMs) --
       fromVoltage=0 gives a plain step-and-release test (with a small free
       baseline/pre-roll window at the start, useful as a zero-reference);
       fromVoltage!=0 spins the wheel up first, THEN steps to a DIFFERENT
       target (often 0 or negative) while already spinning, which is
       exactly what an angle-hold controller does routinely when
       correcting overshoot. Only characterizing 0->+V steps would leave
       zero data on crossing back through zero, which is exactly where
       gate-driver/motor nonlinearities (dead-time, minimum pulse width,
       cogging) tend to live.

    4. NOT IN THIS REVISION, ON PURPOSE: desaturation ramp-rate testing
       (slowly ramping voltage down at various SLOPES to find how slow a
       ramp friction can absorb "for free"). That's a genuinely different
       experiment from step-response system ID and is deferred until it's
       actually needed.

  WHAT THE DATA IS FOR:
    - Wheel-side unknowns (effective back-EMF constant, winding resistance,
      J_w, damping): fit these from (targetV, wheel_velocity) pairs across
      the step tests, e.g. via scipy.optimize.curve_fit against
      Uq = R*i + Kv*omega and the step-response rise time.
    - Reversal-test data: check for asymmetry/deadband crossing zero,
      separately from the linear-model fit above.
    - Platform inertia (J_p): do NOT derive this from momentum conservation
      in these transients -- friction here is already known to be higher
      than the ball-transfer-unit spec implied, so any energy lost to
      friction during the step shows up as error in a momentum-conservation
      -derived J_p. Measure J_p geometrically instead (mass + radius,
      I ~= 1/2*m*r^2, or a physical pendulum test).

  SAFETY / ABORT: type ANYTHING into either serial link (USB or HC-05) at
  any point and the sweep aborts immediately -- motor forced to 0V, sweep
  halted, requires a physical reset to run again.

  START GATE: WAIT_FOR_START_SIGNAL (on by default) makes the sweep pause
  after hardware bring-up and wait for any byte on either serial link (USB
  or HC-05) before starting -- gives you time to open a monitor / connect
  over Bluetooth and actually be watching before the wheel does anything.
  That waiting-to-start byte is consumed as a "go" signal, NOT an abort --
  once the sweep is actually running, incoming bytes go back to meaning
  abort (see SAFETY / ABORT above). Set the toggle to false to skip this
  and start immediately, e.g. for unattended reruns.

  HOW TO FLASH THIS: same convention already used elsewhere in this repo --
  move full.cpp out of src/ into unflashed_files/, drop this file into src/
  in its place, flash. MagneticSensorMT6701SSI.h/.cpp stay in src/ either
  way -- both sketches need them.

  SRAM NOTE: MAX_LOG_SAMPLES * 20 bytes/sample must comfortably fit in the
  F446RE's 128KB SRAM alongside everything else (SimpleFOC, Adafruit libs,
  stack). Defaults below (4000 samples, ~80KB) leave real margin -- if you
  raise MAX_LOG_SAMPLES further and the build warns/fails on RAM, targetV
  is the first field worth dropping (it's piecewise-constant and
  reconstructible from the phase-transition timestamp; kept here for
  convenience only).

  BOARD: STM32 Nucleo-F446RE. Pin mapping identical to full.cpp -- see that
  file's header for the full wiring writeup if needed.
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

// ======================= TUNABLE TEST PARAMETERS =======================

// Each entry: hold at fromVoltage for holdBeforeStepMs, then step to
// toVoltage, and capture continues (through both phases) until the
// platform settles. fromVoltage=0 => plain step-and-release test.
// fromVoltage!=0 => spin-up-then-transition: release-from-speed if
// toVoltage=0, true reversal if toVoltage is opposite sign.
struct TestStep {
  float fromVoltage;
  float toVoltage;
  unsigned long holdBeforeStepMs;
  const char* label;
};

const TestStep TEST_SEQUENCE[] = {
  // --- Plain step-and-release: spin-up + natural decay, 1V-4V (current cap) ---
  {0.0f,  1.0f, 1500, "step 0 -> 1.0V, then release to 0"},
  {0.0f,  1.5f, 1500, "step 0 -> 1.5V, then release to 0"},
  {0.0f,  2.5f, 1500, "step 0 -> 2.5V, then release to 0"},
  {0.0f,  4.0f, 1500, "step 0 -> 4.0V, then release to 0"},
  {0.0f, -1.0f, 1500, "step 0 -> -1.0V (negative direction), then release to 0"},

  // --- Release/reversal from an already-spinning state -- these take
  // longer to complete than a from-rest step of the same magnitude (a
  // reversal has to traverse roughly DOUBLE the velocity range: from
  // +ss all the way down through zero to -ss), which is why
  // holdBeforeStepMs and the MAX_CAPTURE_MS ceiling below both got more
  // headroom in this revision -- see the header note on negative voltage. ---
  {1.0f,  0.0f, 2000, "spin to 1.0V, then release to 0 (release-from-speed)"},
  {1.0f, -1.0f, 2000, "spin to 1.0V, then reverse to -1.0V"},
  {1.5f, -0.8f, 2000, "spin to 1.5V, then reverse to -0.8V"},
  {3.0f, -2.0f, 3000, "spin to 3.0V, then reverse to -2.0V"},
};
const int NUM_TESTS = sizeof(TEST_SEQUENCE) / sizeof(TEST_SEQUENCE[0]);

// Logging: a sample is stored every LOG_DECIM FOC-loop iterations. The FOC
// loop itself is never decimated -- only storage is. See revision note 2.
#define LOG_DECIM            20

// Hard ceiling on stored samples per test (SRAM budget -- see header).
// Raised from 2500 -> 4000 this revision: higher-voltage steps reach
// higher steady-state speeds (longer friction decay) and reversals now
// traverse roughly double the velocity range of a from-rest step, both of
// which need more samples to capture in full without hitting this ceiling
// early. At 20 bytes/sample this is ~80KB -- still comfortable margin
// inside the F446RE's 128KB SRAM.
#define MAX_LOG_SAMPLES     4000

// Wall-clock cap on phase B (post-transition) duration, regardless of
// whether settle was detected -- a backstop, not the normal stop condition.
// Raised from 8000 -> 12000ms this revision for the same reason as
// MAX_LOG_SAMPLES above -- reversals and high-voltage decays legitimately
// take longer.
#define MAX_CAPTURE_MS     12000

// Minimum time into phase B before settle-detection is even EVALUATED.
// Without this, small steps (1V, 1.5V...) can satisfy the debounce almost
// instantly -- the platform barely moved to begin with, or friction
// arrests it very fast, so the gyro rate is already under GYRO_SETTLE_DPS
// within the first handful of logged samples and the capture ends before
// there's much of a decay curve to look at. This forces every test to run
// for at least this long before settle is allowed to end it, regardless of
// how quickly the platform actually goes quiet.
#define MIN_CAPTURE_MS        500

// Platform considered "settled" below this gyro rate, in dps.
#define GYRO_SETTLE_DPS      3.0f

// Consecutive logged samples below GYRO_SETTLE_DPS required before
// declaring settle (debounced against transient near-zero noise).
#define SETTLE_DEBOUNCE_N       5

// --- Between-test cleanup: make sure BOTH the wheel AND the platform are
// back near rest before starting the next queued test. The wheel returning
// to ~0 rad/s does NOT mean the platform has stopped -- the release-to-0V
// at the end of the previous test (or a test that hit MAX_LOG_SAMPLES/
// MAX_CAPTURE_MS before truly settling) can leave the platform still
// coasting on its own residual momentum after the wheel itself is already
// still. Reuses GYRO_SETTLE_DPS/SETTLE_DEBOUNCE_N below -- same underlying
// condition as the in-test platform-settle detection, just used here as a
// gate before the next test instead of as a capture stop condition. ---
#define SETTLE_VEL_THRESH    1.0f   // rad/s -- wheel settle threshold
#define SETTLE_TIMEOUT_MS   15000

#define PRE_TEST_DELAY_MS    2000

// --- Start gate: wait for any byte on either serial link before the sweep
// starts, so you have a chance to open a monitor / connect over Bluetooth
// and be watching before the wheel does anything. Set to false to start
// immediately after hardware bring-up (e.g. unattended reruns). Note: this
// only applies BEFORE the sweep starts -- once a test is running, any
// serial byte means ABORT instead (see checkAbort()), not "start again." ---
#define WAIT_FOR_START_SIGNAL  true

// After the first "go" byte arrives, keep draining the buffer for this long
// before actually starting. A terminal like PuTTY sends a typed character
// and its trailing Enter (\r and/or \n) as SEPARATE bytes a few ms apart --
// draining only once, immediately, can miss the trailing byte(s), which
// then land in the buffer just as the sweep starts and get read by
// checkAbort() as an (unintended) abort. This window resets every time a
// new byte shows up, so it only returns once the link has been quiet.
#define START_FLUSH_MS        400
// =========================================================================

unsigned long log_time_us[MAX_LOG_SAMPLES];
float         log_targetV[MAX_LOG_SAMPLES];
float         log_vel_raw[MAX_LOG_SAMPLES];
float         log_vel_filt[MAX_LOG_SAMPLES];
float         log_gyroZ_dps[MAX_LOG_SAMPLES];
int samplesLogged = 0;

volatile bool aborted = false;

void printBoth(const String &s) {
  Serial.println(s);
  hc05Serial.println(s);
}

// Safety check -- call every loop iteration, including inside the tight
// capture loop. Serial.available()/hc05Serial.available() are cheap,
// non-blocking flag reads. Typing ANYTHING into either link is the kill
// switch, content doesn't matter.
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

void dumpBurstTo(Print &out, int testIndex, const TestStep &t, const String &stopReason) {
  out.print("--- capture start (test ");
  out.print(testIndex + 1);
  out.print("/");
  out.print(NUM_TESTS);
  out.print(": ");
  out.print(t.label);
  out.println(") ---");
  out.print("from=");   out.print(t.fromVoltage, 2);
  out.print("V to=");   out.print(t.toVoltage, 2);
  out.print("V hold=");  out.print(t.holdBeforeStepMs);
  out.print("ms stop_reason=");  out.println(stopReason);
  out.println("t_us,targetV,vel_raw,vel_filtered,gyroZ_dps");
  for (int i = 0; i < samplesLogged; i++) {
    out.print(log_time_us[i]);    out.print(",");
    out.print(log_targetV[i], 2); out.print(",");
    out.print(log_vel_raw[i], 3); out.print(",");
    out.print(log_vel_filt[i], 3);out.print(",");
    out.println(log_gyroZ_dps[i], 3);
  }
  out.println("--- capture end ---");
}

void dumpBurst(int testIndex, const TestStep &t, const String &stopReason) {
  dumpBurstTo(Serial, testIndex, t, stopReason);
  dumpBurstTo(hc05Serial, testIndex, t, stopReason);
}

// Waits for any byte on either serial link before letting the sweep start,
// if WAIT_FOR_START_SIGNAL is on. Unlike checkAbort(), this treats the
// incoming byte as a "go" signal, not an abort -- this function only runs
// BEFORE the sweep begins, so there's nothing to abort yet. Keeps the FOC
// loop serviced (target held at 0) the whole time it's waiting, same as
// everywhere else motor/driver are live.
void waitForStartSignal() {
  if (!WAIT_FOR_START_SIGNAL) return;

  // Clear out any stray bytes already buffered (e.g. a terminal sending a
  // newline on connect) so they don't get mistaken for the real "go".
  while (Serial.available())      Serial.read();
  while (hc05Serial.available())  hc05Serial.read();

  Serial.println("Hardware bring-up done. Send any character on Serial or HC-05 to start the sweep...");
  hc05Serial.println("Hardware bring-up done. Send any character to start the sweep...");

  motor.target = 0.0f;
  while (!Serial.available() && !hc05Serial.available()) {
    motor.loopFOC();
    motor.move();
  }

  // A typed character's trailing Enter (\r and/or \n) can arrive a few ms
  // AFTER the character itself -- draining once here and returning
  // immediately can miss it, leaving it to land in the buffer just as the
  // sweep starts and get picked up by checkAbort() as an unintended abort.
  // So: keep draining, and keep resetting the settle window every time a
  // new byte shows up, until the link has actually been quiet for
  // START_FLUSH_MS.
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

  printBoth("Starting sweep now.");
}

// Between-test cleanup: wait for the WHEEL (velocity) AND the PLATFORM
// (debounced gyro rate) to both settle before the next queued test starts.
// Not logged -- this is administrative, not itself an experiment.
bool waitForSettle() {
  unsigned long start = millis();
  unsigned long iterCount = 0;
  bool wheelSettled = false;
  int platformSettleCount = 0;

  while (millis() - start < SETTLE_TIMEOUT_MS) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    if (fabs(motor.shaft_velocity) < SETTLE_VEL_THRESH) {
      wheelSettled = true;
    }

    // Gyro read decimated the same way as in-test logging -- an I2C
    // transaction every single iteration would be wasted effort here,
    // and a few hundred Hz is plenty to debounce a decaying platform rate.
    iterCount++;
    if (iterCount % LOG_DECIM == 0) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float gyroZ_dps = g.gyro.z * 180.0f / PI;
      if (fabs(gyroZ_dps) < GYRO_SETTLE_DPS) {
        platformSettleCount++;
      } else {
        platformSettleCount = 0;
      }
    }

    if (wheelSettled && platformSettleCount >= SETTLE_DEBOUNCE_N) {
      printBoth("Wheel + platform both settled after " + String(millis() - start) + " ms.");
      return true;
    }
  }

  String wheelMsg    = wheelSettled ? String("settled")
                                     : ("still at " + String(motor.shaft_velocity, 2) + " rad/s");
  String platformMsg = (platformSettleCount >= SETTLE_DEBOUNCE_N) ? String("settled") : String("still moving");
  printBoth("!!! Settle timeout -- wheel " + wheelMsg + ", platform " + platformMsg
            + " -- proceeding anyway. Next test's data may show carryover motion. !!!");
  return true;
}

// Runs one test: hold at fromVoltage, step to toVoltage, capture
// continuously through both phases until the PLATFORM settles (debounced),
// hits MAX_LOG_SAMPLES, or hits MAX_CAPTURE_MS on phase B -- whichever
// comes first. Dumps the full capture, then does the between-test wheel
// cleanup above.
bool runOneTest(int testIndex) {
  const TestStep &t = TEST_SEQUENCE[testIndex];

  printBoth("");
  printBoth("=== Test " + String(testIndex + 1) + "/" + String(NUM_TESTS) + ": " + String(t.label) + " ===");
  printBoth("  hold " + String(t.fromVoltage, 2) + "V for " + String(t.holdBeforeStepMs)
            + " ms, then step to " + String(t.toVoltage, 2) + "V, capture until platform settles.");
  delay(PRE_TEST_DELAY_MS);
  if (checkAbort()) return false;

  samplesLogged = 0;
  unsigned long iterCount = 0;
  int settleCount = 0;
  bool inPhaseB = false;
  unsigned long phaseAStartMs = millis();
  unsigned long phaseBStartMs = 0;

  bool stoppedBySettle = false, stoppedByCeiling = false, stoppedByTimeout = false;

  motor.target = t.fromVoltage;

  while (true) {
    motor.loopFOC();
    motor.move();
    if (checkAbort()) return false;

    if (!inPhaseB && (millis() - phaseAStartMs >= t.holdBeforeStepMs)) {
      motor.target = t.toVoltage;
      inPhaseB = true;
      phaseBStartMs = millis();
      printBoth("  -> stepping to " + String(t.toVoltage, 2) + "V now, watching for platform settle...");
    }

    iterCount++;
    if (iterCount % LOG_DECIM == 0) {
      if (samplesLogged >= MAX_LOG_SAMPLES) {
        stoppedByCeiling = true;
        break;
      }

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float gyroZ_dps = g.gyro.z * 180.0f / PI;

      int i = samplesLogged;
      log_time_us[i]   = micros();
      log_targetV[i]   = motor.target;
      log_vel_raw[i]   = sensor.getVelocity();
      log_vel_filt[i]  = motor.shaft_velocity;
      log_gyroZ_dps[i] = gyroZ_dps;
      samplesLogged++;

      if (inPhaseB) {
        bool minTimeElapsed = (millis() - phaseBStartMs >= MIN_CAPTURE_MS);
        if (minTimeElapsed) {
          if (fabs(gyroZ_dps) < GYRO_SETTLE_DPS) {
            settleCount++;
            if (settleCount >= SETTLE_DEBOUNCE_N) {
              stoppedBySettle = true;
              break;
            }
          } else {
            settleCount = 0;
          }
        }

        if (millis() - phaseBStartMs >= MAX_CAPTURE_MS) {
          stoppedByTimeout = true;
          break;
        }
      }
    }
  }

  String stopReason = stoppedBySettle  ? "platform_settled" :
                       stoppedByCeiling ? "sample_buffer_full" :
                                          "capture_timeout";
  printBoth("  capture stopped: " + stopReason + " -- " + String(samplesLogged) + " samples logged.");
  if (samplesLogged >= 2) {
    float durationMs = (log_time_us[samplesLogged - 1] - log_time_us[0]) / 1000.0f;
    printBoth("  captured " + String(durationMs, 1) + " ms (realized log rate ~"
              + String(samplesLogged / (durationMs / 1000.0f), 0) + " Hz).");
  }
  if (stoppedByCeiling) {
    printBoth("  !!! Hit MAX_LOG_SAMPLES before settling -- raise it (watch SRAM) or shorten holdBeforeStepMs. !!!");
  }
  if (stoppedByTimeout) {
    printBoth("  !!! Hit MAX_CAPTURE_MS before settling -- data may not show the full decay. !!!");
  }

  dumpBurst(testIndex, t, stopReason);

  motor.target = 0.0f;   // ensure fully released regardless of how phase B ended
  printBoth("Target forced to 0V -- waiting for wheel and platform to settle before next test...");
  return waitForSettle();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(HC05_STATE_PIN, INPUT);
  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  Wire.begin();

  if (!mpu.begin()) {
    printBoth("[MPU6050] NOT FOUND -- check wiring / address (0x68)");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    printBoth("[MPU6050] OK");
  }

  if (!ina219.begin()) {
    printBoth("[INA219] NOT FOUND -- check wiring / address (0x40)");
  } else {
    printBoth("[INA219] OK (not logged in this sweep -- motor-side data is what matters here)");
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

  waitForStartSignal();  // placed after motor/FOC init since it keeps the
                          // FOC loop serviced (target=0) while it waits

  if (!aborted) {
    printBoth("=== Automated voltage-step calibration sweep (settle-based capture) ===");
    printBoth(String(NUM_TESTS) + " tests queued.");
    printBoth("Type anything into either serial link at any time to abort.");
    printBoth("");

    for (int t = 0; t < NUM_TESTS; t++) {
      if (!runOneTest(t)) {
        break;  // aborted, or a fault occurred mid-test
      }
    }
  }

  if (!aborted) {
    printBoth("");
    printBoth("=== Sweep complete -- all " + String(NUM_TESTS) + " tests done. Reset the board to run again. ===");
  }
}

void loop() {
  // The entire sweep runs once, inside setup(). Idle the FOC loop at 0V
  // afterward (also the state the abort path leaves things in).
  motor.target = 0.0f;
  motor.loopFOC();
  motor.move();
}