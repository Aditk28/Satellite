/*
  PURPOSE: Closed-loop platform HEADING control using the reaction wheel.
  This is the first real controller in the project -- everything before it
  (full.cpp, calibration.cpp) was open-loop bring-up and system ID.

  Flash this INSTEAD of calibration.cpp (PlatformIO builds one sketch at a
  time): move calibration.cpp to unflashed_files/, drop this into src/.
  MagneticSensorMT6701SSI.h/.cpp stay in src/ either way.

  ============================================================================
  THE CONTROL LAW, AND WHERE EVERY NUMBER CAME FROM
  ============================================================================

  Plant, after inverting the wheel loop (see below):
        theta_p'' = -a * alpha          a = J_w/J_p = 0.19

  Control law, executed every CONTROL_PERIOD_US:

    1. e     = wrap(target - theta)                         heading error
    2. alpha = -K_theta*e + K_omega*omega_p                 LQR / PD
    3. alpha += FF * friction_feedforward                   Coulomb cancel
    4. u     = (alpha + A_2*omega_w) / A_1                    feedback linearisation
    5. clamp u to +-VOLTAGE_LIMIT, write to motor.target

  SIGN WARNINGS -- both of these were gotten wrong at least once during
  design, and both produce plausible-looking but broken behaviour:

    (a) Step 2 has MINUS on the error and PLUS on the rate. The plant is
        theta'' = -a*alpha (wheel accelerates one way, platform the other),
        and that minus flips the sign of the damping term relative to a
        textbook PD. Using -(K_theta*e + K_omega*w) is UNSTABLE -- in
        simulation it diverged, ending 34 deg short of a 90 deg target with
        alpha blowing up past 600 rad/s^2.

    (b) The two friction feedforward branches have OPPOSITE signs.
        Derivation: we want the closed loop to behave as if friction were
        absent, i.e. theta'' = -a*alpha_lqr. With friction present,
              -a*(alpha_lqr + alpha_ff) - A_f*sign(omega_p) = -a*alpha_lqr
              => alpha_ff = -(A_f/a)*sign(omega_p)          [MOVING: negative]
        When the platform is STUCK there is no velocity to oppose; instead we
        push in whatever direction the controller wants to go, so the sign
        follows alpha:
              alpha_ff = +(A_f/a)*sign(alpha)               [STUCK: positive]
        Getting the moving branch backwards is WORSE THAN NO FEEDFORWARD --
        simulated final error 48 deg vs 27 deg with none, vs 0.7 deg correct.

  IDENTIFIED CONSTANTS (2026-07-30 calibration runs, 48 step tests + 46
  breakaway trials):

    A_1 = 40.56 rad/s^2 per V   wheel angular accel per volt from rest
                               = K_dc / tau_w = 7.3 / 0.18
    A_2 = 5.56  1/s             wheel damping pole = 1/tau_w
    a  = 0.19                  J_w/J_p, from regressing platform rate against
                               wheel-speed change over all 48 tests (20% scatter)
    A_FRICTION = 22.3 rad/s^2  wheel accel needed to break the platform free.
                               From the breakaway sweep: smallest step producing
                               net platform displacement was 0.55 V, and
                               0.19 * 40.56 * 0.55 = 4.24 rad/s^2 of PLATFORM
                               acceleration; divided by a gives the WHEEL accel
                               figure used here (4.24 / 0.19 = 22.3).
    GYRO_BIAS                  re-measured at every startup, not hardcoded --
                               it drifts thermally. ~0.42-0.46 dps historically.

  Not used here but identified: R = 0.788 ohm, Kv = 0.113 V/(rad/s). Those
  are physical motor parameters; the controller only needs the end-to-end
  A_1/A_2, which are immune to convention questions (phase vs line resistance,
  SimpleFOC modulation scaling) that would otherwise bite when converting
  to absolute torque units.

  GYRO SIGN: the raw gyro and the wheel encoder use OPPOSITE conventions --
  in raw data, wheel velocity and gyro rate come out positively correlated,
  which would imply both bodies turning the same way. They don't; it's a
  mounting/axis artifact. GYRO_SIGN below negates the gyro so that a positive
  wheel velocity corresponds to a negative platform rate, matching the
  theta'' = -a*alpha model. (This mirrors --flip-gyro in
  filter_calibration.py.)

  DEADZONE, AND WHY IT IS NOT OPTIONAL: with the feedforward always active,
  it never switches off once the platform has stopped, and keeps
  accelerating the wheel. Simulated 90 deg slew: no deadzone -> wheel ends
  at 72.9 rad/s; with a 2 deg deadzone -> 29.9 rad/s for the same final
  accuracy. Inside the deadzone we command zero and let stiction hold the
  platform.

  GAIN SELECTION: closed-loop natural frequency wn relates to the gains by
        wn^2 = a * K_theta      =>  K_theta = wn^2 / 0.19
        K_omega = 2*zeta*wn / a =>  K_omega = 1.4*wn / 0.19   (zeta = 0.7)
  and settle time ~ 4/(zeta*wn), so wn = 5.714 / t_settle.

        t_settle   wn     K_theta   K_omega
          3.0 s   1.90     19.1      14.0     <- START HERE
          2.0 s   2.86     43.0      21.1
          1.5 s   3.81     76.4      28.1
          1.2 s   4.76    119.3      35.1     <- near the ceiling

  The ceiling exists because the wheel pole sits at A_2 = 5.56 rad/s; above
  wn ~ 4.8 the feedback linearisation stops cancelling it cleanly. Faster
  gains are better on BOTH accuracy and wheel saturation (slow gains spend
  longer with feedforward active, winding the wheel up more) -- simulated
  90 deg slew, 2 deg deadzone: 3.0 s gains -> 6.30 deg error / 72.9 rad/s;
  1.5 s gains -> 0.03 deg error / 22.4 rad/s. So work DOWN the table.

  ============================================================================
  SERIAL COMMANDS (USB or HC-05, 115200, newline-terminated)
  ============================================================================
      T <deg>   set target heading and run a STEP CAPTURE (dumps CSV)
      H <deg>   hold this heading continuously (telemetry streams slowly)
      Z         zero the heading estimate here (defines "current = 0 deg")
      P <val>   set K_theta          D <val>   set K_omega
      F <val>   set feedforward fraction (0..1)
      W <deg>   set deadzone, degrees
      G         print current gains and state
      B         re-measure gyro bias (platform must be still)
      X         STOP -- motor to 0 V, controller disabled
      R         re-enable controller after X

  Any unrecognised input also triggers STOP, on the principle that a confused
  operator should not leave a flywheel spinning.

  CSV dumps use the same framing as calibration.cpp, so
  capture_calibration.py records them without modification.

  RTOS NOTE: this is a plain super-loop, deliberately, to keep first
  closed-loop bring-up simple. When merging into the FreeRTOS structure
  later, remember TIM2 and TIM3 are already claimed by motor PWM -- the
  control-loop timer must use TIM4, TIM5 or TIM9.

  BOARD: STM32 Nucleo-F446RE. Wiring identical to calibration.cpp.
*/

#include <SimpleFOC.h>
#include "MagneticSensorMT6701SSI.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ------------------------- hardware -------------------------
#define POLE_PAIRS      11
#define PIN_PWM_A       13
#define PIN_PWM_B       12
#define PIN_PWM_C       11
#define PIN_ENABLE      10
#define PIN_ENCODER_CS  PB1

#define HC05_EN_PIN     PC12
#define HC05_BAUD       115200
HardwareSerial hc05Serial(PC11, PC10);

#define VOLTAGE_LIMIT   10.0f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);
Adafruit_MPU6050 mpu;

// ------------------- identified plant constants -------------------
// NAMING NOTE: these are A_1 / A_2, not A1 / A2. On STM32duino, A0-A15 are
// predefined analog pin macros, so plain A1/A2 collide with the core headers.
// Same reason the control mode enum is CTRL_* rather than MODE_*.
static const float A_1          = 40.56f;   // rad/s^2 per V
static const float A_2          = 5.56f;    // 1/s
static const float A_FRICTION  = 22.3f;    // rad/s^2 (wheel) to break platform free
static const float GYRO_SIGN   = -1.0f;    // aligns gyro with wheel convention

// ------------------- runtime-tunable gains -------------------
// Start conservative (3.0 s row) and work down using P/D over serial --
// no reflash needed between tuning steps.
float K_theta   = 19.1f;
float K_omega   = 14.0f;
float ffFrac    = 0.85f;    // trim up until it overshoots, then back off
float deadzone  = 0.035f;   // rad (~2 deg)

// ------------------- safety -------------------
#define W_MOVING            0.05f    // rad/s, "platform is moving" threshold
#define WHEEL_SAT_LIMIT     45.0f    // rad/s -- abort above this
#define CONTROL_PERIOD_US   5000     // 200 Hz control loop
#define TELEM_PERIOD_MS     100      // 10 Hz streaming telemetry in HOLD

// ------------------- state -------------------
enum CtrlMode { CTRL_IDLE, CTRL_HOLD, CTRL_STEP, CTRL_OPEN };
volatile CtrlMode ctrlMode = CTRL_IDLE;

// Open-loop pulse: apply a fixed voltage for a fixed time with NO feedback.
// This exists to verify the gyro sign before ever closing the loop -- if
// GYRO_SIGN is wrong, a closed loop runs away instead of correcting, whereas
// an open-loop pulse is bounded no matter what the sign is.
float openVolts = 0.0f;
unsigned long openStartMs = 0;
#define OPEN_PULSE_MS 800

float theta      = 0.0f;    // estimated platform heading, rad
float omega_p    = 0.0f;    // platform rate, rad/s
float omega_w    = 0.0f;    // wheel rate, rad/s
float target     = 0.0f;    // commanded heading, rad
float gyroBias   = 0.0f;    // dps, measured at startup
float lastAlpha  = 0.0f;
float lastU      = 0.0f;
bool  controllerEnabled = true;

unsigned long lastControlUs = 0;
unsigned long lastTelemMs   = 0;

// ------------------- capture buffer -------------------
// 1500 samples at 200 Hz = 7.5 s, ample for a step response.
// 7 fields x 4 bytes = 28 bytes/sample -> ~41 KB, comfortable in 128 KB SRAM.
#define MAX_CAP 1500
unsigned long cap_t[MAX_CAP];
float cap_target[MAX_CAP], cap_theta[MAX_CAP], cap_wp[MAX_CAP];
float cap_ww[MAX_CAP], cap_alpha[MAX_CAP], cap_u[MAX_CAP];
int   capN = 0;
bool  capturing = false;
unsigned long capStartMs = 0;
#define CAPTURE_MS 6000

int stepCount = 0;

// =====================================================================
static inline float wrapPi(float x) {
  while (x >  PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  return x;
}
static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

void printBoth(const String &s) { Serial.println(s); hc05Serial.println(s); }

void stopMotor(const String &why) {
  controllerEnabled = false;
  ctrlMode = CTRL_IDLE;
  capturing = false;
  motor.target = 0.0f;
  motor.loopFOC();
  motor.move();
  printBoth("!! STOP: " + why + "  (send R to re-enable)");
}

// Platform must be stationary. Takes about a second.
void measureGyroBias() {
  const int N = 200;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z * 180.0f / PI;
    motor.loopFOC();
    motor.move();
    delay(5);
  }
  gyroBias = sum / N;
  printBoth("[GYRO] bias = " + String(gyroBias, 4) + " dps");
}

void printGains() {
  printBoth("K_theta=" + String(K_theta, 2) + "  K_omega=" + String(K_omega, 2)
            + "  ff=" + String(ffFrac, 2) + "  deadzone=" + String(degrees(deadzone), 2) + "deg");
  printBoth("theta=" + String(degrees(theta), 2) + "deg  target=" + String(degrees(target), 2)
            + "deg  omega_p=" + String(omega_p, 3) + "  omega_w=" + String(omega_w, 2)
            + "  mode=" + String(ctrlMode == CTRL_IDLE ? "IDLE" : (ctrlMode == CTRL_HOLD ? "HOLD" : "STEP")));
}

void dumpCaptureTo(Print &out) {
  out.print("--- capture start (test ");
  out.print(stepCount);
  out.print("/");
  out.print(stepCount);
  out.print(": heading step to ");
  out.print(degrees(target), 1);
  out.println(" deg) ---");
  out.print("mode=heading_step target_deg="); out.print(degrees(target), 3);
  out.print(" K_theta=");  out.print(K_theta, 3);
  out.print(" K_omega=");  out.print(K_omega, 3);
  out.print(" ff=");       out.print(ffFrac, 3);
  out.print(" deadzone_deg="); out.print(degrees(deadzone), 3);
  out.print(" gyro_bias_dps="); out.print(gyroBias, 4);
  out.println(" stop_reason=fixed_window");
  out.println("t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u");
  for (int i = 0; i < capN; i++) {
    out.print(cap_t[i]);                    out.print(",");
    out.print(degrees(cap_target[i]), 3);   out.print(",");
    out.print(degrees(cap_theta[i]), 3);    out.print(",");
    out.print(cap_wp[i], 4);                out.print(",");
    out.print(cap_ww[i], 3);                out.print(",");
    out.print(cap_alpha[i], 3);             out.print(",");
    out.println(cap_u[i], 4);
  }
  out.println("--- capture end ---");
}

void dumpCapture() { dumpCaptureTo(Serial); dumpCaptureTo(hc05Serial); }

// =====================================================================
// THE CONTROL LAW
// =====================================================================
void controlUpdate(float dt) {
  // ---- sense ----
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float gyro_dps = (g.gyro.z * 180.0f / PI) - gyroBias;
  omega_p = GYRO_SIGN * gyro_dps * PI / 180.0f;     // rad/s, model convention
  omega_w = motor.shaft_velocity;                    // rad/s, alignment-corrected

  // ---- estimate heading (gyro integration; vision correction later) ----
  theta = wrapPi(theta + omega_p * dt);

  if (!controllerEnabled || ctrlMode == CTRL_IDLE) {
    motor.target = 0.0f;
    lastAlpha = 0.0f; lastU = 0.0f;
    return;
  }

  // Open-loop pulse: no feedback at all, just hold a voltage for a moment.
  // Bounded regardless of whether the gyro sign is correct.
  if (ctrlMode == CTRL_OPEN) {
    if (millis() - openStartMs < OPEN_PULSE_MS) {
      motor.target = openVolts;
      lastU = openVolts;
      lastAlpha = 0.0f;
    } else {
      motor.target = 0.0f;
      lastU = 0.0f; lastAlpha = 0.0f;
    }
    return;
  }

  // ---- safety: wheel saturation ----
  if (fabsf(omega_w) > WHEEL_SAT_LIMIT) {
    stopMotor("wheel saturation " + String(omega_w, 1) + " rad/s");
    return;
  }

  // ---- 1. error ----
  float e = wrapPi(target - theta);

  // ---- 2. LQR / PD.  MINUS on error, PLUS on rate -- see header note (a). ----
  float alpha = -K_theta * e + K_omega * omega_p;

  // ---- 3. Coulomb feedforward, gated by the deadzone -- see header (b) ----
  if (fabsf(e) > deadzone) {
    float ff;
    if (fabsf(omega_p) > W_MOVING)
      ff = -A_FRICTION * signf(omega_p);   // MOVING: cancel friction opposing motion
    else
      ff =  A_FRICTION * signf(alpha);     // STUCK:  push to break free
    alpha += ffFrac * ff;
  } else {
    // Inside tolerance: command nothing and let stiction hold the platform.
    // This is what stops the wheel winding up indefinitely.
    alpha = 0.0f;
  }

  // ---- 4. feedback linearisation: cancel back-EMF and the wheel pole ----
  float u = (alpha + A_2 * omega_w) / A_1;

  // ---- 5. saturate and command ----
  if (u >  VOLTAGE_LIMIT) u =  VOLTAGE_LIMIT;
  if (u < -VOLTAGE_LIMIT) u = -VOLTAGE_LIMIT;
  motor.target = u;

  lastAlpha = alpha;
  lastU = u;
}

// =====================================================================
void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  float v = (s.length() > 1) ? s.substring(1).toFloat() : 0.0f;

  switch (c) {
    case 'O':
      // Open-loop sign check. Positive voltage should drive the wheel
      // positive, which by theta'' = -a*alpha must move theta NEGATIVE.
      // If theta goes positive instead, GYRO_SIGN is wrong -- flip it in
      // the source and reflash BEFORE closing the loop.
      openVolts = constrain(v, -3.0f, 3.0f);
      theta = 0.0f;
      capN = 0; capturing = true; capStartMs = millis();
      openStartMs = millis();
      stepCount++;
      ctrlMode = CTRL_OPEN; controllerEnabled = true;
      printBoth("OPEN-LOOP pulse " + String(openVolts, 2) + "V for "
                + String(OPEN_PULSE_MS) + "ms, capturing...");
      printBoth("  expect: wheel POSITIVE, theta NEGATIVE (for positive volts)");
      break;
    case 'T':
      target = wrapPi(radians(v));
      capN = 0; capturing = true; capStartMs = millis();
      stepCount++;
      ctrlMode = CTRL_STEP; controllerEnabled = true;
      printBoth("STEP -> " + String(v, 1) + " deg, capturing...");
      break;
    case 'H':
      target = wrapPi(radians(v));
      ctrlMode = CTRL_HOLD; controllerEnabled = true;
      printBoth("HOLD -> " + String(v, 1) + " deg");
      break;
    case 'Z':
      theta = 0.0f; target = 0.0f;
      printBoth("heading zeroed here");
      break;
    case 'P': K_theta = v; printGains(); break;
    case 'D': K_omega = v; printGains(); break;
    case 'F': ffFrac  = constrain(v, 0.0f, 1.5f); printGains(); break;
    case 'W': deadzone = radians(fabsf(v)); printGains(); break;
    case 'G': printGains(); break;
    case 'B':
      ctrlMode = CTRL_IDLE; motor.target = 0.0f;
      printBoth("hold still, measuring bias...");
      measureGyroBias();
      break;
    case 'X': stopMotor("operator"); break;
    case 'R':
      controllerEnabled = true; ctrlMode = CTRL_IDLE;
      printBoth("controller re-enabled (IDLE)");
      break;
    default:
      // Unrecognised input stops the motor rather than being ignored.
      stopMotor("unrecognised command '" + s + "'");
      break;
  }
}

void pollSerial() {
  static String bufU, bufB;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (bufU.length()) { handleLine(bufU); bufU = ""; } }
    else bufU += ch;
  }
  while (hc05Serial.available()) {
    char ch = hc05Serial.read();
    if (ch == '\n' || ch == '\r') { if (bufB.length()) { handleLine(bufB); bufB = ""; } }
    else bufB += ch;
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  Wire.begin();
  // 400 kHz fast mode. At the default 100 kHz each IMU read cost ~3.7 ms,
  // which dominated the loop during calibration. The control loop needs a
  // gyro read every cycle, so this directly sets the achievable rate.
  Wire.setClock(400000);

  if (!mpu.begin()) {
    printBoth("[MPU6050] NOT FOUND -- cannot run heading control.");
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  printBoth("[MPU6050] OK");

  sensor.init(&encoderSPI);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = VOLTAGE_PSU;
  driver.voltage_limit = VOLTAGE_LIMIT;
  if (!driver.init()) { printBoth("Driver init FAILED"); while (1) delay(1000); }
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.init();
  motor.initFOC();
  motor.target = 0.0f;

  printBoth("");
  printBoth("=== Heading controller ===");
  printBoth("Platform must be STILL for bias measurement.");
  measureGyroBias();
  theta = 0.0f; target = 0.0f;

  printGains();
  printBoth("FIRST: send O1 (open-loop pulse) to verify the gyro sign before closing the loop.");
  printBoth("Commands: O<V> openloop | T<deg> step+capture | H<deg> hold | Z zero | P/D gains");
  printBoth("          F<0-1> ff | W<deg> deadzone | G status | B rebias | X stop | R resume");
  printBoth("Start at K_theta=19.1 K_omega=14.0, then work down the gain table.");
  printBoth("Mode is IDLE -- send H0 or T<deg> to engage.");

  lastControlUs = micros();
}

void loop() {
  // FOC must run as fast as possible, every iteration, uninterrupted.
  motor.loopFOC();
  motor.move();

  pollSerial();

  unsigned long now = micros();
  if (now - lastControlUs >= CONTROL_PERIOD_US) {
    float dt = (now - lastControlUs) * 1e-6f;
    lastControlUs = now;

    controlUpdate(dt);

    if (capturing) {
      if (capN < MAX_CAP) {
        cap_t[capN]      = now;
        cap_target[capN] = target;
        cap_theta[capN]  = theta;
        cap_wp[capN]     = omega_p;
        cap_ww[capN]     = omega_w;
        cap_alpha[capN]  = lastAlpha;
        cap_u[capN]      = lastU;
        capN++;
      }
      if (capN >= MAX_CAP || (millis() - capStartMs) >= CAPTURE_MS) {
        capturing = false;
        dumpCapture();
        // After an OPEN-LOOP pulse, return to IDLE -- do NOT engage the
        // closed loop, since the whole point was to verify the sign first.
        bool wasOpen = (ctrlMode == CTRL_OPEN);
        ctrlMode = wasOpen ? CTRL_IDLE : CTRL_HOLD;
        if (wasOpen)
          printBoth("open-loop capture done, back to IDLE. Check: positive volts "
                    "should give POSITIVE wheel and NEGATIVE theta.");
        else
          printBoth("capture done, now HOLDing. Adjust gains and send T again.");
      }
    }
  }

  // Slow telemetry while holding, so you can nudge the platform by hand and
  // watch it recover without needing a full capture.
  if (ctrlMode == CTRL_HOLD && !capturing && (millis() - lastTelemMs) >= TELEM_PERIOD_MS) {
    lastTelemMs = millis();
    String s = String(degrees(theta), 2) + "," + String(degrees(target), 2) + ","
             + String(omega_p, 3) + "," + String(omega_w, 2) + ","
             + String(lastAlpha, 2) + "," + String(lastU, 3);
    Serial.println(s);
    hc05Serial.println(s);
  }
}