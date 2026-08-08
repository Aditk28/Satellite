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
    4. u     = (alpha + compFrac*A_2*omega_w) / A_1           feedback linearisation
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

    A_1 = 45.5 rad/s^2 per V   wheel angular accel per volt from rest
                               = K_dc / tau_w = 8.51 / 0.187   [re-ID 2026-08-02]
    A_2 = 5.35  1/s            wheel damping pole = 1/tau_w    [re-ID 2026-08-02]
    compFrac = 0.85            fraction of A_2 actually applied. The
                               linearisation is only as good as A_2; over-
                               compensation makes the wheel loop UNSTABLE
                               while under-compensation is merely sluggish,
                               so this deliberately sits below 1.0. Verify
                               with C<V> -- the wheel must coast DOWN.
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
#include "timebase.h"
#include "hw_timers.h"
#include "timing_stats.h"
#include <STM32FreeRTOS.h>
#include "faults.h"
#include "trace.h"           // TRACE_ID_CTRL for the task's trace index

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
// RE-IDENTIFIED 2026-08-02 from six O(+/-3V) open-loop captures. The original
// values (A_1 = 40.56, A_2 = 5.56, from the 2026-07-30 step campaign) made the
// feedback linearisation OVER-compensate: measured DC gain came out 8.51
// (rad/s)/V against the modelled 7.3, which left a residual pole of
// +0.89 /s in the wheel loop -- unstable, and the cause of the first
// closed-loop runaway to wheel saturation.
//   K'   = 8.51 (rad/s)/V   (2.3% scatter over 6 tests)
//   tau' = 0.187 s          (spin-up 0.181, decay 0.193)
//   A_1  = K'/tau' = 45.5   A_2 = 1/tau' = 5.35
static const float A_1          = 45.5f;    // rad/s^2 per V
static const float A_2          = 5.35f;    // 1/s
// RUNTIME TUNABLE (command A). Was a fixed 22.3, computed as 4.24/0.19 -- but
// `a` is nearer 0.15, which makes the true break-free figure 4.24/0.15 = 28.3.
// Running 22.3 left feedforward ~25% light: large errors had enough PD to cover
// the gap, small ones did not, and a correction from rest at 5 deg error cleared
// breakaway by only 10% -- a coin flip on stiction, and the cause of the
// intermittent -90/-180 stalls. Tune with A<val> rather than measuring `a`:
// raise until small corrections close reliably, lower if it overshoots.
float A_FRICTION  = 28.3f;    // rad/s^2 (wheel) to break platform free
static const float GYRO_SIGN   = -1.0f;    // aligns gyro with wheel convention

// ------------------- control loop timing -------------------
static stat_t st_foc, st_move, st_mpu, st_ina, st_law, st_telem, st_period;

// ------------------- runtime-tunable gains -------------------
// Start conservative (3.0 s row) and work down using P/D over serial --
// no reflash needed between tuning steps.
// Tuned on hardware 2026-08-02 (1.2 s row). A 148 deg slew settled to 0.79 deg
// with a 14-34 rad/s wheel peak -- better than the simulated table predicted.
float K_theta   = 119.3f;
float K_omega   = 35.1f;
float ffFrac    = 0.90f;    // trim up until it overshoots, then back off
// Fraction of the modelled A_2 actually applied in the linearisation.
// 1.0 = trust the model exactly. Below 1.0 deliberately UNDER-compensates.
// This asymmetry is the whole point: under-compensation leaves a stable
// residual pole (mildly sluggish), over-compensation leaves an unstable one
// (exponential runaway). Always err low. Verify with the C command.
//
// MEASURED 2026-08-02 by the C sweep (7 trials, both directions):
//     positive:  pole = 3.00*cf - 2.92   ->  neutral at cf = 0.972
//     negative:  pole = 3.68*cf - 3.28   ->  neutral at cf = 0.892
// Note cf = 1.0 is ALREADY UNSTABLE positive-going even with the re-identified
// A_1/A_2, so the true damping pole is nearer 5.0 than 5.35. Rather than chase
// A_2 again, the margin lives here where it is visible.
// 0.89 = essentially AT the lower measured neutral (0.892). Chosen over the
// initially-suggested 0.80 because under-compensation is not free: the shortfall
// is subtracted from every commanded alpha, and at 0.80 a sustained alpha died
// out after ~1.9 s, so slews ran out of torque mid-manoeuvre. 0.89 gives a
// ~4 s torque hold. Measured residual pole at this value: about -0.84 /s.
// Do NOT make this direction-dependent: the measured asymmetry REVERSED sign
// between those two runs, so a per-direction value would be wrong half the time.
float compFrac  = 0.89f;
float deadzone  = 0.035f;   // rad (2.0 deg) COARSE -- used while the wheel is
                            // still fast and full torque is unavailable
// FINE deadzone, used only once the wheel has unwound below FINE_WW, where the
// controller has its full authority back. This is the "reduced-speed, tightened-
// deadband terminal approach" from Phase 9 of the project plan, and it is what
// makes sub-degree pointing reachable: a fine correction needs the wheel SLOW,
// because delivered alpha = commanded - 0.84*omega_w.
// Must stay ABOVE the achievable Coulomb deadband (1-ffFrac)*A_FRICTION/K_theta,
// or feedforward never switches off and the wheel winds indefinitely.
//   ff = 0.90 -> 1.36 deg floor    ff = 0.95 -> 0.68 deg    ff = 0.97 -> 0.41 deg
// 2.0 deg. Was 1.0 and that was a BUG: at ffFrac = 0.90 and K_theta = 119.3 the
// floor is (1-0.90)*22.3/119.3 = 1.07 deg nominal, ~1.36 deg using the true
// friction figure -- so 1.0 deg was UNREACHABLE. The controller kept pushing,
// feedforward never switched off, and the wheel wound back up into a stall.
// At 2.0 deg both stages are equal, i.e. a single 2 deg tolerance, which is the
// configuration that produced the clean 15-step envelope (1.20 deg mean error).
// To go tighter you must raise ffFrac FIRST -- see the floor table above.
float deadzoneFine = 0.035f;
#define FINE_WW         5.0f   // rad/s, wheel slow enough for a fine correction

// ------------------- safety -------------------
#define W_MOVING            0.05f    // rad/s, "platform is moving" threshold

// ---- momentum management ----------------------------------------------
// DESATURATION IS PASSIVE -- there is deliberately no active unwind routine.
// Inside the deadzone alpha = 0, so u = compFrac*A_2*omega_w/A_1 = 0.105*omega_w,
// while merely holding speed needs 0.123*omega_w. The shortfall bleeds the wheel
// down as  d(omega_w)/dt = -0.84*omega_w,  a 1.2 s exponential.
//
// The important property is that it is SELF-LIMITING: reaction torque during the
// unwind is 0.19*0.84*omega_w = 0.16*omega_w rad/s^2 on the platform, which stays
// under the 4.24 rad/s^2 breakaway for any omega_w below 26.5 rad/s. So the wheel
// comes home and friction holds the heading, with no logic and no tuning.
//
// An active ramp was tried and REMOVED. Commanding alpha = -10 to brake gives
// u = (-10 + 0.89*5.35*20)/45.5 = 1.87 V at 20 rad/s, against a 2.46 V hold
// voltage -- so the wheel actually decelerated at 27 rad/s^2, not 10. That is
// 5.1 rad/s^2 on the platform, ABOVE breakaway: the platform broke free, the
// heading drifted, the controller fought back, and the wheel wound up. The
// compensation shortfall that starves a positive alpha ADDS to a negative one.
// Do not command braking torque through the linearisation at speed.
//
// STALL is the one case passive unwind cannot reach: parked OUTSIDE the deadzone
// with the wheel already fast, where the wheel cannot deliver the commanded alpha
// and pushing harder only winds toward the abort (see the 120 deg and 179 deg
// slews, 2026-08-02). Recovery is simply to stop commanding for a moment and let
// the passive unwind restore authority, then retry.
// NOTE: above omega_w = 26.5 the passive unwind torque (0.16*omega_w) does
// exceed the 4.24 breakaway, so a slew ending fast may drag the heading a little
// on the way down. An active ramp to cover that band was written and removed --
// it is not currently needed, and braking through the linearisation is actively
// dangerous (the compensation shortfall ADDS to a negative alpha, so a commanded
// -10 rad/s^2 came out as -27). If it is ever revisited, it must command VOLTAGE
// directly from the wheel's hold curve (u_hold = omega_w/K_HOLD, K_HOLD ~ 8.13
// (rad/s)/V measured at 42 rad/s), never a negative alpha.
#define ALPHA_STALL_MAX     55.0f    // rad/s^2, cap while platform is stationary
#define STALL_WW            25.0f    // rad/s -- was 30; lowered so a stuck
                                     // platform is caught before the wheel has
                                     // enough momentum to abort violently
#define STALL_MS             600     // ms of continuous stall before backing off
#define STALL_HOLD_MS       2000     // ms of alpha = 0 before retrying
#define WW_MAX_JUMP         15.0f    // rad/s per control cycle -- above this is a
                                     // velocity-estimate glitch, not real (physical
                                     // max ~2.3 rad/s/cycle). Rejected in sense.
#define WHEEL_SAT_LIMIT     45.0f    // rad/s -- abort above this.
                                     // Keep at 45: a C3 spin-up alone reaches
                                     // ~26 rad/s, so a lower limit aborts the
                                     // comp test before it starts. Use C2 if
                                     // you want a lower-speed check.
#define CONTROL_PERIOD_US   5000     // 200 Hz control loop
#define TELEM_PERIOD_MS     100      // 10 Hz streaming telemetry in HOLD
#define FOC_RATE_HZ         4000     // TIM9 FOC tick (Step 0.5 / Appendix B4)
#define CTRL_DIVISOR        20       // 4000 / 20 = 200 Hz control (== CONTROL_PERIOD_US)

// ------------------- state -------------------
enum CtrlMode { CTRL_IDLE, CTRL_HOLD, CTRL_STEP, CTRL_OPEN, CTRL_COMP };
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
unsigned long wwRejects = 0;// count of rejected wheel-velocity glitches (see sense)
float target     = 0.0f;    // commanded heading, rad
float gyroBias   = 0.0f;    // dps, measured at startup
float lastAlpha  = 0.0f;
bool  stallHold = false;
unsigned long stallStartMs = 0;
unsigned long stallHoldUntil = 0;
int   stallCount = 0;         // consecutive stalls against the same target
bool  parked = false;         // gave up retrying; hold until a new target
#define MAX_STALL_RETRIES 3
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
  stallHold = false; stallStartMs = 0; parked = false; stallCount = 0;
  motor.target = 0.0f;               // focTask applies this within one tick (<=250 us)
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
    delay(5);                        // focTask commutates via the TIM9 ISR meanwhile
  }
  gyroBias = sum / N;
  printBoth("[GYRO] bias = " + String(gyroBias, 4) + " dps");
}

// The smallest heading error the controller can actually close, set by the
// friction that feedforward does not cancel. Any deadzone below this can never
// be reached: the controller pushes forever and the wheel winds up.
float deadbandFloorDeg() {
  return degrees((1.0f - ffFrac) * A_FRICTION / K_theta);
}

void printGains() {
  printBoth("K_theta=" + String(K_theta, 2) + "  K_omega=" + String(K_omega, 2)
            + "  ff=" + String(ffFrac, 2) + "  deadzone=" + String(degrees(deadzone), 2)
            + "/" + String(degrees(deadzoneFine), 2) + "deg"
            + "  compFrac=" + String(compFrac, 2)
            + (stallHold ? "  [STALL-HOLD]" : "")
            + (parked ? "  [PARKED]" : ""));
  printBoth("A_FRICTION=" + String(A_FRICTION, 1)
            + "  ff*A_F=" + String(ffFrac * A_FRICTION, 1)
            + "  (break-free from rest needs about 28)");
  float floorDeg = deadbandFloorDeg();
  printBoth("deadband floor=" + String(floorDeg, 2) + "deg"
            + (degrees(deadzoneFine) < floorDeg
               ? "  <-- WARNING: fine deadzone is BELOW the floor, raise N or F"
               : "  (both deadzones clear it)"));
  printBoth("theta=" + String(degrees(theta), 2) + "deg  target=" + String(degrees(target), 2)
            + "deg  omega_p=" + String(omega_p, 3) + "  omega_w=" + String(omega_w, 2)
            + "  mode=" + String(ctrlMode == CTRL_IDLE ? "IDLE" : (ctrlMode == CTRL_HOLD ? "HOLD" : "STEP")));
  printBoth("wheel-velocity glitches rejected: " + String(wwRejects));
  // RTOS (Step 2.1, Trap 3): the single task never blocks, so the stack-overflow
  // hook (checked only at switch time) never runs. Read the high-water mark by
  // hand -- the MINIMUM free stack (in words) the ctrl task has ever had.
  printBoth("ctrl stack free (min words): "
            + String((uint32_t)uxTaskGetStackHighWaterMark(NULL)));
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
  TIME_BLOCK(st_mpu, { mpu.getEvent(&a, &g, &t); });
  float gyro_dps = (g.gyro.z * 180.0f / PI) - gyroBias;
  omega_p = GYRO_SIGN * gyro_dps * PI / 180.0f;     // rad/s, model convention

  // Wheel velocity, with a physical-plausibility reject. The wheel cannot change
  // speed faster than ~A_1*V_max = ~455 rad/s^2 = ~2.3 rad/s per 5 ms cycle, so a
  // single-cycle jump beyond WW_MAX_JUMP is a velocity-ESTIMATE glitch (noise on
  // the SSI read, coupled from the MPU I2C traffic on nearby wiring) -- hold the
  // last good value rather than act on it or false-trip WHEEL_SAT. The streak cap
  // resyncs if the "spike" persists, so a real level change is never stuck out.
  float w_w_raw = motor.shaft_velocity;              // rad/s, alignment-corrected
  static float w_w_prev = 0.0f;
  static bool  w_w_init = false;
  static int   w_w_rejStreak = 0;
  if (w_w_init && fabsf(w_w_raw - w_w_prev) > WW_MAX_JUMP && w_w_rejStreak < 3) {
    w_w_raw = w_w_prev;                              // reject an isolated glitch...
    w_w_rejStreak++;
    wwRejects++;
  } else {
    w_w_rejStreak = 0;                               // ...but resync if it persists
  }
  w_w_prev = w_w_raw; w_w_init = true;
  omega_w = w_w_raw;

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

  // Compensation-only test (C command). Spin the wheel up open-loop, then
  // command alpha = 0 with the back-EMF cancellation term still active.
  // The wheel MUST coast down. If it holds speed the compensation is exactly
  // neutral (no margin); if it accelerates, compFrac is too high and the
  // closed loop will run away. This isolates the inner loop from the gains.
  if (ctrlMode == CTRL_COMP) {
    if (fabsf(omega_w) > WHEEL_SAT_LIMIT) {
      stopMotor("wheel saturation " + String(omega_w, 1) + " rad/s");
      return;
    }
    float u;
    if (millis() - openStartMs < OPEN_PULSE_MS) {
      u = openVolts;                              // spin-up, open loop
    } else {
      u = compFrac * A_2 * omega_w / A_1;         // alpha = 0
    }
    if (u >  VOLTAGE_LIMIT) u =  VOLTAGE_LIMIT;
    if (u < -VOLTAGE_LIMIT) u = -VOLTAGE_LIMIT;
    motor.target = u;
    lastU = u; lastAlpha = 0.0f;
    return;
  }

  // ---- safety: wheel saturation. Hard stop, deliberately. The platform will
  //      spin as the wheel dumps its momentum; that is accepted behaviour. ----
  if (fabsf(omega_w) > WHEEL_SAT_LIMIT) {
    stopMotor("wheel saturation " + String(omega_w, 1) + " rad/s");
    dumpCapture();   // DIAG: dump omega_w trajectory up to the abort (ramp vs spike)
    return;
  }

  // ---- 1. error ----
  float e = wrapPi(target - theta);
  // Two-stage tolerance: coarse while the wheel still holds momentum, fine once
  // it has bled down and full torque is available again.
  float dz = (fabsf(omega_w) < FINE_WW) ? deadzoneFine : deadzone;
  bool outside   = fabsf(e) > dz;
  bool notMoving = fabsf(omega_p) < W_MOVING;

  // ---- 1b. stall detection: outside the deadzone, platform stationary, wheel
  //      already fast. Back off and let the passive unwind restore authority. ----
  if (outside && notMoving && fabsf(omega_w) > STALL_WW) {
    if (stallStartMs == 0) stallStartMs = millis();
    else if (millis() - stallStartMs > STALL_MS && !stallHold && !parked) {
      stallHold = true;
      stallHoldUntil = millis() + STALL_HOLD_MS;
      stallCount++;
      if (stallCount > MAX_STALL_RETRIES) {
        // Retrying is not working; without this the loop runs forever.
        parked = true;
        printBoth("PARKED at " + String(degrees(e), 1) + " deg after "
                  + String(MAX_STALL_RETRIES) + " stalls -- raise A, or accept."
                  " Send a new target to resume.");
      } else {
        printBoth("STALL at " + String(degrees(e), 1) + " deg, ww="
                  + String(omega_w, 1) + " -- backing off ("
                  + String(stallCount) + "/" + String(MAX_STALL_RETRIES) + ")");
      }
    }
  } else {
    stallStartMs = 0;
  }
  if (stallHold && millis() > stallHoldUntil) {
    stallHold = false; stallStartMs = 0;
    printBoth("stall hold released, retrying");
  }

  float alpha;

  if (stallHold || parked) {
    // ---- 2a. command nothing; the passive unwind bleeds the wheel down ----
    alpha = 0.0f;

  } else if (outside) {
    // ---- 2b. LQR / PD.  MINUS on error, PLUS on rate -- see header note (a). ----
    alpha = -K_theta * e + K_omega * omega_p;

    // ---- 3. Coulomb feedforward -- see header (b) ----
    float ff;
    if (fabsf(omega_p) > W_MOVING)
      ff = -A_FRICTION * signf(omega_p);   // MOVING: cancel friction opposing motion
    else
      ff =  A_FRICTION * signf(alpha);     // STUCK:  push to break free
    alpha += ffFrac * ff;

    // ---- 3b. authority ceiling, applied ONLY while the platform is stationary.
    //      A sustained alpha drives the wheel to a steady state of about
    //      alpha/0.84 rad/s, so alpha = 62 (K_theta at 30 deg error) implies
    //      74 rad/s -- far past the abort. While the platform is MOVING that
    //      never matters, because the manoeuvre finishes first, and clamping
    //      there would just slow large slews. The pathological case is pushing
    //      hard while stuck: no motion results and the wheel winds up regardless.
    //      ALPHA_STALL_MAX must exceed A_FRICTION (22.3) with REAL margin or the
    //      platform may never break free at all. 28 was tried and FAILED
    //      intermittently (test06, 2026-08-02 165048): 28 gives a*28 = 4.2 rad/s^2
    //      at a = 0.15, essentially equal to the 4.24 breakaway, so whether the
    //      platform moved was a coin flip on stiction. It sat at alpha = 28.0 for
    //      1.8 s, never moved, and wound the wheel to 44.5 into the abort.
    //      40 was ALSO too low (2026-08-02 170424): a terminal correction needs
    //      delivered alpha of ~28, but delivered = commanded - 0.84*omega_w, so
    //      arriving at 20 rad/s needs commanded 45. Steps that arrived below
    //      ~15 rad/s landed (0.31 deg); those arriving above ~17 stalled 2-9 deg
    //      short. 55 covers arrival up to ~32 rad/s. The implied steady wheel
    //      speed is past the abort, but only reached if the platform stays stuck
    //      for seconds -- which is what the stall hold below catches.
    if (notMoving) {
      if (alpha >  ALPHA_STALL_MAX) alpha =  ALPHA_STALL_MAX;
      if (alpha < -ALPHA_STALL_MAX) alpha = -ALPHA_STALL_MAX;
    }

  } else {
    // Inside tolerance: command nothing and let stiction hold the platform.
    alpha = 0.0f;
  }

  // ---- 4. feedback linearisation: cancel back-EMF and the wheel pole ----
  float u = (alpha + compFrac * A_2 * omega_w) / A_1;

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
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      target = wrapPi(radians(v));
      capN = 0; capturing = true; capStartMs = millis();
      stepCount++;
      ctrlMode = CTRL_STEP; controllerEnabled = true;
      printBoth("STEP -> " + String(v, 1) + " deg, capturing...");
      break;
    case 'H':
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      target = wrapPi(radians(v));
      ctrlMode = CTRL_HOLD; controllerEnabled = true;
      printBoth("HOLD -> " + String(v, 1) + " deg");
      break;
    case 'Z':
      theta = 0.0f; target = 0.0f;
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      printBoth("heading zeroed here");
      break;
    case 'C':
      // Compensation-only test: spin up at V, then alpha = 0. Watch omega_w.
      openVolts = constrain(v, -3.0f, 3.0f);
      theta = 0.0f;
      capN = 0; capturing = true; capStartMs = millis();
      openStartMs = millis();
      stepCount++;
      ctrlMode = CTRL_COMP; controllerEnabled = true;
      printBoth("COMP test " + String(openVolts, 2) + "V spin-up, compFrac="
                + String(compFrac, 2) + ", capturing...");
      printBoth("  expect: wheel coasts DOWN after the pulse. Growth = too high.");
      break;
    case 'A': A_FRICTION = constrain(v, 0.0f, 60.0f); printGains(); break;
    case 'K': compFrac = constrain(v, 0.0f, 1.2f); printGains(); break;
    case 'P': K_theta = v; printGains(); break;
    case 'D': K_omega = v; printGains(); break;
    case 'F': ffFrac  = constrain(v, 0.0f, 1.5f); printGains(); break;
    case 'W': deadzone = radians(fabsf(v)); printGains(); break;
    case 'N':
      deadzoneFine = radians(fabsf(v));
      if (degrees(deadzoneFine) < deadbandFloorDeg())
        printBoth("WARNING: " + String(v, 2) + " deg is below the "
                  + String(deadbandFloorDeg(), 2)
                  + " deg floor -- raise F first or this will stall");
      printGains();
      break;
    case 'G': printGains(); break;
    case 'B':
      ctrlMode = CTRL_IDLE; motor.target = 0.0f;
      printBoth("hold still, measuring bias...");
      measureGyroBias();
      break;
    case 'M':
      // Timing dump. `M` prints min/mean/MAX for every instrumented block;
      // `M!` resets all accumulators so the next window starts clean (run a
      // T90 + H0 first, then M! and repeat if you want a fresh worst-case).
      if (s.length() > 1 && s.charAt(1) == '!') {
        stat_reset(&st_foc);   stat_reset(&st_move);  stat_reset(&st_mpu);
        stat_reset(&st_ina);   stat_reset(&st_law);   stat_reset(&st_telem);
        stat_reset(&st_period);
        printBoth("timing stats reset");
      } else {
        // Dump to BOTH channels -- if this only went to USB, sending M over
        // HC-05 would land the reply on USB and look like nothing happened.
        Print* outs[2] = { &Serial, &hc05Serial };
        for (int i = 0; i < 2; i++) {
          Print& o = *outs[i];
          o.println("--- timing (us) ---");
          stat_print(o, "loopFOC",      &st_foc);
          stat_print(o, "move",         &st_move);
          stat_print(o, "MPU6050 read", &st_mpu);
          stat_print(o, "INA219 read",  &st_ina);   // no samples: no INA219 here
          stat_print(o, "control law",  &st_law);   // incl. MPU; compute = law - MPU
          stat_print(o, "telem row",    &st_telem);
          stat_print(o, "ctrl period",  &st_period);   // control-release interval (~5000 us)
          uint32_t fmn, fmx; focTick_jitter(&fmn, &fmx);
          o.print("FOC tick dt (us): min="); o.print(fmn); o.print(" max="); o.println(fmx);
          o.println("-------------------");
        }
      }
      break;
    case 'X': stopMotor("operator"); break;
    case 'E': {
      // Encoder diagnostic. Motor OFF (zero torque) so the wheel hand-turns
      // safely. Reads the CACHED angle/velocity focTask refreshes every 250us --
      // must NOT do its own SSI read, SPI2 is focTask's post-split. at rest:
      // single_turn_deg steady, vel ~0. hand-turn: sweeps smoothly. jumps = bad read.
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      capturing = false;
      printBoth("ENC DIAG: motor OFF -- hand-turn the wheel and watch.");
      printBoth("i,shaft_angle_rad,single_turn_deg,shaft_vel_rad_s");
      for (int i = 0; i < 30; i++) {
        float ang = motor.shaft_angle;
        float vel = motor.shaft_velocity;
        float turn = ang - floorf(ang / (2.0f * PI)) * (2.0f * PI);
        String line = String(i) + "," + String(ang, 4) + ","
                    + String(degrees(turn), 2) + "," + String(vel, 3);
        Serial.println(line); hc05Serial.println(line);
        delay(100);
      }
      printBoth("ENC DIAG done (motor still OFF; send R to re-enable).");
      break;
    }
    case 'V': {
      // Manual constant-voltage drive + unlimited encoder stream. focTask
      // commutates it -- do NOT call loopFOC/move here, SPI2 is focTask's.
      // Streams cached angle/velocity at ~50 Hz until ANY serial byte. Start
      // SMALL (V1, V2). steady V -> angle smooth, vel steady ~8.5*V; spikes = noise.
      float volts = constrain(v, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
      controllerEnabled = false; ctrlMode = CTRL_IDLE; capturing = false;
      motor.enable();
      motor.target = volts;
      printBoth("MANUAL DRIVE " + String(volts, 2) + "V. Press any key (or X) to STOP.");
      printBoth("t_ms,shaft_angle_rad,single_turn_deg,shaft_vel_rad_s,V");
      unsigned long t0 = millis(), lastP = 0;
      while (!Serial.available() && !hc05Serial.available()) {
        motor.target = volts;                          // focTask applies it each tick
        if (millis() - lastP >= 20) {                  // ~50 Hz, unlimited
          lastP = millis();
          float ang  = motor.shaft_angle;
          float vel  = motor.shaft_velocity;
          float turn = ang - floorf(ang / (2.0f * PI)) * (2.0f * PI);
          String line = String(millis() - t0) + "," + String(ang, 4) + ","
                      + String(degrees(turn), 2) + "," + String(vel, 3) + ","
                      + String(volts, 2);
          Serial.println(line); hc05Serial.println(line);
        }
      }
      motor.target = 0.0f;                              // focTask applies within one tick
      while (Serial.available())     Serial.read();     // drain the stop key
      while (hc05Serial.available()) hc05Serial.read();
      printBoth("MANUAL DRIVE stopped (target 0).");
      break;
    }
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
// ============================================================================
// RTOS STRUCTURE (Step 2.1) — the ONLY change from the verbatim super-loop.
// Everything above (control law, constants, commands, sensors) is untouched.
// ============================================================================

// Phase 4 tasks + ISR callback, defined below -- forward-declared so hwSetup()
// can create focTask and register focTickNotify.
extern TaskHandle_t hFoc;
static void focTask(void*);
static void focTickNotify(void);

// hwSetup(): everything the old setup() did after Serial.begin -- run INSIDE the
// control task. initFOC()'s alignment spins the motor for hundreds of ms and is
// FPU-heavy, so running it in the task uses the task's stack + FPU context (what
// we want to be testing), and the fault hooks are live while it runs.
static void hwSetup() {
  // Arm the safety net BEFORE motor bring-up: a fault during initFOC (which
  // spins the wheel) then still kills the driver. Hardware kill = enable pin LOW;
  // graceful stop hook = the SimpleFOC path.
  faults_init(PIN_ENABLE, LED_BUILTIN,
              []() { motor.target = 0.0f; motor.disable(); driver.disable(); });

  timers_dumpAll("BEFORE SimpleFOC init");

  // ... existing SPI/encoder/driver/motor init, initFOC(), Wire, MPU6050, INA219

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

  // Phase 4: hand commutation to focTask, clocked by the TIM9 tick. Created here
  // (after initFOC, motor ready) not in setup(). focTask blocks on its notify at
  // once; nothing commutates until focTick_init() starts TIM9 on the next line.
  configASSERT(xTaskCreate(focTask, "foc", 768, NULL, 4, &hFoc) == pdPASS);
  focTick_attach(focTickNotify);
  focTick_init(FOC_RATE_HZ);     // 4 kHz: focTask each tick, control every 20th

  timers_dumpAll("AFTER SimpleFOC init");
  Serial.print("TIM2 CR1=0x"); Serial.println(TIM2->CR1, HEX);

  // ... rest of your existing setup

  printBoth("");
  printBoth("=== Heading controller ===");
  printBoth("Platform must be STILL for bias measurement.");
  measureGyroBias();
  theta = 0.0f; target = 0.0f;

  printGains();
  printBoth("FIRST: send O1 (open-loop pulse) to verify the gyro sign before closing the loop.");
  printBoth("Commands: O<V> openloop | C<V> comp-test | T<deg> step+capture | H<deg> hold");
  printBoth("          Z zero | P/D gains | K<0-1.2> compFrac | F<0-1> ff");
  printBoth("          W<deg> coarse dz | N<deg> fine dz | A<val> friction magnitude");
  printBoth("          G status | B rebias | M timing | M! reset timing | X stop | R resume");
  printBoth("          E encoder-diag (motor off) | V<volts> manual drive + encoder stream");
  printBoth("Start at K_theta=19.1 K_omega=14.0, then work down the gain table.");
  printBoth("Mode is IDLE -- send H0 or T<deg> to engage.");

  lastControlUs = micros();
}

// controlStep(): one 200 Hz control iteration. Body is the old superLoopBody
// verbatim MINUS loopFOC/move (now in focTask) and MINUS the micros() rate gate
// (the TIM9 notification IS the 200 Hz clock now). dt still comes from micros()
// so it stays correct across a blocking command (e.g. B rebias), as before.
static void controlStep() {
  pollSerial();

  unsigned long now = micros();
  float dt = (now - lastControlUs) * 1e-6f;
  lastControlUs = now;

  TIME_BLOCK(st_law, { controlUpdate(dt); });

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
      bool wasOpen = (ctrlMode == CTRL_OPEN || ctrlMode == CTRL_COMP);
      ctrlMode = wasOpen ? CTRL_IDLE : CTRL_HOLD;
      if (wasOpen)
        printBoth("open-loop capture done, back to IDLE. Check: positive volts "
                  "should give POSITIVE wheel and NEGATIVE theta.");
      else
        printBoth("capture done, now HOLDing. Adjust gains and send T again.");
    }
  }

  // Slow telemetry while holding, so you can nudge the platform by hand and
  // watch it recover without needing a full capture.
  if (ctrlMode == CTRL_HOLD && !capturing && (millis() - lastTelemMs) >= TELEM_PERIOD_MS) {
    lastTelemMs = millis();
    TIME_BLOCK(st_telem, {
      String s = String(degrees(theta), 2) + "," + String(degrees(target), 2) + ","
               + String(omega_p, 3) + "," + String(omega_w, 2) + ","
               + String(lastAlpha, 2) + "," + String(lastU, 3);
      Serial.println(s);
      hc05Serial.println(s);
    });
  }
}

// ---- the tasks, the FOC-tick ISR callback, and the pre-scheduler setup ----
TaskHandle_t hControl;
TaskHandle_t hFoc;

// focTickNotify(): runs inside the TIM9 ISR (NVIC prio 5 = the configMAX_SYSCALL
// boundary, so these FromISR calls are legal; anything more urgent would trip the
// kernel assert). Notify focTask every tick, controlTask every CTRL_DIVISOR-th,
// then request a context switch on exception return.
static void focTickNotify(void) {
  static uint32_t n = 0;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(hFoc, &woken);
  if (++n >= CTRL_DIVISOR) { n = 0; vTaskNotifyGiveFromISR(hControl, &woken); }
  portYIELD_FROM_ISR(woken);
}

// focTask: commutation only. Prio 4 (highest) + hardware-interrupt driven, so it
// preempts straight through the blocking MPU read in the control task -- the
// whole point of Phase 4. loopFOC()/move() run ONLY here now.
static void focTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TIME_BLOCK(st_foc,  { motor.loopFOC(); });
    TIME_BLOCK(st_move, { motor.move();    });
  }
}

static void controlTask(void*) {
  hwSetup();                 // bring up hardware; creates focTask + starts TIM9
  static uint32_t t_prev = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // 200 Hz release from TIM9 ISR
    uint32_t t = us_now();
    if (t_prev) stat_add(&st_period, t - t_prev);   // st_period = control-release interval
    t_prev = t;
    controlStep();
  }
}

// setup(): pre-scheduler only. Create the task and hand the CPU to FreeRTOS.
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  us_init();
  faults_reportLastBoot();   // the .noinit black box (Step 1.2)

  configASSERT(xTaskCreate(controlTask, "ctrl", 1536, NULL, 3, &hControl) == pdPASS);
  vTaskSetThreadLocalStoragePointer(hControl, 0, (void*)(uintptr_t)TRACE_ID_CTRL);

  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);   // only if the heap was too small
}

void loop() {}   // never runs under the scheduler