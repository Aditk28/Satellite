/*
  Docking platform controller on FreeRTOS (Nucleo-F446RE).
  Heading is controlled by the reaction wheel, x/y by the four fans, and the
  stepped docking sequencer ties the two together with the pose estimator.

  Heading control law, run every control cycle (200 Hz):
    1. e     = wrap(target - theta)
    2. alpha = -K_theta*e + K_omega*omega_p                PD (LQR-derived)
    3. alpha += ffFrac * friction feedforward              Coulomb cancel
    4. u     = (alpha + compFrac*A_2*omega_w) / A_1        feedback linearisation
    5. clamp u to +-VOLTAGE_LIMIT, write motor.target

  After linearisation the plant is theta_p'' = -a*alpha, a = J_w/J_p (~0.105).

  Sign notes. Both are easy to get wrong and both give plausible but broken behaviour:
    - Step 2 has minus on the error and plus on the rate because the plant gain is
      -a. The textbook -(K_theta*e + K_omega*omega_p) is unstable here.
    - The two feedforward branches have opposite signs. While moving it opposes the
      platform rate: -A_MOVING*sign(omega_p). While stuck it pushes the way the
      controller wants to go: +A_FRICTION*sign(alpha). A backwards moving branch is
      worse than no feedforward at all.

  Gain design: wn^2 = a*K_theta, K_omega = 2*zeta*wn/a, t_settle ~ 4/(zeta*wn).
  wn is limited to roughly A_2, the wheel pole.

  The gyro and the wheel encoder have opposite sign conventions as mounted, so
  GYRO_SIGN flips the gyro (same as --flip-gyro in filter_calibration.py).

  Serial commands (USB or HC-05, 115200, newline terminated) are listed in the
  boot banner in hwSetup(). Unrecognised input stops the wheel and the fans.
*/

#include <SimpleFOC.h>
#include "MagneticSensorMT6701SSI.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <STM32FreeRTOS.h>
#include "timebase.h"
#include "hw_timers.h"
#include "timing_stats.h"
#include "faults.h"
#include "telemetry.h"       // telemTask is the only serial writer
#include "safety.h"          // independent watchdog task
#include "i2c_bus.h"         // I2C mutex (MPU6050 here, INA219 in safetyTask)
#include "commands.h"        // commsTask owns serial RX
#include "fans.h"            // fanTask is the only fan writer
#include "pi_link.h"         // USART6 pose link from the Pi
#include "estimator.h"       // dock-relative pose estimator
#include "translation.h"     // x/y control via the fans
#include "trans_capture.h"   // 20 Hz recorder for a translation move

// Hardware
#define POLE_PAIRS      11
#define PIN_PWM_A       13
#define PIN_PWM_B       12
#define PIN_PWM_C       11
#define PIN_ENABLE      10
#define PIN_ENCODER_CS  PB1

#define HC05_EN_PIN     PC12
#define HC05_BAUD       115200
HardwareSerial hc05Serial(PC11, PC10);     // (RX, TX) = USART3

// Pi pose link. Argument order is (RX, TX); swapping them compiles but the link is dead.
#define PI_BAUD         115200
HardwareSerial piSerial(PC7, PC6);         // USART6

#define VOLTAGE_LIMIT   10.0f
// Ceiling for the O/C open-loop pulses. Plateau = 9.64*V - 1.23, so 5.5 V gives
// 51.8 rad/s, just under WHEEL_SAT_LIMIT. Raise both together or neither.
#define OPEN_PULSE_VMAX  5.5f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);
Adafruit_MPU6050 mpu;
Adafruit_INA219  ina219;          // read by safetyTask under the I2C mutex
bool             inaPresent = false;

// Identified plant constants. Named A_1/A_2 because A1/A2 are analog pin macros
// on STM32duino (same reason the mode enum is CTRL_* and not MODE_*).
// From open-loop O tests with the fan hardware fitted:
//   omega_plateau = 9.64*V - 1.23   (the intercept is the wheel's own Coulomb friction)
//   tau = 0.201 s from free decay
//   A_1 = K/tau = 47.9, A_2 = 1/tau = 4.97
static const float A_1 = 47.9f;    // rad/s^2 per V
static const float A_2 = 4.97f;    // 1/s

// Friction feedforward, in wheel-acceleration units (rad/s^2). Split in two because
// static breakaway is larger than kinetic friction: a single value big enough to
// break free over-cancels once moving and acts as negative damping.
float A_FRICTION  = 60.0f;    // stuck branch, must beat static breakaway (~53-56). Command A.
float A_MOVING    = 34.0f;    // moving branch, cancels kinetic friction. Command AM.
// Extra moving-branch term per rad/s of platform rate. Command AV. Friction grows with
// speed (~34 near rest, ~65 at 2.1 rad/s, so a slope of ~15). 0 = pure Coulomb.
float A_VISCOUS   = 0.0f;
static const float GYRO_SIGN = -1.0f;    // aligns the gyro with the wheel convention

// Timing instrumentation
static stat_t st_foc, st_move, st_mpu, st_ina, st_law, st_telem, st_period, st_adc;

// Runtime-tunable gains (P, D, F, K, W, N commands).
// K_theta is the 1.2 s settle row for a = 0.105 (wn = 4.76). K_omega was bisected
// down on hardware to 52 (zeta ~0.54): Coulomb friction already adds damping, so the
// zeta 0.7 design stopped short of target.
// Keep the gains consistent with the deadzones. If a deadzone is below the floor
// (1-ffFrac)*A_MOVING/K_theta the feedforward never switches off and the wheel winds
// up. printGains() warns when that happens.
float K_theta   = 216.0f;
float K_omega   = 52.0f;
float ffFrac    = 0.95f;
// Fraction of A_2 applied in the linearisation. Under-compensating leaves a stable,
// slightly sluggish residual pole; over-compensating leaves an unstable one. The C
// sweep put the neutral point at ~1.14, and 0.90 gives a residual pole of about
// -0.92 /s in both directions. Verify with C<V>: the wheel must coast down.
float compFrac  = 0.90f;
// Two-stage deadzone: coarse while the wheel is fast and has little authority left,
// fine once it has slowed below FINE_WW (delivered alpha drops with wheel speed).
float deadzone     = 0.0262f;  // rad (1.5 deg)
float deadzoneFine = 0.0140f;  // rad (0.8 deg)
#define FINE_WW         5.0f   // rad/s

#define W_MOVING            0.05f    // rad/s, platform counts as moving above this

// Momentum management is passive. Inside the deadzone alpha = 0, the linearisation
// under-compensates, and the wheel bleeds down on its own while friction holds the
// heading. Do not brake by commanding a negative alpha through the linearisation:
// the compensation shortfall adds to it and the reaction torque breaks the platform
// free. If active braking is ever needed, command voltage from the wheel's hold curve.
//
// The case passive unwind cannot fix is a stall: outside the deadzone, platform stuck,
// wheel already fast. Then we stop commanding for STALL_HOLD_MS and retry.
#define ALPHA_STALL_MAX     70.0f    // rad/s^2, cap on alpha while the platform is stationary
// The stall detector must fire before WHEEL_SAT_LIMIT. Stuck at alpha = 70 the wheel
// passes 20 rad/s at ~0.33 s, so detection lands at ~0.63 s and ~33 rad/s.
#define STALL_WW            20.0f    // rad/s
#define STALL_MS             300     // ms of continuous stall before backing off
#define STALL_HOLD_MS       4500     // ms of alpha = 0 before retrying (~50 -> ~10 rad/s)
#define WW_MAX_JUMP         15.0f    // rad/s per cycle; bigger jumps are estimate glitches
#define WHEEL_SAT_LIMIT     55.0f    // rad/s, abort above this (the motor can reach ~96)
#define TELEM_PERIOD_MS     100      // 10 Hz streaming telemetry in HOLD
#define FOC_RATE_HZ         4000     // TIM9 FOC tick
#define CTRL_DIVISOR        20       // 4000 / 20 = 200 Hz control

// State
enum CtrlMode { CTRL_IDLE, CTRL_HOLD, CTRL_STEP, CTRL_OPEN, CTRL_COMP };
volatile CtrlMode ctrlMode = CTRL_IDLE;

// Open-loop pulse (O) and comp test (C). The open-loop pulse is bounded whatever the
// gyro sign is, which makes it the safe way to check GYRO_SIGN before closing the loop.
float openVolts = 0.0f;
unsigned long openStartMs = 0;
#define OPEN_PULSE_MS 800

float theta      = 0.0f;    // estimated platform heading, rad
float omega_p    = 0.0f;    // platform rate, rad/s
float omega_w    = 0.0f;    // wheel rate, rad/s
unsigned long wwRejects = 0;// rejected wheel-velocity glitches
float target     = 0.0f;    // commanded heading, rad
float gyroBias   = 0.0f;    // dps, measured at startup

// Translation plant ID works in acceleration units (x_ddot = A(throttle) - A_c*sign(v)),
// so mass and thrust in grams are never needed.
#define ESC_CUR_PIN A4      // PC1, the 4-in-1 ESC current-sense pad
float accelBiasX = 0.0f, accelBiasY = 0.0f;  // m/s^2, measured with the gyro bias
float accel_x = 0.0f, accel_y = 0.0f;        // m/s^2, bias removed, body frame
uint16_t escCurRaw = 0;                      // raw ADC counts
static bool transActive = false;             // I/Q thrust step running (stopMotor aborts it)
// Capture flavour, latched at capture start because the dump runs later on telemTask.
static bool capTranslation = false;  // plant-ID run: stay IDLE afterwards
static bool capKeepWheel = false;    // Q: keep heading control running so the wheel unwinds gently
static bool capOpenLoop = false;
static bool capCompTest = false;
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

TaskHandle_t hControl;
TaskHandle_t hFoc;

// Capture buffer: 1500 samples at 200 Hz = 7.5 s, ~58 KB.
#define MAX_CAP 1500
unsigned long cap_t[MAX_CAP];
float cap_target[MAX_CAP], cap_theta[MAX_CAP], cap_wp[MAX_CAP];
float cap_ww[MAX_CAP], cap_alpha[MAX_CAP], cap_u[MAX_CAP];
float cap_ax[MAX_CAP], cap_ay[MAX_CAP];
uint16_t cap_cur[MAX_CAP];
int   capN = 0;
bool  capturing = false;
unsigned long capStartMs = 0;
// Long enough to see a retry after a stall hold, and the most MAX_CAP allows.
#define CAPTURE_MS 7400
uint32_t    capWindowMs  = CAPTURE_MS;
const char* capModeName  = "heading_step";
float       capThrottle  = 0.0f;   // recorded in the metadata line
int         capFanSel    = 0;

int stepCount = 0;

static inline float wrapPi(float x) {
  while (x >  PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  return x;
}
static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

// Queued to telemTask (written directly during boot, before telem_activate()).
void printBoth(const String &s) { telem_print(s); }

// Distance from the magnet to the dock point.
static float dockDist(const EstState& e) {
  return sqrtf(e.mag_x * e.mag_x + (e.mag_y - EST_D_DOCK) * (e.mag_y - EST_D_DOCK));
}

/* Stepped docking. Dead-reckoning error grows as bias*T^2, so instead of one long
   approach the platform hops a short distance, stops, takes a vision fix while
   stationary (the only time vision is trustworthy), and hops again.
     MOVE   translation drives to an intermediate waypoint
     SETTLE thrust off, wait for the rest latch (ZUPT + averaged vision fix) */
#define DOCK_HOP_M      0.06f   // default metres per leg, runtime via TH
static float dockHopM = DOCK_HOP_M;
#define DOCK_DONE_M     0.015f  // close enough to call it docked
#define DOCK_SETTLE_MS  4500    // rest detect + 8 frames at ~10 fps, plus slack
#define DOCK_MAX_HOPS   10
enum DockPhase { DOCK_OFF, DOCK_MOVE, DOCK_SETTLE };
static DockPhase dockPhase = DOCK_OFF;
static uint32_t  dockPhaseMs = 0;
static int       dockHops = 0;

void stopMotor(const String &why) {
  // Fans first: with unguarded props they are the bigger hazard. This is the hard
  // kill (pins driven low and latched); R re-arms in ~1 s.
  fans_stopAll();
  transActive = false;
  trans_enable(false);
  dockPhase = DOCK_OFF;
  controllerEnabled = false;
  ctrlMode = CTRL_IDLE;
  capturing = false;
  stallHold = false; stallStartMs = 0; parked = false; stallCount = 0;
  motor.target = 0.0f;               // focTask applies this within one tick (<=250 us)
  printBoth("!! STOP: " + why + "  (send R to re-enable)");
}

// Adapters for safetyTask, which does not depend on SimpleFOC. Wheel speed comes
// straight from the FOC layer so a wedged control task cannot freeze it.
static float safetyWheelVel(void)          { return motor.shaft_velocity; }
static void  safetyStop(const char* why)   { stopMotor(String(why)); }

// X fast path, run on commsTask the moment an X arrives, ahead of the queue. An
// aligned float store is atomic on the M4 and fans_stopAll() is lock-free register
// writes, so both are safe here. handleLine() still runs the full stopMotor() later.
static void commsEmergencyStop(void) { motor.target = 0.0f; fans_stopAll(); }

// Runs on commsTask once per loss of the Pi link. The link layer has already zeroed
// the fans; the wheel is left alone because dumping a spinning flywheel would itself
// kick the platform.
static void piLinkDead(void) {
  printBoth("!! PI LINK SILENT: no frames from the Pi (Pi or wiring, not a lost tag). "
            "Fans killed, wheel control unchanged.");
}

// Called from safetyTask. Longer lock timeout than the control path because safety
// is not deadline-critical and would rather wait than skip a sample.
static bool safetyReadPower(float* busV, float* mA) {
  if (!inaPresent) return false;
  if (!i2c_lock(5)) return false;
  TIME_BLOCK(st_ina, {
    *busV = ina219.getBusVoltage_V();
    *mA   = ina219.getCurrent_mA();
  });
  i2c_unlock();
  return true;
}

// Platform must be still and level. Takes about a second. Also measures the
// accelerometer bias, which includes any table tilt.
void measureGyroBias() {
  const int N = 200;
  float sum = 0.0f;
  float sumAx = 0.0f, sumAy = 0.0f;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    if (i2c_lock(20)) { mpu.getEvent(&a, &g, &t); i2c_unlock(); }
    sum   += g.gyro.z * 180.0f / PI;
    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    safety_kick();
    // vTaskDelay, not delay(): delay() busy-waits and would starve the lower
    // priority safety and telemetry tasks for the whole second.
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  gyroBias = sum / N;
  accelBiasX = sumAx / N;
  accelBiasY = sumAy / N;
  printBoth("[GYRO] bias = " + String(gyroBias, 4) + " dps");
  printBoth("[ACCEL] bias = " + String(accelBiasX, 4) + " / " + String(accelBiasY, 4)
            + " m/s^2  (includes any residual tilt)");
}

// Smallest heading error the controller can close, set by the moving friction that
// the feedforward leaves uncancelled. A deadzone below this is never reached.
float deadbandFloorDeg() {
  return degrees((1.0f - ffFrac) * A_MOVING / K_theta);
}

void printGains() {
  printBoth("K_theta=" + String(K_theta, 2) + "  K_omega=" + String(K_omega, 2)
            + "  ff=" + String(ffFrac, 2) + "  deadzone=" + String(degrees(deadzone), 2)
            + "/" + String(degrees(deadzoneFine), 2) + "deg"
            + "  compFrac=" + String(compFrac, 2)
            + (stallHold ? "  [STALL-HOLD]" : "")
            + (parked ? "  [PARKED]" : ""));
  printBoth("A_static=" + String(A_FRICTION, 1) + " (ff*=" + String(ffFrac * A_FRICTION, 1)
            + ")  A_moving=" + String(A_MOVING, 1) + " (ff*=" + String(ffFrac * A_MOVING, 1) + ")"
            + "  A_visc=" + String(A_VISCOUS, 1) + "/rad/s");
  float floorDeg = deadbandFloorDeg();
  bool floorBad = (degrees(deadzoneFine) < floorDeg) || (degrees(deadzone) < floorDeg);
  printBoth("deadband floor=" + String(floorDeg, 2) + "deg"
            + (floorBad ? "  <-- BELOW A DEADZONE" : "  (both deadzones clear it)"));
  if (floorBad) {
    printBoth("** DEADZONE BELOW FLOOR: the controller cannot settle and the wheel will wind up.");
    printBoth("** Fix: raise N/W above " + String(floorDeg, 2) + " deg, or raise F, or raise P.");
  }
  printBoth("theta=" + String(degrees(theta), 2) + "deg  target=" + String(degrees(target), 2)
            + "deg  omega_p=" + String(omega_p, 3) + "  omega_w=" + String(omega_w, 2)
            + "  mode=" + String(ctrlMode == CTRL_IDLE ? "IDLE" : (ctrlMode == CTRL_HOLD ? "HOLD" : "STEP")));
  printBoth("wheel-velocity glitches rejected: " + String(wwRejects)
            + "  |  telem drops: " + String(telem_drops())
            + "  telem stack free (words): " + String(telem_stackFreeWords()));
  printBoth("safety checks: " + String(safety_checks())
            + "  stack free (words): " + String(safety_stackFreeWords())
            + "  i2c timeouts: " + String(i2c_timeouts()));
  printBoth("comms: rx bytes=" + String(commands_rxBytes())
            + "  drops=" + String(commands_drops())
            + "  stack free (words): " + String(commands_stackFreeWords()));
  // Healthy: overruns = 0 with frames climbing at ~500/s. Skips are benign.
  printBoth(String("fans: ") + (fans_killed() ? "KILLED" : (fans_armed() ? "armed" : "arming"))
            + "  frames=" + String(fans_frames())
            + "  skips=" + String(fans_skips())
            + "  overruns=" + String(fans_overruns())
            + "  rejects=" + String(fans_rejects())
            + "  budget-scaled=" + String(fans_budgetHits())
            + "  cap=" + String(FAN_THROTTLE_MAX, 0) + "%"
            + "  stack free (words): " + String(fans_stackFreeWords()));
  printBoth("fans pct: " + String(fans_pct(1), 1) + " / " + String(fans_pct(2), 1)
            + " / " + String(fans_pct(3), 1) + " / " + String(fans_pct(4), 1));
  // Healthy Pi link: frames climbing, crc/badlen/resync/seqgaps all 0. rx climbing
  // while frames does not means the two ends disagree on the format or CRC.
  {
    static const char* kState[] = { "NEVER", "FRESH", "STALE", "LOST", "DEAD" };
    uint32_t a = pi_ageUs();
    uint32_t l = pi_linkAgeUs();
    printBoth(String("pi link: ") + kState[(int)pi_state()]
              + (pi_linkAlive() ? "  [link OK]" : "  [LINK SILENT]")
              + "  poseage=" + (a == UINT32_MAX ? String("--") : String(a / 1000)) + "ms"
              + "  linkage=" + (l == UINT32_MAX ? String("--") : String(l / 1000)) + "ms"
              + "  frames=" + String(pi_frames())
              + "  rx=" + String(pi_rxBytes())
              + "  crc=" + String(pi_crcErrors())
              + "  badlen=" + String(pi_badLen())
              + "  resync=" + String(pi_resyncs())
              + "  seqgaps=" + String(pi_seqGaps())
              + "  seqrestart=" + String(pi_seqRestarts())
              + "  maxburst=" + String(pi_maxBurst()));
    PiPose p;
    if (pi_getPose(&p)) {
      printBoth("pi pose: seq=" + String(p.seq)
                + "  tag=" + String(p.tag_id) + " x" + String(p.n_tags)
                + "  range=" + String(p.range_m, 3) + "m"
                + "  bearing=" + String(degrees(p.bearing_rad), 2) + "deg"
                + "  relyaw=" + String(degrees(p.relyaw_rad), 2) + "deg"
                + "  q=" + String(p.quality, 2)
                + "  piage=" + String(p.age_us / 1000) + "ms"
                + "  flags=0x" + String(p.flags, HEX));
    }
  }
  {
    EstState e;
    float ga, gb, gc;
    est_getGains(&ga, &gb, &gc);
    if (est_get(&e)) {
      printBoth("est: x=" + String(e.x, 3) + " y=" + String(e.y, 3)
                + " psi=" + String(degrees(e.psi), 1) + "deg"
                + "  v=(" + String(e.vx, 3) + "," + String(e.vy, 3) + ")m/s"
                + "  mag=(" + String(e.mag_x, 3) + "," + String(e.mag_y, 3) + ")"
                + "  dockerr=" + String(dockDist(e), 3) + "m");
    } else {
      printBoth("est: no fix yet (needs one valid vision frame)");
    }
    printBoth("est gains: aPos=" + String(ga, 3) + " bVel=" + String(gb, 3)
              + " aPsi=" + String(gc, 3) + "   fixes=" + String(est_fixes())
              + " rejects=" + String(est_rejects()));
  }
  {
    float tkp, tkd, tff, tax, tay, p1, p2, p3, p4, tx, ty;
    trans_getGains(&tkp, &tkd, &tff);
    trans_lastCommand(&tax, &tay, &p1, &p2, &p3, &p4);
    trans_getTarget(&tx, &ty);
    printBoth(String("trans: ") + (trans_enabled() ? "ON " : "off")
              + (trans_tripped() ? " TRIPPED" : "")
              + "  target=(" + String(tx, 3) + "," + String(ty, 3) + ")"
              + "  err=" + String(trans_lastErr(), 3) + "m"
              + "  a=(" + String(tax, 2) + "," + String(tay, 2) + ")m/s2"
              + "  pct=" + String(p1, 0) + "/" + String(p2, 0) + "/"
              + String(p3, 0) + "/" + String(p4, 0));
    printBoth("trans gains: Kp=" + String(tkp, 2) + " Kd=" + String(tkd, 2)
              + " ff=" + String(tff, 2) + "   idle=" + String(TRANS_IDLE_PCT, 0)
              + "%  deadzone=" + String(TRANS_DEADZONE * 100.0f, 1) + "cm");
    if (trans_tripped()) printBoth(String("  !! ") + trans_tripReason());
  }
  if (fans_overruns()) {
    uint32_t nl, nmin, nmax, cen;
    fans_overrunDetail(&nl, &nmin, &nmax, &cen);
    // NDTR of 72 = nothing transferred; small (1-3) = frame ended short.
    printBoth("fan overrun detail: NDTR last=" + String(nl) + " min=" + String(nmin)
              + " max=" + String(nmax) + " (of 72)  TIM1 CEN was " + String(cen));
  }
  {
    float mnV, mxV, mxA; uint32_t pf;
    if (safety_powerStats(&mnV, &mxV, &mxA, &pf))
      printBoth("power: busV min=" + String(mnV, 2) + " max=" + String(mxV, 2)
                + "  |I| max=" + String(mxA, 0) + "mA  read fails=" + String(pf)
                + "   (trips: <10.0V, >2500mA)");
    else
      printBoth("power: no INA219 samples yet");
  }
  printBoth("ctrl stack free (min words): "
            + String((uint32_t)uxTaskGetStackHighWaterMark(NULL)));
}

void dumpCaptureTo(Print &out) {
  out.print("--- capture start (test ");
  out.print(stepCount);
  out.print("/");
  out.print(stepCount);
  // The label becomes the filename in capture_calibration.py, so it describes the run.
  out.print(": ");
  if (capTranslation) {
    out.print(capModeName);
    if (capFanSel > 0) { out.print(" fan"); out.print(capFanSel); }
    else if (capThrottle > 0.0f) out.print(" all");
    if (capThrottle > 0.0f) { out.print(" at "); out.print(capThrottle, 0); out.print(" pct"); }
  } else if (ctrlMode == CTRL_OPEN || capOpenLoop) {
    out.print("openloop ");
    out.print(openVolts, 2);
    out.print(" V");
  } else if (ctrlMode == CTRL_COMP || capCompTest) {
    out.print("comptest ");
    out.print(openVolts, 2);
    out.print("V cf ");
    out.print(compFrac, 2);
  } else {
    out.print("heading step to ");
    out.print(degrees(target), 1);
    out.print(" deg");
  }
  out.println(") ---");
  // Metadata: everything runtime-tunable, so a capture can be reproduced from its header.
  out.print("mode="); out.print(capModeName);
  out.print(" target_deg="); out.print(degrees(target), 3);
  out.print(" K_theta=");  out.print(K_theta, 3);
  out.print(" K_omega=");  out.print(K_omega, 3);
  out.print(" ff=");       out.print(ffFrac, 3);
  out.print(" deadzone_deg="); out.print(degrees(deadzone), 3);
  out.print(" gyro_bias_dps="); out.print(gyroBias, 4);
  out.print(" accel_bias_x="); out.print(accelBiasX, 4);
  out.print(" accel_bias_y="); out.print(accelBiasY, 4);
  out.print(" fan_pct=");      out.print(capThrottle, 1);
  out.print(" fan_sel=");      out.print(capFanSel);
  out.print(" open_volts=");   out.print(openVolts, 2);
  out.print(" A_static=");     out.print(A_FRICTION, 1);
  out.print(" A_moving=");     out.print(A_MOVING, 1);
  out.print(" A_visc=");       out.print(A_VISCOUS, 1);
  out.print(" compFrac=");     out.print(compFrac, 3);
  out.print(" alpha_stall_max="); out.print(ALPHA_STALL_MAX, 1);
  out.println(" stop_reason=fixed_window");
  out.println("t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u,ax,ay,iadc");
  for (int i = 0; i < capN; i++) {
    out.print(cap_t[i]);                    out.print(",");
    out.print(degrees(cap_target[i]), 3);   out.print(",");
    out.print(degrees(cap_theta[i]), 3);    out.print(",");
    out.print(cap_wp[i], 4);                out.print(",");
    out.print(cap_ww[i], 3);                out.print(",");
    out.print(cap_alpha[i], 3);             out.print(",");
    out.print(cap_u[i], 4);                 out.print(",");
    out.print(cap_ax[i], 4);                out.print(",");
    out.print(cap_ay[i], 4);                out.print(",");
    out.println(cap_cur[i]);
  }
  out.println("--- capture end ---");
}

// The bulk writers below run on telemTask via telem_run(), which owns the serial
// ports. Never call them directly from the control task.
void dumpCapture() { dumpCaptureTo(Serial); dumpCaptureTo(hc05Serial); }
void dumpTransCapture() { tcap_dumpTo(Serial); tcap_dumpTo(hc05Serial); }

// Per-task CPU share and stack high-water marks. The run-time counter is TIM5 at
// 1 MHz, which wraps every ~71 minutes. High-water marks only cover code paths that
// have actually run.
void printSystemReport() {
  static char buf[768];          // ~40 B per task
  vTaskGetRunTimeStats(buf);
  Print* outs[2] = { &Serial, &hc05Serial };
  for (int i = 0; i < 2; i++) {
    Print& o = *outs[i];
    o.println("--- CPU utilisation (since boot; TIM5 1MHz, wraps ~71 min) ---");
    o.println("Task            AbsTime         %");
    o.print(buf);
    o.println("--- stack high-water: free words (allocated) ---");
    o.print("foc     "); o.print((unsigned)uxTaskGetStackHighWaterMark(hFoc));
    o.println(" / 256");
    o.print("ctrl    "); o.print((unsigned)uxTaskGetStackHighWaterMark(hControl));
    o.println(" / 768");
    o.print("safety  "); o.print((unsigned)safety_stackFreeWords());
    o.println(" / 384");
    o.print("comms   "); o.print((unsigned)commands_stackFreeWords());
    o.println(" / 256");
    o.print("telem   "); o.print((unsigned)telem_stackFreeWords());
    o.println(" / 512");
    o.println("-------------------------------------------------------------");
  }
}

void printTimingStats() {
  Print* outs[2] = { &Serial, &hc05Serial };
  for (int i = 0; i < 2; i++) {
    Print& o = *outs[i];
    o.println("--- timing (us) ---");
    stat_print(o, "loopFOC",      &st_foc);
    stat_print(o, "move",         &st_move);
    stat_print(o, "MPU6050 read", &st_mpu);
    stat_print(o, "INA219 read",  &st_ina);
    stat_print(o, "control law",  &st_law);   // includes the MPU read
    stat_print(o, "telem row",    &st_telem);
    stat_print(o, "ESC cur ADC",  &st_adc);
    stat_print(o, "ctrl period",  &st_period);   // control release interval (~5000 us)
    uint32_t fmn, fmx; focTick_jitter(&fmn, &fmx);
    o.print("FOC tick dt (us): min="); o.print(fmn); o.print(" max="); o.println(fmx);
    o.println("-------------------");
  }
}

// TC single-fan direction probe: one fan, heading held by the wheel so psi stays
// put, position from vision. Separate from the I thrust step, which turns the wheel
// off on purpose to keep reaction torque out of the accelerometer.
#define TCAL_PRE_MS   500
#define TCAL_HOLD_MS  900
#define TCAL_TOTAL_MS 4000
static bool     tcalActive  = false;
static float    tcalPct     = 0.0f;
static uint32_t tcalStartMs = 0;

/* Rest detector. "At rest" means the operator has stopped moving the platform:
   velocity is known to be zero (a free ZUPT) and it is the moment to take a
   vision fix. Tight on magnitude and loose on time, because a false "at rest"
   while sliding would zero a real velocity, while a late one only costs a second. */
#define REST_ACCEL     0.10f    // m/s^2, bias removed
#define REST_OMEGA     0.06f    // rad/s
#define REST_MS        500      // continuously quiet for this long
#define MOVING_ACCEL   0.25f    // re-arms the detector
static bool     restLatched = false;
static uint32_t restQuietMs = 0;

/* TB: accel-bias measurement with the fans at idle. Runs in the control loop rather
   than blocking in the command path because it must keep feeding the fan dead-man. */
#define TBIAS_SPIN_MS  1500     // let the props reach steady state first
#define TBIAS_MEAS_MS  2500     // ~500 samples at 200 Hz
static bool     tbiasActive  = false;
static uint32_t tbiasStartMs = 0;
static double   tbiasSumX = 0.0, tbiasSumY = 0.0;
static uint32_t tbiasN = 0;

static int fanSel = 0;   // which fan L/I/Q/J/TC drive: 0 = all, 1-4 = one channel

// Apply throttle to the selected fan(s). Called every cycle during a run, which
// also feeds the fan dead-man timeout.
static void applyFanStep(float pct) {
  if (fanSel == 0) fans_setAll(pct, pct, pct, pct);
  else             fans_setThrottle(fanSel, pct);
}

void controlUpdate(float dt) {
  // Sense. The gyro read takes the I2C mutex with a short timeout and never blocks
  // forever; on failure it reuses the previous sample.
  static float gyro_dps_prev = 0.0f;
  float gyro_dps;
  sensors_event_t a, g, t;
  if (i2c_lock(2)) {
    TIME_BLOCK(st_mpu, { mpu.getEvent(&a, &g, &t); });
    i2c_unlock();
    gyro_dps = (g.gyro.z * 180.0f / PI) - gyroBias;
    gyro_dps_prev = gyro_dps;
    // The accel comes in the same transaction, so it is free. Body frame, bias removed.
    accel_x = a.acceleration.x - accelBiasX;
    accel_y = a.acceleration.y - accelBiasY;
  } else {
    gyro_dps = gyro_dps_prev;
  }
  omega_p = GYRO_SIGN * gyro_dps * PI / 180.0f;     // rad/s, model convention

  // ESC current sense, raw counts (only used as ratios, so no scaling). Decimated to
  // 20 Hz because analogRead() costs ~100 us on STM32duino (it re-inits the ADC).
  static uint8_t adcDiv = 0;
  if (++adcDiv >= 10) {
    adcDiv = 0;
    TIME_BLOCK(st_adc, { escCurRaw = (uint16_t)analogRead(ESC_CUR_PIN); });
  }

  // Wheel velocity with a plausibility check. The wheel cannot change by more than
  // ~2.3 rad/s per cycle, so a bigger jump is a glitch in the velocity estimate.
  // Hold the last good value, but resync after 3 cycles so a real change gets through.
  float w_w_raw = motor.shaft_velocity;
  static float w_w_prev = 0.0f;
  static bool  w_w_init = false;
  static int   w_w_rejStreak = 0;
  if (w_w_init && fabsf(w_w_raw - w_w_prev) > WW_MAX_JUMP && w_w_rejStreak < 3) {
    w_w_raw = w_w_prev;
    w_w_rejStreak++;
    wwRejects++;
  } else {
    w_w_rejStreak = 0;
  }
  w_w_prev = w_w_raw; w_w_init = true;
  omega_w = w_w_raw;

  theta = wrapPi(theta + omega_p * dt);

  // Pose estimator. Runs even when the controller is idle so the estimate is
  // ready the moment control is enabled.
  est_predict(dt, theta, accel_x, accel_y);
  {
    PiPose pp;
    if (pi_getPose(&pp)) est_correct(&pp, theta, omega_p);
  }

  // Translation runs before the heading early-return below: the wheel being idle
  // says nothing about whether the fans should hold station.
  {
    EstState es;
    bool haveEst = est_get(&es);
    if (haveEst) trans_update(&es, pi_state() == PI_FRESH);
    else         trans_update(NULL, false);

    // Record the move, after trans_update so the row has this cycle's command.
    // Skipped during a TC probe, which records itself and never enables the
    // translation controller.
    if (tcap_active() && !tcalActive) {
      if (haveEst) tcap_sample(&es, pi_ageUs(), accel_x, accel_y, omega_p);
      // Stop the recorder when translation stops (trip, TS or X). Not during a
      // stepped dock, which disables translation between hops and stops the
      // capture itself. tcap_active() is re-checked because a full buffer stops
      // the recorder inside tcap_sample.
      if (dockPhase == DOCK_OFF && tcap_active() && !trans_enabled()) {
        tcap_stop(trans_tripped() ? (strstr(trans_tripReason(), "TIMEOUT")
                                       ? "timeout" : "diverged")
                                  : "operator_stop");
      }
    }

    // TC probe sequencer
    if (tcalActive) {
      uint32_t el = millis() - tcalStartMs;
      float pct = (el >= TCAL_PRE_MS && el < TCAL_PRE_MS + TCAL_HOLD_MS)
                    ? tcalPct : 0.0f;
      applyFanStep(pct);
      float p[4] = {0, 0, 0, 0};
      if (fanSel >= 1 && fanSel <= 4) p[fanSel - 1] = pct;
      if (haveEst) tcap_sampleProbe(&es, pi_ageUs(), p, accel_x, accel_y, omega_p);
      if (el >= TCAL_TOTAL_MS) {
        tcalActive = false;
        applyFanStep(0.0f);
        tcap_stop("fan_probe");
      }
    }

    // TB bias-at-idle sequencer
    if (tbiasActive) {
      uint32_t el = millis() - tbiasStartMs;
      fans_setAll(TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT);
      if (el >= TBIAS_SPIN_MS) {
        // accel_x/y already have the old bias removed, so this sums the residual.
        tbiasSumX += accel_x; tbiasSumY += accel_y; tbiasN++;
      }
      if (el >= TBIAS_SPIN_MS + TBIAS_MEAS_MS) {
        tbiasActive = false;
        fans_setAll(0, 0, 0, 0);
        if (tbiasN > 0) {
          float dx = (float)(tbiasSumX / tbiasN), dy = (float)(tbiasSumY / tbiasN);
          accelBiasX += dx; accelBiasY += dy;
          est_zupt();
          telem_print(String("BIAS @ IDLE: residual was (") + String(dx, 4) + ", "
                      + String(dy, 4) + ") m/s^2 over " + String(tbiasN)
                      + " samples -> bias now (" + String(accelBiasX, 4) + ", "
                      + String(accelBiasY, 4) + "). Compare with B (fans off).");
        }
      }
    }

    // Rest detection, only while nothing is driving the fans. During a commanded
    // move the platform is slow at both ends and a ZUPT there would erase real velocity.
    if (!trans_enabled() && !tcalActive && !tbiasActive) {
      float am = sqrtf(accel_x * accel_x + accel_y * accel_y);
      bool quiet = (am < REST_ACCEL) && (fabsf(omega_p) < REST_OMEGA);
      if (quiet) {
        if (restQuietMs == 0) restQuietMs = millis();
        if (!restLatched && (millis() - restQuietMs) >= REST_MS) {
          restLatched = true;
          est_zupt();
          // One averaged vision fix per stop: stationary is when vision is best and
          // dead reckoning has drifted the most.
          if (pi_state() == PI_FRESH) est_snapBegin(EST_SNAP_FRAMES);
          EstState e;
          if (est_get(&e)) {
            telem_print(String("AT REST: magnet at (") + String(e.mag_x, 3)
                        + ", " + String(e.mag_y, 3) + ")  psi="
                        + String(degrees(e.psi), 1) + "deg   dock is "
                        + String(dockDist(e) * 100.0f, 1)
                        + " cm away.  TG to dock, X to stop.");
          }
        }
      } else {
        restQuietMs = 0;
        if (am > MOVING_ACCEL) restLatched = false;
      }
    }

    // Stepped docking sequencer
    if (dockPhase != DOCK_OFF) {
      EstState de;
      bool haveDe = est_get(&de);
      float rem = haveDe ? dockDist(de) : 0.0f;

      if (dockPhase == DOCK_MOVE) {
        if (!trans_enabled()) {
          // Tripped or stopped. Do not quietly start another leg.
          dockPhase = DOCK_OFF;
          tcap_stop("dock_aborted");
          telem_print("DOCK ABORTED: translation stopped mid-leg.");
        } else if (trans_lastErr() < TRANS_DEADZONE) {
          trans_enable(false);
          restLatched = false; restQuietMs = 0;
          dockPhase = DOCK_SETTLE; dockPhaseMs = millis();
        }
      } else if (dockPhase == DOCK_SETTLE) {
        // Wait until the platform is at rest and the averaged vision fix is in.
        // The timeout keeps a missing Pi from stalling the sequence.
        bool ready = (restLatched && est_snapDone())
                     || (millis() - dockPhaseMs) > DOCK_SETTLE_MS;
        if (ready && haveDe) {
          if (rem < DOCK_DONE_M) {
            dockPhase = DOCK_OFF;
            tcap_stop("docked");
            telem_print(String("DOCKED: magnet at (") + String(de.mag_x, 3)
                        + ", " + String(de.mag_y, 3) + "), " + String(rem * 100.0f, 1)
                        + " cm from target, in " + String(dockHops) + " hops.");
          } else if (dockHops >= DOCK_MAX_HOPS) {
            dockPhase = DOCK_OFF;
            tcap_stop("dock_gave_up");
            telem_print(String("DOCK GAVE UP after ") + String(dockHops)
                        + " hops, still " + String(rem * 100.0f, 1) + " cm out.");
          } else {
            // Next waypoint: one hop along the line to the dock, or the dock itself.
            float ux = (0.0f - de.mag_x) / rem;
            float uy = (EST_D_DOCK - de.mag_y) / rem;
            float step = (rem < dockHopM) ? rem : dockHopM;
            trans_setTarget(de.mag_x + ux * step, de.mag_y + uy * step);
            // Drop any snap still collecting, or its frames would be adopted mid-hop.
            est_snapCancel();
            trans_enable(true);
            dockHops++;
            dockPhase = DOCK_MOVE; dockPhaseMs = millis();
            telem_print(String("hop ") + String(dockHops) + ": " + String(step * 100.0f, 1)
                        + " cm, " + String(rem * 100.0f, 1) + " cm to go");
          }
        }
      }
    }

    // Take-once rather than polling tcap_pending(), which stays true for the whole
    // dump and would queue the same dump every cycle.
    if (tcap_takePending()) telem_run(dumpTransCapture);
  }

  if (!controllerEnabled || ctrlMode == CTRL_IDLE) {
    motor.target = 0.0f;
    lastAlpha = 0.0f; lastU = 0.0f;
    return;
  }

  // Open-loop pulse: hold a voltage for OPEN_PULSE_MS, no feedback.
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

  // Compensation test: spin up open loop, then command alpha = 0 with only the
  // back-EMF compensation active. The wheel must coast down. Holding speed means no
  // margin; speeding up means compFrac is too high and the closed loop will run away.
  if (ctrlMode == CTRL_COMP) {
    if (fabsf(omega_w) > WHEEL_SAT_LIMIT) {
      stopMotor("wheel saturation " + String(omega_w, 1) + " rad/s");
      return;
    }
    float u;
    if (millis() - openStartMs < OPEN_PULSE_MS) {
      u = openVolts;
    } else {
      u = compFrac * A_2 * omega_w / A_1;
    }
    if (u >  VOLTAGE_LIMIT) u =  VOLTAGE_LIMIT;
    if (u < -VOLTAGE_LIMIT) u = -VOLTAGE_LIMIT;
    motor.target = u;
    lastU = u; lastAlpha = 0.0f;
    return;
  }

  // Wheel saturation: hard stop. The platform spins as the wheel dumps its momentum.
  if (fabsf(omega_w) > WHEEL_SAT_LIMIT) {
    stopMotor("wheel saturation " + String(omega_w, 1) + " rad/s");
    telem_run(dumpCapture);   // trajectory up to the abort
    return;
  }

  // 1. Error and two-stage deadzone.
  // This chatters around FINE_WW (coarse -> wheel bleeds -> fine -> push -> repeat).
  // That is intentional: a latching version was tested and was worse, because it
  // kept pushing against stiction it could not beat and wound the wheel up.
  float e = wrapPi(target - theta);
  float dz = (fabsf(omega_w) < FINE_WW) ? deadzoneFine : deadzone;
  bool outside   = fabsf(e) > dz;
  bool notMoving = fabsf(omega_p) < W_MOVING;

  // Stall detection: outside the deadzone, platform stationary, wheel already fast.
  // Back off and let the passive unwind restore authority.
  if (outside && notMoving && fabsf(omega_w) > STALL_WW) {
    if (stallStartMs == 0) stallStartMs = millis();
    else if (millis() - stallStartMs > STALL_MS && !stallHold && !parked) {
      stallHold = true;
      stallHoldUntil = millis() + STALL_HOLD_MS;
      stallCount++;
      if (stallCount > MAX_STALL_RETRIES) {
        parked = true;
        printBoth("PARKED at " + String(degrees(e), 1) + " deg after "
                  + String(MAX_STALL_RETRIES) + " stalls. Raise A, or accept."
                  " Send a new target to resume.");
      } else {
        printBoth("STALL at " + String(degrees(e), 1) + " deg, ww="
                  + String(omega_w, 1) + ", backing off ("
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
    alpha = 0.0f;

  } else if (outside) {
    // 2. PD. Minus on error, plus on rate (see header).
    alpha = -K_theta * e + K_omega * omega_p;

    // 3. Friction feedforward (see header for the signs).
    float ff;
    if (fabsf(omega_p) > W_MOVING)
      ff = -(A_MOVING + A_VISCOUS * fabsf(omega_p)) * signf(omega_p);
    else
      ff =  A_FRICTION * signf(alpha);
    alpha += ffFrac * ff;

    // Authority cap, only while stationary. Pushing hard against stiction just winds
    // the wheel up; while moving the manoeuvre finishes first, so no cap there.
    // Must stay well above A_FRICTION or the platform may never break free.
    if (notMoving) {
      if (alpha >  ALPHA_STALL_MAX) alpha =  ALPHA_STALL_MAX;
      if (alpha < -ALPHA_STALL_MAX) alpha = -ALPHA_STALL_MAX;
    }

  } else {
    // Inside tolerance: command nothing and let stiction hold the platform.
    alpha = 0.0f;
  }

  // 4. Feedback linearisation: cancel back-EMF and the wheel pole.
  float u = (alpha + compFrac * A_2 * omega_w) / A_1;

  // 5. Saturate and command.
  if (u >  VOLTAGE_LIMIT) u =  VOLTAGE_LIMIT;
  if (u < -VOLTAGE_LIMIT) u = -VOLTAGE_LIMIT;
  motor.target = u;

  lastAlpha = alpha;
  lastU = u;
}

// Single entry point for every capture so the window and metadata always match.
static void startCapture(const char* mode, uint32_t windowMs, float pct, int sel,
                         bool translation) {
  // stepCount advances for every capture. capture_calibration.py names files from
  // the "test N/M" marker, so a repeated number would overwrite the earlier file.
  stepCount++;
  capN = 0;
  capModeName = mode;
  capWindowMs = windowMs;
  capThrottle = pct;
  capFanSel   = sel;
  capTranslation = translation;
  capKeepWheel   = false;      // callers set these after calling
  capOpenLoop    = false;
  capCompTest    = false;
  capStartMs  = millis();
  capturing   = true;
}

// Translation plant ID thrust step (I). One capture gives both model terms:
//   thrust phase: x_ddot = A(throttle) - A_c
//   coast phase:  x_ddot = -A_c
static float    transPct     = 0.0f;
static uint32_t transPreMs   = 500;    // quiet baseline before thrust
static uint32_t transHoldMs  = 1000;   // thrust
static uint32_t transTotalMs = 4000;   // remainder is coast-down
#define YAW_TOTAL_MS 7000              // Q: long enough to see the wheel settle

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  float v = (s.length() > 1) ? s.substring(1).toFloat() : 0.0f;

  // A capture writes cap_* while a queued dump may still be reading it on telemTask,
  // so refuse a new capture until the dump is done. Translation subcommands
  // (T + letter) use the separate tcap buffer and are not blocked here.
  bool transSub = (c == 'T' && s.length() > 1 && isAlpha(s.charAt(1)));
  if ((c == 'T' || c == 'O' || c == 'C' || c == 'Y' || c == 'I')
      && !transSub && telem_busy()) {
    printBoth("busy: previous capture still dumping. Wait, then resend.");
    return;
  }

  switch (c) {
    case 'O':
      // Open-loop sign check. Positive volts should drive the wheel positive and
      // theta negative. If theta goes positive, GYRO_SIGN is wrong.
      openVolts = constrain(v, -OPEN_PULSE_VMAX, OPEN_PULSE_VMAX);
      theta = 0.0f;
      startCapture("openloop", CAPTURE_MS, 0.0f, 0, false);
      capOpenLoop = true;
      openStartMs = millis();
      ctrlMode = CTRL_OPEN; controllerEnabled = true;
      printBoth("OPEN-LOOP pulse " + String(openVolts, 2) + "V for "
                + String(OPEN_PULSE_MS) + "ms, capturing...");
      printBoth("  expect: wheel POSITIVE, theta NEGATIVE (for positive volts)");
      break;
    case 'T':
      // T<digit> is a heading step. T<letter> is a translation command.
      if (s.length() > 1 && isAlpha(s.charAt(1))) {
        char sub = toupper(s.charAt(1));
        float v2 = (s.length() > 2) ? s.substring(2).toFloat() : 0.0f;
        float tx, ty;
        trans_getTarget(&tx, &ty);
        // TX/TY are offsets from the magnet's current position (easier by hand than
        // absolute dock coordinates). TZ targets the dock itself.
        if (sub == 'X' || sub == 'Y' || sub == 'Z') {
          EstState e;
          if (!est_get(&e)) {
            printBoth("no fix yet, cannot place a target relative to the magnet");
            break;
          }
          if      (sub == 'X') trans_setTarget(e.mag_x + v2, ty);
          else if (sub == 'Y') trans_setTarget(tx, e.mag_y + v2);
          else                 trans_setTarget(0.0f, EST_D_DOCK);
        }
        // TT: start the move
        else if (sub == 'T') {
          {
            EstState ev;
            if (!est_get(&ev)) {
              printBoth("REFUSED: no pose. Send TI first (TI = docked, TI<m> = "
                        "centre <m> out) so dead reckoning has an origin.");
              break;
            }
          }
          if (tcap_pending()) {
            printBoth("REFUSED: previous capture still dumping. Wait for "
                      "'--- capture end ---'.");
            break;
          }
          // Heading control must be running. The allocator rotates thrust into the
          // body frame by psi, and the fans themselves yaw the platform, so an
          // uncontrolled heading misdirects the thrust while it is thrusting.
          if (!controllerEnabled || ctrlMode == CTRL_IDLE) {
            printBoth("REFUSED: heading control is IDLE. Send H<deg> first. "
                      "Translation steers by psi and the fans yaw the platform.");
            break;
          }
          // Refuse implausibly long moves. The usual cause is an axis that was never
          // set and still holds the power-on default of 0, which is the wall.
          {
            EstState e;
            if (est_get(&e)) {
              float ex = tx - e.mag_x, ey = ty - e.mag_y;
              float d = sqrtf(ex * ex + ey * ey);
              if (d > TRANS_MAX_MOVE) {
                printBoth("REFUSED: that target is " + String(d * 100.0f, 0)
                          + " cm away. Set both axes (TX<off> and TY<off>); an "
                          "unset one is still 0.0, which is the wall.");
                break;
              }
            }
          }
          trans_enable(true);
          trans_getTarget(&tx, &ty);
          tcap_start(tx, ty, ++stepCount);
          printBoth("TRANSLATE -> magnet target (" + String(tx, 3) + ", "
                    + String(ty, 3) + ")  PROPS ARE LIVE, capturing at "
                    + String(200 / TCAP_DECIM) + " Hz");
          break;
        }
        else if (sub == 'S') { trans_enable(false); printBoth("translation STOP"); break; }
        // TP/TD/TF: translation gains
        else if (sub == 'P' || sub == 'D' || sub == 'F') {
          float kp, kd, ff;
          if      (sub == 'P') trans_setGains(v2, -1.0f, -1.0f);
          else if (sub == 'D') trans_setGains(-1.0f, v2, -1.0f);
          else                 trans_setGains(-1.0f, -1.0f, v2);
          trans_getGains(&kp, &kd, &ff);
          float wn = (kp > 0.0f) ? sqrtf(kp) : 0.0f;
          printBoth("trans gains: Kp=" + String(kp, 2) + " Kd=" + String(kd, 2)
                    + " ff=" + String(ff, 2)
                    + "   -> omega_n=" + String(wn, 2)
                    + " zeta=" + String((wn > 0.0f) ? kd / (2.0f * wn) : 0.0f, 2)
                    + " t_settle=" + String((wn > 0.0f) ? 5.714f / wn : 0.0f, 1) + "s");
          break;
        }
        // TI[m]: declare the start pose on the dock centreline, in magnet
        // coordinates. No argument means docked. Dead reckoning integrates from here.
        else if (sub == 'I') {
          if (v2 > 0.0f) est_setMagnetPose(0.0f, v2);
          else           est_setMagnetPose(0.0f, EST_D_DOCK);
          restLatched = false; restQuietMs = 0;
          EstState e0; est_get(&e0);
          printBoth(String("pose SET: magnet (") + String(e0.mag_x, 3) + ", "
                    + String(e0.mag_y, 3) + (v2 > 0.0f ? ") " : ") = DOCKED ")
                    + " centre (" + String(e0.x, 3) + ", " + String(e0.y, 3)
                    + ")  magLat=" + String(est_getMagLat(), 3));
          break;
        }
        // TG: go dock. Not subject to TRANS_MAX_MOVE since the target is the dock by
        // construction. Also points the platform square to the dock so the magnet mates.
        else if (sub == 'G') {
          if (!fans_armed() || fans_killed()) {
            printBoth("fans not ready, R to re-arm"); break;
          }
          EstState e;
          if (!est_get(&e)) { printBoth("no pose yet, send TI first"); break; }
          if (tcap_pending()) { printBoth("REFUSED: capture still dumping"); break; }
          target = wrapPi(est_psiOffset());   // wheel target that makes psi = 0
          ctrlMode = CTRL_HOLD; controllerEnabled = true;
          tcap_start(0.0f, EST_D_DOCK, ++stepCount);
          // Start in SETTLE so the first action is a rest check and vision fix,
          // since the estimate is least trustworthy right after a hand move.
          dockHops = 0;
          dockPhase = DOCK_SETTLE; dockPhaseMs = millis();
          restLatched = false; restQuietMs = 0;
          printBoth("DOCKING from (" + String(e.mag_x, 3) + ", "
                    + String(e.mag_y, 3) + ") in " + String(dockHopM * 100.0f, 0)
                    + " cm hops, max " + String(TRANS_V_MAX * 100.0f, 0)
                    + " cm/s.  PROPS ARE LIVE.  X to stop.");
          break;
        }
        // TR<deg>: accelerometer-to-body rotation
        else if (sub == 'R') {
          if (s.length() > 2) est_setAccRot(v2);
          printBoth("accel->body rotation = " + String(est_getAccRot(), 1) + " deg");
          break;
        }
        // TE<0|1>: brake assist, off by default
        else if (sub == 'E') {
          if (s.length() > 2) trans_setBrakeAssist(v2 >= 0.5f);
          printBoth(String("brake assist ") + (trans_getBrakeAssist()
                    ? "ON: friction is left uncancelled while decelerating"
                    : "OFF (default): drag cancelled whenever moving"));
          break;
        }
        // TL<m/s>: approach speed cap
        else if (sub == 'L') {
          if (s.length() > 2) trans_setVmax(v2);
          printBoth("approach speed cap = " + String(trans_getVmax() * 100.0f, 1)
                    + " cm/s   (taper starts "
                    + String(trans_getVmax() / 0.50f * 100.0f, 1) + " cm out)");
          break;
        }
        // TH<m>: hop length for the stepped dock. Shorter hops drift less but each
        // restart has to break stiction again.
        else if (sub == 'H') {
          if (s.length() > 2 && v2 >= 0.02f && v2 <= 0.30f) dockHopM = v2;
          printBoth("dock hop = " + String(dockHopM * 100.0f, 1) + " cm");
          break;
        }
        // TW<0|1>: arm/disarm the INA219 power trips. Monitoring keeps running either
        // way. The fans sag the shared pack below 10 V under thrust, which trips the
        // undervoltage stop mid-dock. That trip is also the only LiPo protection, so
        // do not leave it off unattended.
        else if (sub == 'W') {
          bool on = (v2 >= 0.5f);
          if (on) safety_setPowerLimits(10.0f, 2500.0f);
          else    safety_setPowerLimits(0.0f, 0.0f);
          printBoth(on ? "power trips ARMED (<10.0V, >2500mA)"
                       : "power trips DISABLED. Monitoring still runs, no "
                         "safe-stop. Watch the pack yourself.");
          break;
        }
        // TM<m>: magnet lateral offset, + = left of the camera axis
        else if (sub == 'M') {
          if (s.length() > 2) est_setMagLat(v2);
          printBoth("magnet lateral offset = " + String(est_getMagLat(), 4) + " m");
          break;
        }
        // TA<deg>: rotate the whole fan table (0/90/180/270)
        else if (sub == 'A') {
          if (s.length() > 2) trans_setFanRot(v2);
          printBoth("fan table rotation = " + String(trans_getFanRot(), 0)
                    + " deg  (fan1 " + String(-40.0f + trans_getFanRot(), 0)
                    + ", fan2 " + String(50.0f + trans_getFanRot(), 0)
                    + ", fan3 " + String(230.0f + trans_getFanRot(), 0)
                    + ", fan4 " + String(140.0f + trans_getFanRot(), 0) + ")");
          break;
        }
        // TB: measure accel bias with the fans at idle, the condition the estimator
        // runs in. Prop vibration shifts the MEMS accelerometer's DC offset, so the
        // fans-off bias from B is slightly wrong, and the double integration turns
        // that into position drift. Platform must be still.
        else if (sub == 'B') {
          if (!fans_armed() || fans_killed()) {
            printBoth("fans not ready, R to re-arm"); break;
          }
          printBoth("BIAS @ IDLE: spinning all four at "
                    + String(TRANS_IDLE_PCT, 0) + "%. Hold still, ~4 s. KEEP CLEAR.");
          tbiasActive = true; tbiasStartMs = millis();
          tbiasSumX = tbiasSumY = 0.0; tbiasN = 0;
          break;
        }
        // TN: snap position to an averaged vision fix now (manual version of the
        // at-rest snap)
        else if (sub == 'N') {
          if (pi_state() != PI_FRESH) { printBoth("no fresh pose to snap to"); break; }
          est_snapBegin(EST_SNAP_FRAMES);
          printBoth("adopting position from the next " + String(EST_SNAP_FRAMES)
                    + " vision frames (averaged)");
          break;
        }
        // TV<gain>: vision position correction gain. TV0 runs pure IMU. Heading
        // correction is unaffected.
        else if (sub == 'V') {
          float a, b, c;
          if (s.length() > 2) est_setGains(v2 <= 0.0f ? 1e-6f : v2, -1.0f, -1.0f);
          est_getGains(&a, &b, &c);
          printBoth("vision position gain aPos=" + String(a, 3)
                    + (a < 0.001f ? "  (OFF, pure IMU)" : ""));
          break;
        }
        // TC<pct>: single-fan direction probe (see TCAL_* above). Needs heading
        // held, a pose, and a single fan selected.
        else if (sub == 'C') {
          if (!fans_armed() || fans_killed()) {
            printBoth("fans not ready, R to re-arm"); break;
          }
          if (tcap_pending()) { printBoth("REFUSED: capture still dumping"); break; }
          if (!controllerEnabled || ctrlMode == CTRL_IDLE) {
            printBoth("REFUSED: send H<deg> first. The probe needs heading held.");
            break;
          }
          {
            EstState ev;
            if (!est_get(&ev)) { printBoth("REFUSED: no pose, send TI first"); break; }
          }
          if (fanSel < 1 || fanSel > 4) {
            printBoth("select a single fan first with S1..S4"); break;
          }
          if (v2 <= 0.0f) v2 = 45.0f;
          tcalPct = v2; tcalActive = true; tcalStartMs = millis();
          tcap_startProbe(fanSel, v2, ++stepCount);
          printBoth("FAN PROBE: fan" + String(fanSel) + " at " + String(v2, 0)
                    + "%: " + String(TCAL_PRE_MS) + "ms quiet / "
                    + String(TCAL_HOLD_MS) + "ms thrust / coast. KEEP CLEAR.");
          break;
        }
        else { printBoth("TI[m] set pose | TG dock | TX/TY offsets | TZ dock target | "
                         "TT go | TS stop | TP/TD/TF gains | TR<deg> accel rot | "
                         "TE brake assist | TL<m/s> speed cap | TH<m> hop | "
                         "TW power trips | TM<m> magnet offset | TA<deg> fan rot | "
                         "TB idle bias | TN vision snap | TV<g> vision gain | "
                         "TC<pct> fan probe"); break; }
        // After TX/TY/TZ: echo the magnet position and the implied move, so a wrong
        // target (e.g. the centre instead of the magnet) is obvious before TT.
        trans_getTarget(&tx, &ty);
        {
          EstState e;
          if (est_get(&e)) {
            float ex = tx - e.mag_x, ey = ty - e.mag_y;
            printBoth("target (" + String(tx, 3) + ", " + String(ty, 3) + ")"
                      + "   magnet now (" + String(e.mag_x, 3) + ", "
                      + String(e.mag_y, 3) + ")"
                      + "   -> move " + String(sqrtf(ex * ex + ey * ey) * 100.0f, 1)
                      + " cm  d=(" + String(ex, 3) + "," + String(ey, 3) + ")");
          } else {
            printBoth("target (" + String(tx, 3) + ", " + String(ty, 3)
                      + ")   [no fix yet, cannot show the implied move]");
          }
        }
        break;
      }
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      target = wrapPi(radians(v));
      startCapture("heading_step", CAPTURE_MS, 0.0f, 0, false);
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
      // Compensation test: spin up at V, then alpha = 0. Watch omega_w.
      openVolts = constrain(v, -OPEN_PULSE_VMAX, OPEN_PULSE_VMAX);
      theta = 0.0f;
      startCapture("comptest", CAPTURE_MS, 0.0f, 0, false);
      capCompTest = true;
      openStartMs = millis();
      ctrlMode = CTRL_COMP; controllerEnabled = true;
      printBoth("COMP test " + String(openVolts, 2) + "V spin-up, compFrac="
                + String(compFrac, 2) + ", capturing...");
      printBoth("  expect: wheel coasts DOWN after the pulse. Growth = too high.");
      break;
    case 'A':
      // A<val> static, AM<val> moving, AV<val> viscous
      if (s.length() > 1 && toupper(s.charAt(1)) == 'M')
        A_MOVING = constrain(s.substring(2).toFloat(), 0.0f, 80.0f);
      else if (s.length() > 1 && toupper(s.charAt(1)) == 'V')
        A_VISCOUS = constrain(s.substring(2).toFloat(), 0.0f, 60.0f);
      else
        A_FRICTION = constrain(v, 0.0f, 80.0f);
      printGains();
      break;
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
                  + " deg floor. Raise F first or this will stall.");
      printGains();
      break;
    case 'G': printGains(); break;
    case 'B':
      ctrlMode = CTRL_IDLE; motor.target = 0.0f;
      printBoth("hold still, measuring bias...");
      measureGyroBias();
      break;
    case 'M':
      // M prints min/mean/max for each timed block, M! resets them.
      if (s.length() > 1 && s.charAt(1) == '!') {
        stat_reset(&st_adc);
        stat_reset(&st_foc);   stat_reset(&st_move);  stat_reset(&st_mpu);
        stat_reset(&st_ina);   stat_reset(&st_law);   stat_reset(&st_telem);
        stat_reset(&st_period);
        printBoth("timing stats reset");
      } else {
        telem_run(printTimingStats);
      }
      break;
    case 'U':
      telem_run(printSystemReport);
      break;
    case 'X': stopMotor("operator"); break;
    case 'J': {
      // DSHOT special command to the selected fan. 20/21 = direction normal/reversed,
      // 7/8 = direction 1/2, 12 = save (needed to persist any of them).
      // Reverse fan 4 permanently: S4, J21, J12, power-cycle.
      int cmd = (int)v;
      if (fanSel < 1 || fanSel > 4) {
        printBoth("J needs a single channel selected first: send S1..S4 (not S0)");
        break;
      }
      if (!fans_sendCommand(fanSel, (uint16_t)cmd, 10)) {
        printBoth("ESC command refused: needs fans armed, not killed, nothing "
                  "spinning, cmd 1-47, and no burst already running");
        break;
      }
      printBoth("ESC cmd " + String(cmd) + " -> fan" + String(fanSel)
                + " x10 frames (motors held at 0)");
      break;
    }
    case 'Y': {
      // Hand-push coast-down capture. Coulomb friction gives a constant deceleration,
      // so A_c is a straight-line fit. The wheel is turned off because its reaction
      // torque yaws the platform and contaminates the accelerometer. Fans untouched.
      uint32_t ms = (v > 0.0f) ? (uint32_t)v : 5000;
      if (ms > 7000) ms = 7000;              // MAX_CAP limit
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      startCapture("coast_down", ms, 0.0f, 0, true);
      printBoth("COAST capture " + String(ms) + " ms. Push the platform now.");
      printBoth("  (fans untouched, wheel off)");
      break;
    }
    case 'Q': {
      // Yaw coupling: the I thrust step with heading control engaged. A fan whose
      // thrust line misses the CoM applies a yaw torque, and the wheel acceleration
      // needed to hold heading measures it directly.
      if (!fans_armed() || fans_killed()) {
        printBoth("fans not ready, send R to re-arm");
        break;
      }
      transPct    = v;
      transActive = true;
      // Hold the current heading so all wheel effort is attributable to the fan.
      target = theta;
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      ctrlMode = CTRL_HOLD; controllerEnabled = true;
      startCapture("yaw_coupling", YAW_TOTAL_MS, v, fanSel, true);
      capKeepWheel = true;
      printBoth("YAW COUPLING " + String(v, 1) + "% on "
                + String(fanSel == 0 ? "ALL" : String(fanSel))
                + ". Heading control active, the wheel will spin up to fight the torque.");
      printBoth("  if omega_w walks toward " + String(WHEEL_SAT_LIMIT, 0)
                + " rad/s the thrust line needs mechanical correction. X aborts.");
      break;
    }
    case 'I': {
      // Thrust step on the selected fan(s): quiet, thrust, coast. Wheel off, as in Y.
      if (!fans_armed() || fans_killed()) {
        printBoth("fans not ready (armed=" + String(fans_armed())
                  + " killed=" + String(fans_killed()) + "), R to re-arm");
        break;
      }
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      transPct    = v;
      transActive = true;
      startCapture("thrust_step", transTotalMs, v, fanSel, true);
      printBoth("THRUST STEP " + String(v, 1) + "% on "
                + String(fanSel == 0 ? "ALL" : String(fanSel))
                + ": " + String(transPreMs) + "ms quiet / " + String(transHoldMs)
                + "ms thrust / " + String(transTotalMs - transPreMs - transHoldMs)
                + "ms coast");
      printBoth("  KEEP CLEAR. X aborts.");
      break;
    }
    // Manual fan commands (raw throttle)
    case 'S': {
      int n = (int)v;
      if (n < 0 || n > 4) { printBoth("fan select must be 0 (all) or 1-4"); break; }
      fanSel = n;
      printBoth("fan select: " + String(n == 0 ? "ALL" : String(n)));
      break;
    }
    case 'L': {
      if (fans_killed()) {
        printBoth("fans are KILLED, send R to re-arm first");
        break;
      }
      if (!fans_armed()) {
        printBoth("fans still arming (~1 s from boot), wait for 'fans: armed'");
        break;
      }
      if (fanSel == 0) fans_setAll(v, v, v, v);
      else             fans_setThrottle(fanSel, v);
      // Echo what was applied (the setter clamps to FAN_THROTTLE_MAX).
      printBoth("fans -> " + String(fans_pct(1), 1) + " / " + String(fans_pct(2), 1)
                + " / " + String(fans_pct(3), 1) + " / " + String(fans_pct(4), 1)
                + " %   (cap " + String(FAN_THROTTLE_MAX, 0) + "%, auto-zero after "
                + String(FAN_CMD_TIMEOUT_MS / 1000) + " s idle)");
      break;
    }
    case 'E': {
      // Encoder diagnostic with the motor off, so the wheel can be turned by hand.
      // Reads the cached values from focTask; SPI2 belongs to focTask.
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      capturing = false;
      printBoth("ENC DIAG: motor OFF. Hand-turn the wheel and watch.");
      printBoth("i,shaft_angle_rad,single_turn_deg,shaft_vel_rad_s");
      for (int i = 0; i < 30; i++) {
        float ang = motor.shaft_angle;
        float vel = motor.shaft_velocity;
        float turn = ang - floorf(ang / (2.0f * PI)) * (2.0f * PI);
        telem_print(String(i) + "," + String(ang, 4) + ","
                  + String(degrees(turn), 2) + "," + String(vel, 3));
        safety_kick();
        vTaskDelay(pdMS_TO_TICKS(100));   // blocks, so telemTask can write the output
      }
      printBoth("ENC DIAG done (motor still OFF; send R to re-enable).");
      break;
    }
    case 'V': {
      // Constant-voltage drive with an encoder stream at ~50 Hz until any key.
      // focTask does the commutation. Start small (V1, V2).
      float volts = constrain(v, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
      controllerEnabled = false; ctrlMode = CTRL_IDLE; capturing = false;
      motor.enable();
      motor.target = volts;
      printBoth("MANUAL DRIVE " + String(volts, 2) + "V. Press any key (or X) to STOP.");
      printBoth("t_ms,shaft_angle_rad,single_turn_deg,shaft_vel_rad_s,V");
      // Watch the RX byte counter instead of reading the port, since commsTask is the
      // only reader. The key that stops this is then handled as a normal command.
      unsigned long t0 = millis(), lastP = 0;
      uint32_t rxAtStart = commands_rxBytes();
      while (commands_rxBytes() == rxAtStart) {
        motor.target = volts;
        if (millis() - lastP >= 20) {
          lastP = millis();
          float ang  = motor.shaft_angle;
          float vel  = motor.shaft_velocity;
          float turn = ang - floorf(ang / (2.0f * PI)) * (2.0f * PI);
          telem_print(String(millis() - t0) + "," + String(ang, 4) + ","
                    + String(degrees(turn), 2) + "," + String(vel, 3) + ","
                    + String(volts, 2));
        }
        safety_kick();
        vTaskDelay(1);   // let telemTask drain
      }
      motor.target = 0.0f;
      printBoth("MANUAL DRIVE stopped (target 0).");
      break;
    }
    case 'R':
      // Re-enable the controller (IDLE) and undo a fan hard kill. fanTask repeats its
      // ~1 s zero-throttle arming before it accepts throttle again.
      controllerEnabled = true; ctrlMode = CTRL_IDLE;
      fans_rearm();
      printBoth("controller re-enabled (IDLE); fans re-arming (~1 s)");
      break;
    default:
      stopMotor("unrecognised command '" + s + "'");
      break;
  }
}

// commsTask assembles lines; the control task runs them here, at a fixed point in
// its cycle, so a command never lands in the middle of controlUpdate().
void pollSerial() {
  String line;
  while (commands_next(line)) handleLine(line);
}

static void focTask(void*);
static void focTickNotify(void);

// Hardware bring-up, run inside the control task so initFOC() uses the task's stack
// and FPU context and the fault hooks are already live.
static void hwSetup() {
  // Arm the fault handler before motor bring-up, since initFOC() spins the wheel.
  faults_init(PIN_ENABLE, LED_BUILTIN,
              []() { motor.target = 0.0f; motor.disable(); driver.disable(); });

  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  Wire.begin();
  Wire.setClock(400000);   // at 100 kHz an IMU read takes ~3.7 ms

  if (!mpu.begin()) {
    printBoth("[MPU6050] NOT FOUND, cannot run heading control.");
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  printBoth("[MPU6050] OK");

  // begin() only confirms an I2C ack; without setCalibration the readings are inf.
  // A missing INA219 is not fatal, monitoring is just skipped.
  inaPresent = ina219.begin();
  if (inaPresent) {
    ina219.setCalibration_32V_2A();
    printBoth("[INA219] OK (power monitoring active)");
  } else {
    printBoth("[INA219] NOT FOUND, power monitoring disabled (not fatal)");
  }

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

  // Hand commutation to focTask, clocked by TIM9. Nothing commutates until
  // focTick_init() starts the timer.
  configASSERT(xTaskCreate(focTask, "foc", 256, NULL, 4, &hFoc) == pdPASS);
  focTick_attach(focTickNotify);
  focTick_init(FOC_RATE_HZ);

  printBoth("");
  printBoth("=== Heading controller ===");
  printBoth("Platform must be STILL for bias measurement.");
  measureGyroBias();
  theta = 0.0f; target = 0.0f;

  printGains();
  printBoth("Heading: T<deg> step+capture | H<deg> hold | Z zero | X stop | R resume/re-arm");
  printBoth("Tuning:  P/D gains | F<0-1.5> ff | K<0-1.2> compFrac | W/N coarse/fine dz deg");
  printBoth("         A/AM/AV static/moving/viscous friction");
  printBoth("Status:  G status | B rebias | M timing (M! reset) | U CPU + stack report");
  printBoth("Plant:   O<V> open-loop pulse | C<V> comp test | Y<ms> coast capture");
  printBoth("         I<pct> thrust step (wheel off) | Q<pct> yaw coupling (wheel on)");
  printBoth("Wheel:   E encoder diag (motor off) | V<volts> manual drive + encoder stream");
  printBoth("Fans:    S<n> select (0=all, 1-4) | L<pct> throttle | J<cmd> DSHOT command");
  printBoth("Dock:    TI[m] set pose | TG dock | TT go | TS stop (T + other letter lists all)");
  printBoth("Mode is IDLE. Send H0 or T<deg> to engage.");

  lastControlUs = micros();
}

// One 200 Hz control iteration, released by the TIM9 tick. dt comes from micros()
// so it stays correct after a blocking command such as B.
static void controlStep() {
  safety_kick();             // control task heartbeat for the watchdog
  pollSerial();

  unsigned long now = micros();
  float dt = (now - lastControlUs) * 1e-6f;
  lastControlUs = now;

  TIME_BLOCK(st_law, { controlUpdate(dt); });

  // I/Q thrust-step sequencer, timed off the capture clock so the phase boundaries
  // fall at known sample indices.
  if (transActive) {
    uint32_t el = millis() - capStartMs;
    if (el < transPreMs)                        applyFanStep(0.0f);
    else if (el < transPreMs + transHoldMs)     applyFanStep(transPct);
    else                                        applyFanStep(0.0f);
    if (el >= transTotalMs) { transActive = false; applyFanStep(0.0f); }
  }

  if (capturing) {
    if (capN < MAX_CAP) {
      cap_t[capN]      = now;
      cap_target[capN] = target;
      cap_theta[capN]  = theta;
      cap_wp[capN]     = omega_p;
      cap_ww[capN]     = omega_w;
      cap_alpha[capN]  = lastAlpha;
      cap_u[capN]      = lastU;
      cap_ax[capN]     = accel_x;
      cap_ay[capN]     = accel_y;
      cap_cur[capN]    = escCurRaw;
      capN++;
    }
    if (capN >= MAX_CAP || (millis() - capStartMs) >= capWindowMs) {
      capturing = false;
      // telemTask writes the dump at low priority while control keeps running.
      telem_run(dumpCapture);
      if (capTranslation && capKeepWheel) {
        // Q: keep heading control on. Cutting the wheel abruptly would kick the
        // platform, and it lets the wheel unwind before the next run.
        printBoth("yaw capture done. Heading control STILL ACTIVE so the wheel can "
                  "unwind. Watch omega_w in G, and X when finished.");
      } else if (capTranslation) {
        // Plant ID: stay IDLE so the wheel stays out of the next run.
        ctrlMode = CTRL_IDLE;
        controllerEnabled = false;
        motor.target = 0.0f;
        printBoth("plant-ID capture done. Wheel OFF, fans idle, ready for the next run.");
      } else {
        // After an open-loop or comp test go back to IDLE, not the closed loop.
        bool wasOpen = (ctrlMode == CTRL_OPEN || ctrlMode == CTRL_COMP);
        ctrlMode = wasOpen ? CTRL_IDLE : CTRL_HOLD;
        if (wasOpen)
          printBoth("open-loop capture done, back to IDLE. Check: positive volts "
                    "should give POSITIVE wheel and NEGATIVE theta.");
        else
          printBoth("capture done, now HOLDing. Adjust gains and send T again.");
      }
    }
  }

  // 10 Hz telemetry while holding, skipped while a dump is being written.
  if (ctrlMode == CTRL_HOLD && !capturing && !telem_busy()
      && (millis() - lastTelemMs) >= TELEM_PERIOD_MS) {
    lastTelemMs = millis();
    TIME_BLOCK(st_telem, {
      telem_print(String(degrees(theta), 2) + "," + String(degrees(target), 2) + ","
                + String(omega_p, 3) + "," + String(omega_w, 2) + ","
                + String(lastAlpha, 2) + "," + String(lastU, 3));
    });
  }
}

// TIM9 ISR callback. NVIC priority 5 is the configMAX_SYSCALL boundary, so the
// FromISR calls are legal. Notifies focTask every tick and the control task every
// CTRL_DIVISOR ticks.
static void focTickNotify(void) {
  static uint32_t n = 0;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(hFoc, &woken);
  if (++n >= CTRL_DIVISOR) { n = 0; vTaskNotifyGiveFromISR(hControl, &woken); }
  portYIELD_FROM_ISR(woken);
}

// Commutation only. Highest priority, so it preempts the blocking MPU read.
static void focTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TIME_BLOCK(st_foc,  { motor.loopFOC(); });
    TIME_BLOCK(st_move, { motor.move();    });
  }
}

static void controlTask(void*) {
  hwSetup();                 // creates focTask and starts TIM9
  telem_activate();          // telemTask owns serial output from here
  safety_kick();             // seed the heartbeat before arming the watchdog
  safety_arm();
  static uint32_t t_prev = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        // 200 Hz release from the TIM9 ISR
    uint32_t t = us_now();
    if (t_prev) stat_add(&st_period, t - t_prev);
    t_prev = t;
    controlStep();
  }
}

// Pre-scheduler setup: create the tasks and start FreeRTOS.
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  us_init();
  faults_reportLastBoot();

  telem_init(Serial, hc05Serial);

  // The watchdog stays idle until safety_arm(), because initFOC and the bias
  // measurement legitimately block the control task for seconds.
  i2c_init();
  safety_init(safetyWheelVel, WHEEL_SAT_LIMIT, safetyStop);
  safety_setPowerMonitor(safetyReadPower);
  // Measured idle is ~12.07 V, minimum under a T90 slew 11.65 V, peak 813 mA.
  // 10.0 V catches a collapsing battery; 2500 mA is ~3x the observed peak.
  safety_setPowerLimits(10.0f, 2500.0f);

  commands_init(Serial, hc05Serial, commsEmergencyStop);

  // The Pi link goes to pi_init, never commands_init: it is binary, and the line
  // parser would read a 0x58 payload byte as 'X'. pi_poll runs in commsTask's poll.
  pi_init(piSerial, PI_BAUD);
  commands_setAuxPoll(pi_poll);
  pi_setDeadHook(piLinkDead);

  est_init();
  trans_init();

  // fanTask arms the ESCs with ~1 s of zero frames before it accepts throttle.
  fans_init();

  // Every fault path kills the fans before the wheel. fans_stopAll() takes no lock
  // and calls no FreeRTOS API, as the fault path requires.
  faults_setHwKillHook(fans_stopAll);

  // safetyTask also watches for a stalled fanTask (silent loss of translation).
  safety_setFanMonitor(fans_frames, fans_armedAndLive);

  configASSERT(xTaskCreate(controlTask, "ctrl", 768, NULL, 3, &hControl) == pdPASS);

  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);   // only if the heap was too small
}

void loop() {}   // never runs under the scheduler
