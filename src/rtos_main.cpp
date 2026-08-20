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
#include "telemetry.h"       // Phase 3: telemTask is the SOLE serial writer
#include "safety.h"          // Phase 5: independent watchdog task
#include "i2c_bus.h"         // Phase 5.2: I2C mutex (control MPU + safety INA219)
#include "commands.h"        // Phase 6: commsTask owns serial RX
#include "fans.h"            // Translation 1.2: fanTask is the SOLE fan writer
#include "pi_link.h"         // Translation 3.1: USART6 pose link, sole owner of that port
#include "estimator.h"       // Translation 5.1: dock-relative pose estimator
#include "translation.h"     // Translation 6: x/y control via the fans
#include <Adafruit_INA219.h>

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

// Translation Phase 3: the Raspberry Pi pose link on USART6.
// ARGUMENT ORDER IS (RX, TX) -- same as hc05Serial above, where PC11 is
// USART3_RX and PC10 is USART3_TX. So PC7 (USART6_RX) first, PC6 (USART6_TX)
// second. Reversing these compiles cleanly and produces a dead link.
// 115200: payload is ~20 B at 10-30 Hz = ~600 B/s, ~6% utilised, and it matches
// the VCP and HC-05 so there is one baud to remember. It also keeps a 2 ms
// commsTask poll at ~23 bytes, well inside the core's 64-byte RX ring.
#define PI_BAUD         115200
HardwareSerial piSerial(PC7, PC6);

#define VOLTAGE_LIMIT   10.0f
// Ceiling on the O/C open-loop pulse. Plateau = 9.64*V - 1.23, so 5.5 V -> 51.8 rad/s,
// just under WHEEL_SAT_LIMIT (55). Do NOT raise without raising that too.
#define OPEN_PULSE_VMAX  5.5f
#define VOLTAGE_PSU     12.0f

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
SPIClass encoderSPI(PB15, PB14, PB13);
MagneticSensorMT6701SSI sensor(PIN_ENCODER_CS);
Adafruit_MPU6050 mpu;
Adafruit_INA219  ina219;          // Phase 5.2: read by safetyTask under the mutex
bool             inaPresent = false;

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
// RE-IDENTIFIED 2026-08-15, after the translation hardware was fitted (12 O-tests
// at +/-1, +/-2, +/-3 V). The plant MOVED, and the shift explains the dead passive
// desaturation exactly:
//   K' is NOT constant with voltage -- the plateau line has an intercept, because
//   the wheel's own Coulomb friction eats a fixed bite:
//        omega_plateau = 9.64*V - 1.23      (fits 1/2/3 V to ~0.5%)
//   so the INCREMENTAL gain is 9.64, not the 8.51 secant value used before.
//   tau' = 0.201 s from FREE DECAY (12 runs, 4.68-5.28) and 0.200 s from spin-up
//   t63 -- two independent methods agreeing to 0.5%. The decay is the better
//   measurement: with no applied voltage it depends on neither A_1 nor any
//   voltage-scaling convention.
//   A_1 = K'/tau' = 47.9      A_2 = 1/tau' = 4.97
// Why this mattered: with the OLD constants the residual pole was
//   5.633*compFrac - 4.97, i.e. +0.043 at compFrac = 0.89 -- POSITIVE, weakly
// unstable. That is why the wheel held speed instead of bleeding down, and it is
// the same 13%-gain-error mechanism that caused the first closed-loop runaway.
// With these constants the pole is simply A_2*(compFrac - 1).
static const float A_1          = 47.9f;    // rad/s^2 per V   [re-ID 2026-08-15]
static const float A_2          = 4.97f;    // 1/s             [re-ID 2026-08-15]
// RUNTIME TUNABLE (command A).  A_FRICTION = A_1 * V_break  -- note `a` CANCELS:
//   friction (platform) = a*A_1*V_break,  and A_FRICTION = friction/a = A_1*V_break
// Check against history: 40.56 * 0.55 = 22.3, exactly the original value. Which
// means the earlier raise 22.3 -> 28.3 "because a is 0.15 not 0.19" DOUBLE-COUNTED
// the correction: 4.24 was itself derived using a = 0.19. This constant is NOT in
// the "derived from `a`, therefore suspect" class that bit ALPHA_STALL_MAX twice.
//
// Re-measured 2026-08-15 (23 O-tests, 0.30-0.90 V, 3 sets): breakaway is between
// 0.65 and 0.85 V and varies by ~1.3x run to run -- intrinsic stiction scatter,
// not measurement error. A_1 * V_break therefore spans 31-41.
//   old: 40.56 * 0.55 = 22.3      new: 47.9 * ~0.75 = 36
// The rise is physical: four motors, an ESC, a Pi and a battery raise the normal
// force on the ball transfers, so friction torque rises while J_w is unchanged.
// Biased to the upper half DELIBERATELY -- the observed failure is "parks short and
// winds the wheel", which is the too-LOW signature; too high merely overshoots.
// Settle it with the small-error sweep (5 deg corrections x10), not large slews.
// SPLIT INTO TWO 2026-08-15. One constant was serving two DIFFERENT physical
// quantities, and no single value can satisfy both:
//    STUCK branch  needs to beat STATIC breakaway  (~55 in alpha units, measured)
//    MOVING branch needs to cancel KINETIC friction (lower -- static > kinetic)
// Set it high enough to break stiction and the moving branch OVER-cancels, turning
// friction into negative damping; set it low enough not to overshoot and it cannot
// break free. The 5-deg sweep showed exactly that split, 2 failures of each kind:
//    test05 moved  0.23 deg -> never broke free   (wanted MORE)
//    test08 moved 31.80 deg -> broke free and ran away on a 5 deg command (wanted LESS)
// Commands:  A<val> sets STATIC,  AM<val> sets MOVING.
float A_FRICTION  = 60.0f;    // rad/s^2 -- STUCK branch, must beat static breakaway
float A_MOVING    = 34.0f;    // rad/s^2 -- MOVING branch, cancels kinetic friction
// VISCOUS term on the MOVING branch, rad/s^2 per (rad/s) of platform rate. Command AV.
// Friction on this platform is NOT purely Coulomb -- it grows with speed:
//    at omega_p ~ 0     effective friction ~34 alpha-units  (A_MOVING, tuned on 5 deg
//                       corrections, where 48 caused 28 deg runaways from over-cancel)
//    at omega_p ~ 2.1   measured ~65 alpha-units, from a stalled T-90:
//                         wheel gives theta_ddot = -a*23  = -2.25 rad/s^2
//                         observed theta_ddot    = +4.13  (decelerating)
//                         => friction supplying   +6.4 rad/s^2 = ~65 in alpha units
// A single Coulomb constant cannot cancel both, which is exactly why small corrections
// are excellent and large slews grind to a halt part-way. CONTROL_README section 6
// already recorded that Coulomb+viscous fit better than Coulomb alone (R^2 0.951 vs
// 0.918); it was simplified away because the lighter platform did not need it.
// Slope from the two points above: (65-34)/2.1 ~ 15.
// DEFAULT 0 so this build is behaviourally identical until you set AV -- change one
// thing at a time.
float A_VISCOUS   = 0.0f;
static const float GYRO_SIGN   = -1.0f;    // aligns gyro with wheel convention

// ------------------- control loop timing -------------------
static stat_t st_foc, st_move, st_mpu, st_ina, st_law, st_telem, st_period;
static stat_t st_adc;   // Phase 2: ESC current-sense ADC read, added to the 200 Hz path

// ------------------- runtime-tunable gains -------------------
// Start conservative (3.0 s row) and work down using P/D over serial --
// no reflash needed between tuning steps.
// RESET TO THE 3.0 s ROW 2026-08-15 for the post-re-ID retune. The old 119.3/35.1
// were computed with a = 0.19; `a` is now measured at 0.105 (the platform got much
// heavier at the perimeter), so every gain in the table roughly doubles:
//        K_theta = wn^2 / a          K_omega = 2*zeta*wn / a        wn = 5.714/t_s
//   settle   wn      K_theta   K_omega        (a = 0.105, zeta = 0.7)
//    3.0 s   1.90       34        25    <-- start here, work DOWN
//    2.0 s   2.86       78        38
//    1.5 s   3.81      138        51
//    1.2 s   4.76      216        64    <-- wn ceiling is A_2 = 4.97
// Work DOWN the table with P/D over serial, not up: slow gains keep the feedforward
// active longer, which winds the wheel MORE, so faster gains are better on both
// accuracy and saturation.
// VALIDATED AND PERSISTED 2026-08-15 (the 1.2 s row, a = 0.105).
// These MUST stay in step with the deadzones: the deadband floor is
// (1-ffFrac)*A_moving/K_theta, so dropping K_theta without widening the deadzones
// puts the fine deadzone BELOW the floor, feedforward never switches off, and the
// wheel winds forever. That is exactly what happened when these two were left at
// the 3.0 s placeholder while everything else was persisted:
//     K = 216 -> floor 0.45 deg (0.8 deg fine deadzone legal)
//     K = 34.4 -> floor 2.83 deg (ILLEGAL) -> six T90 runs ended 21-63 deg short
//                 with the wheel wound to 38-52 rad/s and still spinning.
float K_theta   = 216.0f;
// K_omega 64 -> 52 on 2026-08-19, found by bisection on hardware. zeta ~= 0.54, NOT
// the 0.7 the table assumes -- because Coulomb friction already supplies heavy
// damping, so a zeta-0.7 design brakes twice and the platform stops short of target.
//   D64  over-damped: decelerated early, arrived with no momentum, stalled ~20 deg out
//   D30  under-damped: overshoot -19.8 deg, rang past and stalled at 7.6 deg
//   D52  overshoot -1.8 to -8.1, final error -0.03 to -0.77, 6/6 in one move
float K_omega   = 52.0f;
// 0.90 -> 0.95 on 2026-08-15. Raising ffFrac is the documented route to a tighter
// deadzone, because the floor is (1-ffFrac)*A_moving/K_theta: at 0.90 the floor is
// 1.35 deg (so a 1 deg fine deadzone is unreachable), at 0.95 it is 0.45 deg.
float ffFrac    = 0.95f;    // trim up until it overshoots, then back off
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
// C-SWEEP 2026-08-15, 10 runs at +/-2 V, cf 0.70-0.95. ALL decayed -- no instability
// anywhere in the range, unlike the previous campaign where cf = 1.0 was already
// unstable positive-going.
//        pole = 3.905*cf - 4.453      ->  measured neutral at cf = 1.140
//   cf   pole(+2V)  pole(-2V)
//   0.70   -1.712     -1.713
//   0.80   -1.373     -1.295
//   0.85   -1.178     -1.131
//   0.90   -0.951     -0.893   <-- chosen: -0.92 mean, vs the -0.84 design target
//   0.95   -0.760     -0.727
// Direction asymmetry is now 3-6%, against 0.972/0.892 (8%) last campaign -- most of
// that split was the wrong constants, not physics, so one value covers both.
// Neutral measured at 1.14 vs 1.00 predicted: the gap is the WHEEL's own Coulomb
// friction. The decay runs 18 -> 1 rad/s, so compensation voltage falls to ~0.1 V
// where the -1.23 intercept in (omega_plateau = 9.64*V - 1.23) dominates. Coulomb
// drag adds decay no proportional term can cancel, so neutral sits above 1.0 --
// margin the linear model does not know about.
// Chose 0.90 over the fitted 0.925 because 0.90 is DIRECTLY OBSERVED in both
// directions rather than extrapolated, and section 7's rule is to err low.
// Why 0.90 is safe when the old 0.89 was not -- the ratio it multiplies changed:
//        old: 0.89 * 5.35 / 45.5 = 0.1047 * omega_w   (> hold voltage -> wound up)
//        new: 0.90 * 4.97 / 47.9 = 0.0934 * omega_w   (11% less -> bleeds down)
float compFrac  = 0.90f;
float deadzone  = 0.0262f;  // rad (1.5 deg) COARSE -- used while the wheel is
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
// 2026-08-15: with ffFrac 0.95 and A_moving 34 the floor is 0.45 deg, so 0.8 deg
// is legitimate (nearly 2x the floor). Validated over n=8: mean |e| 0.97 deg,
// worst 1.47, 8/8 closed -- better than the 1.20 deg mean of the ORIGINAL lighter
// platform, on a platform whose friction has since risen ~60%.
float deadzoneFine = 0.0140f;  // rad (0.8 deg) FINE
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
// RAISED 55 -> 70 on 2026-08-15. This constant has now failed THREE times, always
// the same way: sized against a plant that then moved.
//    28  -> a*28 = 4.2 vs a 4.24 breakaway; coin flip, wound to 44.5, aborted
//    40  -> slews landed, terminal corrections stalled 2-9 deg short
//    55  -> worked, ON THE LIGHTER PLATFORM
//    55  -> after the fan subsystem, sits EXACTLY ON the new breakaway
// The A_FRICTION sweep proved the clamp was the binding constraint, not A_FRICTION:
//    A36 -> delivered alpha 49.6, moved 0.03 deg, 0/2 closed
//    A40 -> delivered      52.9, moved 0.10-0.32, 0/2
//    A44 -> delivered      55.0  (CLAMPED), 1/4
//    A48 -> delivered      55.0  (CLAMPED), 4/7   <- same authority as A44
// True breakaway is ~53-56 in alpha units. Raising A_FRICTION past ~44 does nothing
// while the clamp caps the sum.
#define ALPHA_STALL_MAX     70.0f    // rad/s^2, cap while platform is stationary
// STALL_WW/STALL_MS MUST move with the clamp. Stuck at alpha = 70 the wheel runs to
// 70/0.92 = 76 rad/s with tau = 1.09 s, so at the OLD 25 rad/s / 600 ms the detector
// would have fired at 1.00 s -- by which point omega_w is 49 and WHEEL_SAT has
// already aborted. The detector must win the race, not lose it:
//    catch at 20 rad/s (0.33 s) + 300 ms dwell -> fires 0.63 s, omega_w = 33. Safe.
#define STALL_WW            20.0f    // rad/s -- was 25
#define STALL_MS             300     // ms of continuous stall before backing off
// 2000 -> 4500 on 2026-08-15. The hold exists to bleed the wheel far enough that a
// retry has authority again, and 2 s was sized when the unwind pole was assumed to be
// ~0.9/s. MEASURED from 50 rad/s it is only 0.36/s: the extra decay the C-sweep saw
// came from the wheel's COULOMB friction, which is a fixed deceleration -- dominant
// at the 18 rad/s the sweep ran at, negligible at 50. So a 2 s hold left the retry
// starting at 23 rad/s, where delivered = 70 - 0.36*23 = 62 and the platform still
// would not break free reliably. 4.5 s bleeds 50 -> ~10 rad/s, restoring nearly the
// full clamp to the retry.
#define STALL_HOLD_MS       4500     // ms of alpha = 0 before retrying
#define WW_MAX_JUMP         15.0f    // rad/s per control cycle -- above this is a
                                     // velocity-estimate glitch, not real (physical
                                     // max ~2.3 rad/s/cycle). Rejected in sense.
// RAISED 45 -> 55 on 2026-08-15 (user decision). This is a CHOSEN safety number, not
// a hardware limit -- the wheel reaches ~96 rad/s at the 10 V ceiling. 45 was sized
// for the lighter platform; friction has since risen ~60%, so break-free costs more
// momentum and 45 left no headroom on the failure path. 55 buys margin while staying
// far below what the motor can actually do.
#define WHEEL_SAT_LIMIT     55.0f    // rad/s -- abort above this.
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

// ---- Phase 2 (translation plant ID) ----------------------------------------
// We identify the translational plant in ACCELERATION units, never force units:
//     x_ddot = A(throttle) - A_c*sign(v)
// Mass, thrust-in-grams and breakaway-in-grams all cancel and are never needed --
// the same reasoning that let CONTROL_README work in A_1/A_2/A_FRICTION and never
// pin down J_w or J_p (see its section 3). A(throttle) and A_c are both directly
// measurable from the accelerometer that has been sitting unused on the I2C bus.
#define ESC_CUR_PIN A4      // = PC1, the 4-in-1 ESC's current-sense pad
float accelBiasX = 0.0f, accelBiasY = 0.0f;  // m/s^2, measured with the gyro bias
float accel_x = 0.0f, accel_y = 0.0f;        // m/s^2, bias-removed, BODY frame
uint16_t escCurRaw = 0;                      // ESC current sense, raw ADC counts
// Declared up here rather than beside the rest of the thrust-step state because
// stopMotor() -- which is defined well above it -- must be able to abort a run.
static bool transActive = false;
// Marks a capture as a plant-ID run. The post-capture transition in controlStep()
// was written for heading steps and drops into CTRL_HOLD; for translation runs that
// engaged the wheel AND started the HOLD telemetry stream, which kept telem_busy()
// true so the NEXT capture was refused by the buffer-lifetime guard. Observed as
// "after the first test it doesn't capture the rest" (2026-08-14).
static bool capTranslation = false;
// Yaw-coupling runs must NOT have the wheel killed at capture end. Slamming
// motor.target to 0 drops the holding torque abruptly, and the wheel's own
// deceleration reacts on the platform -- a torque impulse landing exactly in the
// tail of the measurement. For Q we leave heading control running so the wheel
// unwinds gracefully and the platform stays stable afterwards.
static bool capKeepWheel = false;
// Which flavour of capture this was. Latched at capture START, because the dump runs
// later on telemTask by which time ctrlMode has already moved on.
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

// ------------------- capture buffer -------------------
// 1500 samples at 200 Hz = 7.5 s, ample for a step response.
// 7 fields x 4 bytes = 28 bytes/sample -> ~41 KB, comfortable in 128 KB SRAM.
#define MAX_CAP 1500
unsigned long cap_t[MAX_CAP];
float cap_target[MAX_CAP], cap_theta[MAX_CAP], cap_wp[MAX_CAP];
float cap_ww[MAX_CAP], cap_alpha[MAX_CAP], cap_u[MAX_CAP];
// Phase 2: three more columns on the SAME buffer rather than a second one.
// +18 KB of the ~58 KB free, and it keeps one buffer, one dump and one CSV format.
// Rotation captures get them too, which is free and occasionally informative.
float cap_ax[MAX_CAP], cap_ay[MAX_CAP];
uint16_t cap_cur[MAX_CAP];
int   capN = 0;
bool  capturing = false;
unsigned long capStartMs = 0;
// 6000 -> 7400. With STALL_HOLD_MS at 4500 a stall at t=1.5 s puts the retry at
// t=6.0 s -- exactly where the old window closed, so captures ended just before the
// interesting part. 7400 ms is the most MAX_CAP (1500 samples at 200 Hz = 7.5 s) allows.
#define CAPTURE_MS 7400
// Phase 2: capture window and label are per-run now, because a translation capture
// wants a different length and needs a different metadata line for the parser.
uint32_t    capWindowMs  = CAPTURE_MS;
const char* capModeName  = "heading_step";
float       capThrottle  = 0.0f;   // recorded in the metadata line
int         capFanSel    = 0;

int stepCount = 0;

// =====================================================================
static inline float wrapPi(float x) {
  while (x >  PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  return x;
}
static inline float signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

// Phase 3: queued to telemTask, which is the sole serial writer after activation.
// Direct-writes during boot (telemTask can't drain until the control loop yields).
void printBoth(const String &s) { telem_print(s); }

void stopMotor(const String &why) {
  // Phase 1.3: fans FIRST -- with the props unguarded (B7) they outrank the wheel.
  // This is the HARD kill (pins low as GPIO, latched), not "throttle to zero": a
  // soft stop would leave the ESC armed and still depending on the DMA path to keep
  // working. `R` re-arms, which costs ~1 s -- the right price for a stop.
  // Note the knock-on: handleCommand's default case calls this, so ANY unrecognised
  // serial input now kills the fans too. That is the existing "a confused operator
  // should not leave a flywheel spinning" rule, and it applies harder to props.
  fans_stopAll();
  transActive = false;          // Phase 2: X aborts a thrust step mid-sequence
  trans_enable(false);          // Phase 6: X aborts closed-loop translation
  controllerEnabled = false;
  ctrlMode = CTRL_IDLE;
  capturing = false;
  stallHold = false; stallStartMs = 0; parked = false; stallCount = 0;
  motor.target = 0.0f;               // focTask applies this within one tick (<=250 us)
  printBoth("!! STOP: " + why + "  (send R to re-enable)");
}

// Adapters handed to safetyTask, which is deliberately SimpleFOC-free.
// Wheel speed is read straight from the FOC layer (focTask keeps it fresh) so a
// wedged control task cannot freeze the value the watchdog depends on.
static float safetyWheelVel(void)          { return motor.shaft_velocity; }
static void  safetyStop(const char* why)   { stopMotor(String(why)); }

// Phase 6 X fast path. Runs on commsTask the instant an X is seen, ahead of the
// queue. An aligned 32-bit float store is atomic on Cortex-M4, so focTask picks it
// up within one tick (<=250 us) instead of waiting up to a full 5 ms control
// period. handleLine() still runs the full stop when the control task drains it.
// Phase 1.3: fans die here too. fans_stopAll() is lock-free register writes with no
// FreeRTOS API, so it is safe on this path, and it drives the pins low in
// MICROSECONDS -- comfortably better than the "within one frame period" the guide
// asks for. handleLine() still runs the full stopMotor() when control drains it.
static void commsEmergencyStop(void) { motor.target = 0.0f; fans_stopAll(); }
// NOTE: trans_enable(false) also runs from handleLine's X path via stopMotor().

// Translation 3.2: terminal action for a dead Pi link (PI_DEAD, 3 s with no
// valid pose). Runs ON commsTask, fires ONCE per loss episode.
//
// This announces rather than stops, by decision B21. Tier 2 has already zeroed
// the fans a full 2 s earlier, which is the action that matters while nothing
// consumes pose. Calling stopMotor() here instead would dump a spinning
// flywheel for a link nothing currently depends on -- and that dump is itself a
// ~42 rad/s^2 kick against a 4.24 breakaway. PHASE 6 REPOINTS THIS AT
// stopMotor() the moment a controller actually reads pose.
//
// printBoth is safe from here: the earliest this can fire is 3 s after a valid
// frame, long after telem_activate(), so the text is queued to telemTask rather
// than written from this task (invariant B15 / trap T18).
static void piLinkDead(void) {
  printBoth("!! PI LINK SILENT: no frames of ANY kind for 500 ms -- the Pi or "
            "the wire, not a lost tag. Fans killed. Wheel control UNCHANGED "
            "(B21 -- nothing consumes pose yet).");
}

// Called from safetyTask (prio 2). Takes the I2C mutex around the transaction ONLY.
// Longer timeout than the control path (5 ms): safety is not deadline-critical and
// would rather wait behind a 2.5 ms gyro read than miss the sample entirely.
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

// Platform must be stationary. Takes about a second.
void measureGyroBias() {
  const int N = 200;
  float sum = 0.0f;
  float sumAx = 0.0f, sumAy = 0.0f;   // Phase 2: accel bias, same at-rest window
  for (int i = 0; i < N; i++) {
    // Under the mutex: `B` re-runs this AFTER the watchdog is armed, so safetyTask
    // may be reading the INA219 on the same bus. Generous timeout -- this is a
    // calibration routine, not the control path, so waiting is fine; a skipped
    // sample would bias the average.
    sensors_event_t a, g, t;
    if (i2c_lock(20)) { mpu.getEvent(&a, &g, &t); i2c_unlock(); }
    sum   += g.gyro.z * 180.0f / PI;
    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    safety_kick();                   // ~1 s loop: keep the heartbeat alive
    // vTaskDelay, NOT delay(): Arduino delay() busy-spins on yield(), which only
    // yields to equal-or-higher priority -- it would starve safetyTask (2) and
    // telemTask (1) for the whole second. vTaskDelay actually BLOCKS.
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  gyroBias = sum / N;
  // Phase 2: the accelerometer measures SPECIFIC force, so any residual table tilt
  // shows up here as a constant offset (Step 2.3 Trap 1). Removing the at-rest
  // average removes the tilt component along with the sensor's own zero-g bias --
  // which is why this must be measured with the platform genuinely still and level.
  accelBiasX = sumAx / N;
  accelBiasY = sumAy / N;
  printBoth("[GYRO] bias = " + String(gyroBias, 4) + " dps");
  printBoth("[ACCEL] bias = " + String(accelBiasX, 4) + " / " + String(accelBiasY, 4)
            + " m/s^2  (includes any residual tilt)");
}

// The smallest heading error the controller can actually close, set by the
// friction that feedforward does not cancel. Any deadzone below this can never
// be reached: the controller pushes forever and the wheel winds up.
float deadbandFloorDeg() {
  // Residual friction while MOVING is what the PD must overcome, so this is
  // A_MOVING, not the static break-free value.
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
            + "  A_visc=" + String(A_VISCOUS, 1) + "/rad/s"
            + "  (break-free from rest needs about 28)");
  float floorDeg = deadbandFloorDeg();
  bool floorBad = (degrees(deadzoneFine) < floorDeg) || (degrees(deadzone) < floorDeg);
  printBoth("deadband floor=" + String(floorDeg, 2) + "deg"
            + (floorBad ? "  <-- BELOW A DEADZONE" : "  (both deadzones clear it)"));
  if (floorBad) {
    // Loud, because the quiet version of this warning cost a full session on
    // 2026-08-15: below the floor the feedforward never switches off, so the
    // controller cannot settle and the wheel winds indefinitely. It is not a
    // degraded mode, it is a broken one.
    printBoth("*********************************************************");
    printBoth("** DEADZONE BELOW FLOOR -- THE CONTROLLER CANNOT SETTLE **");
    printBoth("**   feedforward never switches off; the wheel winds.   **");
    printBoth("**   fix: raise N/W above " + String(floorDeg, 2)
              + " deg, or raise F, or raise P.   **");
    printBoth("*********************************************************");
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
  // Translation 1.2. The health signature is overruns=0 with frames climbing at
  // ~500/s: together they say every DSHOT frame drained well inside its 2 ms slot.
  // skips = benign (frame emitted inside the previous one's 60 us, release jitter);
  // overruns = REAL anomaly, should be 0. See fans.h for why they are separate.
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
  // Translation 3.2: the Pi pose link.
  // HEALTH SIGNATURE: frames climbing, crc/badlen/resync all 0, seqgaps 0.
  // rx climbing while frames does NOT means the two ends disagree on the format
  // or the CRC -- which looks exactly like a wiring fault, so check here first.
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
                + "  dockerr=" + String(sqrtf(e.mag_x * e.mag_x
                    + (e.mag_y - EST_D_DOCK) * (e.mag_y - EST_D_DOCK)), 3) + "m");
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
    // NDTR of 72 = nothing transferred; small (1-3) = frame ended short (desync).
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
  // The label becomes the FILENAME (capture_calibration.py sanitises it), so it has
  // to describe the run. A plant-ID label of "heading step to 0.0 deg" is not just
  // unhelpful, it collided across every run.
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
  out.print("mode="); out.print(capModeName);
  out.print(" target_deg="); out.print(degrees(target), 3);
  out.print(" K_theta=");  out.print(K_theta, 3);
  out.print(" K_omega=");  out.print(K_omega, 3);
  out.print(" ff=");       out.print(ffFrac, 3);
  out.print(" deadzone_deg="); out.print(degrees(deadzone), 3);
  out.print(" gyro_bias_dps="); out.print(gyroBias, 4);
  // Phase 2 fields. accel_bias is what was subtracted, so a parser can undo it;
  // fan_pct/fan_sel say what thrust was commanded during this run.
  out.print(" accel_bias_x="); out.print(accelBiasX, 4);
  out.print(" accel_bias_y="); out.print(accelBiasY, 4);
  out.print(" fan_pct=");      out.print(capThrottle, 1);
  out.print(" fan_sel=");      out.print(capFanSel);
  // The commanded open-loop voltage. Without it an O-test capture is unidentifiable
  // from its header -- 12 files all labelled "heading step to 0.0 deg" and the
  // voltage only recoverable by digging in the u column (2026-08-15).
  out.print(" open_volts=");   out.print(openVolts, 2);
  // A_FRICTION and compFrac are runtime-tunable, so a capture is not reproducible
  // without them -- the A-sweep runs had to be identified by counting files.
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

// Both of these are RUN ON telemTask via telem_run() -- telemTask owns the serial
// ports, so writing them directly in here is correct. Never call these from the
// control task once telem_activate() has run.
void dumpCapture() { dumpCaptureTo(Serial); dumpCaptureTo(hc05Serial); }

extern TaskHandle_t hControl;    // defined with the task bodies further down
extern TaskHandle_t hFoc;

// Step 7.1/7.2 — one system report: per-task CPU share + every stack high-water.
// RUN ON telemTask (telem_run) like the other bulk writers.
//
// vTaskGetRunTimeStats needs configGENERATE_RUN_TIME_STATS +
// configUSE_STATS_FORMATTING_FUNCTIONS + configUSE_TRACE_FACILITY (all 1 since
// Step 1.2). Its clock is rtRunTimeCounter() = TIM5->CNT at 1 MHz -- 1000x the
// tick, so plenty of resolution, but 32 bits at 1 MHz WRAPS EVERY ~71.6 MINUTES.
// Read these within an hour of boot, or reboot before measuring.
//
// High-water is the MINIMUM FREE words ever seen on that stack (FreeRTOS fills new
// stacks with 0xA5 and counts the surviving pattern). It is a floor observed over
// the paths actually exercised -- an untaken error path can still blow a stack
// later, so exercise the fault handlers before trusting these to resize.
void printSystemReport() {
  static char buf[768];          // ~40 B/task; static, single caller (telemTask)
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
  // Dump to BOTH channels -- if this only went to USB, sending M over HC-05
  // would land the reply on USB and look like nothing happened.
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
    stat_print(o, "ESC cur ADC",  &st_adc);   // Phase 2: added to the 200 Hz path
    stat_print(o, "ctrl period",  &st_period);   // control-release interval (~5000 us)
    uint32_t fmn, fmx; focTick_jitter(&fmn, &fmx);
    o.print("FOC tick dt (us): min="); o.print(fmn); o.print(" max="); o.println(fmx);
    o.println("-------------------");
  }
}

// =====================================================================
// THE CONTROL LAW
// =====================================================================
void controlUpdate(float dt) {
  // ---- sense ----
  // Gyro read under the I2C mutex (safetyTask reads the INA219 on the same bus).
  // SHORT TIMEOUT, never portMAX_DELAY: the safety read is ~1 ms and 2 ms of a 5 ms
  // budget is the most this loop can afford to wait. On failure we DEGRADE -- reuse
  // the previous gyro sample and count it -- rather than block. A control task
  // waiting forever on a wedged bus with a spinning flywheel is how runaways happen.
  static float gyro_dps_prev = 0.0f;
  float gyro_dps;
  sensors_event_t a, g, t;
  if (i2c_lock(2)) {
    TIME_BLOCK(st_mpu, { mpu.getEvent(&a, &g, &t); });
    i2c_unlock();                                    // release IMMEDIATELY after
    gyro_dps = (g.gyro.z * 180.0f / PI) - gyroBias;
    gyro_dps_prev = gyro_dps;
    // Phase 2: the accel arrives in the SAME transaction we already pay 2.4 ms for,
    // so reading it is free. Body frame, bias removed. On an I2C failure we simply
    // hold the previous sample, like the gyro.
    accel_x = a.acceleration.x - accelBiasX;
    accel_y = a.acceleration.y - accelBiasY;
  } else {
    gyro_dps = gyro_dps_prev;                        // degrade: hold last good
  }
  omega_p = GYRO_SIGN * gyro_dps * PI / 180.0f;     // rad/s, model convention

  // Phase 2: ESC current sense (4-in-1 total, not per channel). Raw counts -- the
  // mV/A scaling is unknown without the ESC datasheet, and deliberately not guessed:
  // every use here is a RATIO (channel matching, curve shape), where it cancels.
  //
  // DECIMATED TO 20 Hz. Measured cost of analogRead() here was 100 us mean / 102 MAX,
  // which is NOT conversion time (12-bit is 1-3 us) -- STM32duino re-initialises the
  // ADC peripheral on every call. At 200 Hz that was 2% of the CPU for a signal whose
  // bandwidth is fan spin-up, tens of ms. Every 10th cycle costs 0.2% and loses
  // nothing; the held value is what the capture logs in between.
  // If this ever needs to be fast, configure ADC1 once and read DR directly (~1 us).
  static uint8_t adcDiv = 0;
  if (++adcDiv >= 10) {
    adcDiv = 0;
    TIME_BLOCK(st_adc, { escCurRaw = (uint16_t)analogRead(ESC_CUR_PIN); });
  }

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

  // ---- Translation 5.1: dock-relative pose estimator.
  // Runs unconditionally, even when the controller is idle -- it is a SENSOR,
  // and having a converged estimate the moment control is enabled is worth
  // more than the handful of microseconds it costs. Predict every cycle;
  // correct only when pi_link has a frame we have not already used (T10).
  est_predict(dt, theta);
  {
    PiPose pp;
    if (pi_getPose(&pp)) est_correct(&pp, theta, omega_p);
  }

  // Translation 6: x/y control. Runs BEFORE the enable/idle early-return below,
  // because translation and rotation are independent -- the wheel being idle
  // says nothing about whether the fans should be holding station.
  {
    EstState es;
    if (est_get(&es)) trans_update(&es, pi_state() == PI_FRESH);
    else              trans_update(NULL, false);
  }

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
    telem_run(dumpCapture);   // DIAG: trajectory up to the abort (ramp vs spike)
    return;
  }

  // ---- 1. error ----
  float e = wrapPi(target - theta);
  // Two-stage tolerance: coarse while the wheel still holds momentum, fine once
  // it has bled down and full torque is available again.
  // TWO-STAGE DEADZONE. Bare comparison, deliberately -- see the note below.
  //
  // KNOWN, MEASURED, AND LEFT ALONE: this chatters. Crossing FINE_WW toggles the
  // tolerance, which toggles the controller, which drives omega_w back across it:
  //     w > 5 -> dz = coarse -> e inside  -> alpha = 0 -> wheel bleeds
  //     w < 5 -> dz = fine   -> e outside -> push      -> wheel speeds up -> repeat
  // Observed 2026-08-15: five of eight terminal corrections sat pinned at
  // omega_w = 5.0-5.4 with the error frozen BETWEEN the two tolerances (~1.0-1.3 deg).
  //
  // A LATCHING VERSION WAS TRIED AND WAS WORSE. Holding the fine tolerance once the
  // wheel had unwound made the controller persist against stiction it could not
  // always beat: mean error 0.97 -> 1.25 deg, worst 1.47 -> 3.10, and wheel peaks
  // 6-17 -> 8-36 rad/s with two runs parked at 24-30 rad/s. The chatter is a SAFETY
  // VALVE -- the controller gives up and the wheel bleeds -- and removing it costs
  // more than the ~0.3 deg of extra precision it buys. Do not "fix" this again
  // without n >= 8 evidence that the latched version actually wins.
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
      // MOVING: cancel kinetic friction, Coulomb + viscous. A_VISCOUS = 0 gives the
      // pure-Coulomb behaviour this was before.
      ff = -(A_MOVING + A_VISCOUS * fabsf(omega_p)) * signf(omega_p);
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
// Translation 1.4: which fan the L command drives. 0 = all four, 1-4 = one channel.
static int fanSel = 0;

// Apply throttle to whatever S selected. Called every control cycle during a
// translation run, which also pets the fan dead-man (B12) exactly the way a closed
// loop will in Phase 6 -- so the run cannot be cut short by the 10 s timeout.
static void applyFanStep(float pct) {
  if (fanSel == 0) fans_setAll(pct, pct, pct, pct);
  else             fans_setThrottle(fanSel, pct);
}

// One place to start any capture, so the window and the metadata line can never
// disagree with what is actually being recorded.
static void startCapture(const char* mode, uint32_t windowMs, float pct, int sel,
                         bool translation) {
  // stepCount MUST advance for every capture, not just O/T/C. capture_calibration.py
  // names each file from the "test N/M: label" marker and opens it with "w", so two
  // captures emitting the same marker silently overwrite each other -- which is what
  // made every plant-ID run land in test00_heading_step_to_0_0_deg.csv (2026-08-14).
  stepCount++;
  capN = 0;
  capModeName = mode;
  capWindowMs = windowMs;
  capThrottle = pct;
  capFanSel   = sel;
  capTranslation = translation;
  capKeepWheel   = false;      // Q sets this true immediately after calling us
  capOpenLoop    = false;      // O / C set theirs the same way
  capCompTest    = false;
  capStartMs  = millis();
  capturing   = true;
}

// ---- Phase 2 translation plant ID: the automatic thrust step -----------------
// One command produces BOTH quantities the model needs, from one capture:
//   thrust phase : x_ddot = A(throttle) - A_c   (platform accelerating)
//   coast  phase : x_ddot = -A_c                (fans off, still moving)
// so A_c comes out of the tail of the very run that measures A(throttle), and the
// two are measured under identical surface/battery conditions instead of in
// separate sessions.
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

  // Phase 3 buffer-lifetime guard: a capture writes cap_*, and a queued dump
  // READS cap_* from telemTask. Starting a new capture mid-dump would rewrite the
  // buffer underneath it. Refuse rather than corrupt -- the dump is a few seconds.
  // 'Y' and 'I' are captures too and MUST be in this guard -- omitting them let a
  // second Y reset capN while the first dump was still streaming out of cap_*,
  // which silently lost the run (observed 2026-08-13).
  if ((c == 'T' || c == 'O' || c == 'C' || c == 'Y' || c == 'I') && telem_busy()) {
    printBoth("busy: previous capture still dumping -- wait, then resend");
    return;
  }

  switch (c) {
    case 'O':
      // Open-loop sign check. Positive voltage should drive the wheel
      // positive, which by theta'' = -a*alpha must move theta NEGATIVE.
      // If theta goes positive instead, GYRO_SIGN is wrong -- flip it in
      // the source and reflash BEFORE closing the loop.
      // RAISED from +/-3 V on 2026-08-15. The 3 V clamp is why the wheel plant was
      // only ever identified to 28 rad/s (plateau = 9.64*V - 1.23): the tool could
      // not reach higher. But a 90 deg slew takes the wheel to 50, so every constant
      // was being extrapolated to ~2x the speed it was measured at -- and the
      // effective K' at 48 rad/s backs out at 7.46, not 9.64, which is why large
      // slews stall mid-way while small corrections are fine.
      // 5.5 V is the ceiling: plateau 51.8 rad/s, just under WHEEL_SAT_LIMIT 55.
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
      // TX/TY set the dock-frame target for the MAGNET, TT enables, TS stops.
      // 'T' followed by a LETTER is a translation command; followed by a digit
      // it is the original heading step, unchanged.
      if (s.length() > 1 && isAlpha(s.charAt(1))) {
        char sub = toupper(s.charAt(1));
        float v2 = (s.length() > 2) ? s.substring(2).toFloat() : 0.0f;
        float tx, ty;
        trans_getTarget(&tx, &ty);
        if (sub == 'X') { trans_setTarget(v2, ty); }
        else if (sub == 'Y') { trans_setTarget(tx, v2); }
        else if (sub == 'T') {
          if (pi_state() != PI_FRESH) {
            printBoth("REFUSED: pose is not FRESH. Translation will not start "
                      "blind -- check the Pi is sending and the tags are seen.");
            break;
          }
          trans_enable(true);
          trans_getTarget(&tx, &ty);
          printBoth("TRANSLATE -> magnet target (" + String(tx, 3) + ", "
                    + String(ty, 3) + ")  PROPS ARE LIVE");
          break;
        }
        else if (sub == 'S') { trans_enable(false); printBoth("translation STOP"); break; }
        else { printBoth("TX<m> TY<m> set target, TT go, TS stop"); break; }
        trans_getTarget(&tx, &ty);
        printBoth("target (" + String(tx, 3) + ", " + String(ty, 3) + ")");
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
      // Compensation-only test: spin up at V, then alpha = 0. Watch omega_w.
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
      // A<val> = STATIC (break-free). AM<val> = MOVING (kinetic cancellation).
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
        stat_reset(&st_adc);
        stat_reset(&st_foc);   stat_reset(&st_move);  stat_reset(&st_mpu);
        stat_reset(&st_ina);   stat_reset(&st_law);   stat_reset(&st_telem);
        stat_reset(&st_period);
        printBoth("timing stats reset");
      } else {
        // Run the whole block ON telemTask -- it owns the ports, so stat_print
        // may write them directly there. Doing this from the control task would
        // be a second writer (exactly the Step 3.1 bug).
        telem_run(printTimingStats);
      }
      break;
    case 'U':
      // Step 7.1/7.2 system report. Runs on telemTask (sole writer).
      telem_run(printSystemReport);
      break;
    case 'X': stopMotor("operator"); break;
    case 'J': {
      // DSHOT special command to the S-selected channel. 20=dir NORMAL,
      // 21=dir REVERSED, 7/8=dir 1/2, 12=SAVE (required to persist any of them).
      int cmd = (int)v;
      if (fanSel < 1 || fanSel > 4) {
        printBoth("J needs a single channel selected first -- send S1..S4 (not S0)");
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
    // ---- Phase 2: translation plant identification ---------------------------
    case 'Y': {
      // Passive capture -- fans untouched. This is the HAND-PUSH coast-down, and
      // it is the cleanest measurement in the whole phase: no props spinning, no
      // rig, no force units. Push the platform during the window; Coulomb friction
      // gives a CONSTANT deceleration, so A_c falls out of a straight-line fit.
      uint32_t ms = (v > 0.0f) ? (uint32_t)v : 5000;
      if (ms > 7000) ms = 7000;              // MAX_CAP 1500 @200 Hz = 7.5 s
      // Wheel OFF for plant ID -- its reaction torque yaws the platform, and yaw
      // contaminates the accelerometer twice over (body-frame rotation, plus
      // lever-arm terms because the IMU is not at the CoM). Deliberately does NOT
      // touch the fans, so this needs no X/R dance to set up.
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      startCapture("coast_down", ms, 0.0f, 0, true);
      printBoth("COAST capture " + String(ms) + " ms -- push the platform now.");
      printBoth("  (fans untouched; wheel is whatever mode it was in)");
      break;
    }
    case 'Q': {
      // Step 2.4 -- yaw coupling. Same thrust step as I, but with heading control
      // ENGAGED. A fan whose thrust line misses the CoM applies a constant yaw
      // torque; to hold heading against it the wheel must keep accelerating, so
      // d(omega_w)/dt during the thrust phase IS the disturbance torque, in the
      // wheel-acceleration units the rotation controller already works in.
      if (!fans_armed() || fans_killed()) {
        printBoth("fans not ready -- send R to re-arm");
        break;
      }
      transPct    = v;
      transActive = true;
      // Hold the CURRENT heading, so the wheel starts from zero error and every
      // rad/s it banks is attributable to the fan rather than to a slew.
      target = theta;
      parked = false; stallCount = 0; stallHold = false; stallStartMs = 0;
      ctrlMode = CTRL_HOLD; controllerEnabled = true;
      // Longer window than a thrust step: the 4 s one ended mid-settle, so we never
      // saw where the wheel finished. MAX_CAP allows 7.5 s at 200 Hz.
      startCapture("yaw_coupling", YAW_TOTAL_MS, v, fanSel, true);
      capKeepWheel = true;
      printBoth("YAW COUPLING " + String(v, 1) + "% on "
                + String(fanSel == 0 ? "ALL" : String(fanSel))
                + " -- heading control ACTIVE, wheel will spin up to fight the torque");
      printBoth("  watch omega_w: if it walks toward " + String(WHEEL_SAT_LIMIT, 0)
                + " rad/s the thrust line needs MECHANICAL correction. X aborts.");
      break;
    }
    case 'I': {
      // Automatic thrust step on the S-selected fan(s): quiet, thrust, coast.
      if (!fans_armed() || fans_killed()) {
        printBoth("fans not ready (armed=" + String(fans_armed())
                  + " killed=" + String(fans_killed()) + ") -- R to re-arm");
        break;
      }
      // Wheel OFF -- same reasoning as Y. Fans untouched (they must stay armed).
      motor.target = 0.0f; controllerEnabled = false; ctrlMode = CTRL_IDLE;
      transPct    = v;
      transActive = true;
      startCapture("thrust_step", transTotalMs, v, fanSel, true);
      printBoth("THRUST STEP " + String(v, 1) + "% on "
                + String(fanSel == 0 ? "ALL" : String(fanSel))
                + " -- " + String(transPreMs) + "ms quiet / " + String(transHoldMs)
                + "ms thrust / " + String(transTotalMs - transPreMs - transHoldMs)
                + "ms coast");
      printBoth("  KEEP CLEAR. X aborts.");
      break;
    }
    // ---- Translation 1.4: manual fan commands --------------------------------
    // Raw throttle only. A true thrust-VECTOR command is deferred to Phase 6.3:
    // converting force to throttle needs the square-law inversion
    // throttle = sqrt(F/F_max), and F_max comes from the Phase 2.1 thrust curve,
    // which does not exist yet. Raw throttle is what Phase 2 needs to MEASURE it.
    case 'S': {
      int n = (int)v;
      if (n < 0 || n > 4) { printBoth("fan select must be 0 (all) or 1-4"); break; }
      fanSel = n;
      printBoth("fan select: " + String(n == 0 ? "ALL" : String(n)));
      break;
    }
    case 'L': {
      if (fans_killed()) {
        printBoth("fans are KILLED -- send R to re-arm first");
        break;
      }
      if (!fans_armed()) {
        printBoth("fans still arming (~1 s from boot) -- wait for 'fans: armed'");
        break;
      }
      if (fanSel == 0) fans_setAll(v, v, v, v);      // one frame, all four
      else             fans_setThrottle(fanSel, v);
      // Report what was ACTUALLY applied, not what was asked for -- the clamp to
      // FAN_THROTTLE_MAX lives in the setter, so echoing v would hide it.
      printBoth("fans -> " + String(fans_pct(1), 1) + " / " + String(fans_pct(2), 1)
                + " / " + String(fans_pct(3), 1) + " / " + String(fans_pct(4), 1)
                + " %   (cap " + String(FAN_THROTTLE_MAX, 0) + "%, auto-zero after "
                + String(FAN_CMD_TIMEOUT_MS / 1000) + " s idle)");
      break;
    }
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
        telem_print(String(i) + "," + String(ang, 4) + ","
                  + String(degrees(turn), 2) + "," + String(vel, 3));
        safety_kick();   // 3 s loop: keep the heartbeat alive
        // vTaskDelay, NOT delay(): delay() busy-spins and would starve telemTask,
        // so this command's own output would never get written out.
        vTaskDelay(pdMS_TO_TICKS(100));
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
      // Watch the RX byte COUNTER, don't read the port: commsTask is the sole
      // reader now (see commands.h), and a second reader would race it on the RX
      // ring buffer -- commsTask would swallow the stop key and this would never exit.
      unsigned long t0 = millis(), lastP = 0;
      uint32_t rxAtStart = commands_rxBytes();
      while (commands_rxBytes() == rxAtStart) {
        motor.target = volts;                          // focTask applies it each tick
        if (millis() - lastP >= 20) {                  // ~50 Hz, unlimited
          lastP = millis();
          float ang  = motor.shaft_angle;
          float vel  = motor.shaft_velocity;
          float turn = ang - floorf(ang / (2.0f * PI)) * (2.0f * PI);
          telem_print(String(millis() - t0) + "," + String(ang, 4) + ","
                    + String(degrees(turn), 2) + "," + String(vel, 3) + ","
                    + String(volts, 2));
        }
        safety_kick();   // runs until a keypress: keep the heartbeat alive
        vTaskDelay(1);   // yield so telemTask can drain (this loop never blocks otherwise)
      }
      motor.target = 0.0f;   // focTask applies within one tick
      // No RX drain here: commsTask owns the port and has already consumed the
      // stop key into its queue, where it will be handled as a normal command.
      printBoth("MANUAL DRIVE stopped (target 0).");
      break;
    }
    case 'R':
      controllerEnabled = true; ctrlMode = CTRL_IDLE;
      // Phase 1.3: undo the hard kill. Re-runs the hardware init (AF mode + MOE) and
      // makes fanTask repeat its ~1 s zero ramp before it will accept throttle again.
      // No-op if the fans were not killed. Watch for "fans: armed" before commanding.
      fans_rearm();
      printBoth("controller re-enabled (IDLE); fans re-arming (~1 s)");
      break;
    default:
      // Unrecognised input stops the motor rather than being ignored.
      stopMotor("unrecognised command '" + s + "'");
      break;
  }
}

// Phase 6: RX and line assembly moved to commsTask (prio 2). The control task now
// only DRAINS complete lines and executes handleLine() -- unchanged -- at a known
// point in its cycle, so command writes can never land mid-controlUpdate().
void pollSerial() {
  String line;
  while (commands_next(line)) handleLine(line);
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

  // INA219 — the second I2C user (safetyTask reads it under the mutex).
  // begin() returning true only confirms an I2C ack, NOT that the calibration
  // registers are set; without the explicit call the readings come back `inf`
  // (CONTROL_README §16). Absence is NOT fatal: monitoring is simply skipped, so
  // a missing sensor can never cause a spurious safe-stop.
  inaPresent = ina219.begin();
  if (inaPresent) {
    ina219.setCalibration_32V_2A();
    printBoth("[INA219] OK (power monitoring active)");
  } else {
    printBoth("[INA219] NOT FOUND -- power monitoring disabled (not fatal)");
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

  // Phase 4: hand commutation to focTask, clocked by the TIM9 tick. Created here
  // (after initFOC, motor ready) not in setup(). focTask blocks on its notify at
  // once; nothing commutates until focTick_init() starts TIM9 on the next line.
  configASSERT(xTaskCreate(focTask, "foc", 256, NULL, 4, &hFoc) == pdPASS);
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
  printBoth("          U system report (per-task CPU % + stack high-water)");
  printBoth("  FANS:   S<n> select fan (0=all, 1-4) | L<pct> throttle the selection");
  printBoth("          L0 = fans off (stays armed).  X = hard kill, R = re-arm.");
  printBoth("          PROPS OFF unless the test needs thrust -- guards are skipped.");
  printBoth("  ESC:    J<cmd> DSHOT command to the S-selected fan (20/21=dir, 12=SAVE)");
  printBoth("          reverse fan4 permanently:  S4 -> J21 -> J12 -> power-cycle");
  printBoth("  PLANT:  Y<ms> coast capture (hand push, fans idle) | I<pct> thrust step");
  printBoth("          I uses the S selection: quiet/thrust/coast, then auto-dumps.");
  printBoth("          Q<pct> yaw-coupling step -- same, but heading control ACTIVE");
  printBoth("          (I = wheel OFF for plant ID.  Q = wheel ON to measure yaw.)");
  printBoth("Start at K_theta=19.1 K_omega=14.0, then work down the gain table.");
  printBoth("Mode is IDLE -- send H0 or T<deg> to engage.");

  lastControlUs = micros();
}

// controlStep(): one 200 Hz control iteration. Body is the old superLoopBody
// verbatim MINUS loopFOC/move (now in focTask) and MINUS the micros() rate gate
// (the TIM9 notification IS the 200 Hz clock now). dt still comes from micros()
// so it stays correct across a blocking command (e.g. B rebias), as before.
static void controlStep() {
  safety_kick();             // "control task is alive" -- the watchdog's signal
  pollSerial();

  unsigned long now = micros();
  float dt = (now - lastControlUs) * 1e-6f;
  lastControlUs = now;

  TIME_BLOCK(st_law, { controlUpdate(dt); });

  // ---- Phase 2 thrust-step sequencer ---------------------------------------
  // Driven off the capture clock so the phase boundaries land at known sample
  // indices in the CSV -- the analysis can then slice thrust vs coast without
  // having to detect the transition from noisy data.
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
      // PHASE 3 PAYOFF: hand the frozen buffer to telemTask instead of writing it
      // here. The control task returns to its 200 Hz cadence immediately; the ~11 s
      // of serial output happens at priority 1, in the gaps between control
      // releases. The capture buffer stays untouched until the dump finishes --
      // the T/O/C commands refuse to start a new capture while telem_busy().
      telem_run(dumpCapture);
      if (capTranslation && capKeepWheel) {
        // Yaw coupling (Q): leave heading control RUNNING. Killing the wheel here
        // dumps its holding torque in one step and the deceleration reacts on the
        // platform -- a torque impulse in the tail of the very measurement we are
        // taking. Leaving it engaged also lets the wheel unwind gracefully, which
        // matters because consecutive runs stack momentum.
        printBoth("yaw capture done. Heading control STILL ACTIVE so the wheel can "
                  "unwind -- watch omega_w in G, and X when you are finished.");
      } else if (capTranslation) {
        // Phase 2 plant ID: stay IDLE. Falling into HOLD here engaged the wheel
        // (contaminating the accelerometer with reaction torque) and started the
        // HOLD telemetry stream, which held telem_busy() and made the buffer
        // guard refuse the NEXT run. Wheel stays off; fans stay armed at zero.
        ctrlMode = CTRL_IDLE;
        controllerEnabled = false;
        motor.target = 0.0f;
        printBoth("plant-ID capture done. Wheel OFF, fans idle -- ready for the next run.");
      } else {
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
  }

  // Slow telemetry while holding, so you can nudge the platform by hand and
  // watch it recover without needing a full capture.
  // ...but not while a capture dump is in flight: telemTask is busy for ~11 s
  // writing 1201 rows, so these 10 Hz rows would just overflow the queue and be
  // dropped (97 of them, observed). Skipping them is honest -- the operator is
  // reading the dump, not the live stream.
  if (ctrlMode == CTRL_HOLD && !capturing && !telem_busy()
      && (millis() - lastTelemMs) >= TELEM_PERIOD_MS) {
    lastTelemMs = millis();
    TIME_BLOCK(st_telem, {
      // Queued, not written here -- the control task must never touch the ports.
      telem_print(String(degrees(theta), 2) + "," + String(degrees(target), 2) + ","
                + String(omega_p, 3) + "," + String(omega_w, 2) + ","
                + String(lastAlpha, 2) + "," + String(lastU, 3));
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
  telem_activate();          // boot prints done -> telemTask owns serial from here
  safety_kick();             // seed the heartbeat before arming the watchdog
  safety_arm();              // hwSetup done -> watchdog live from here
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

  // Phase 3: telemTask (prio 1) before the control task. It only gets CPU once
  // control blocks, and until telem_activate() output goes straight out.
  telem_init(Serial, hc05Serial);

  // Phase 5: independent watchdog (prio 2). Idles until safety_arm() at the end
  // of hwSetup -- initFOC and the gyro-bias measurement legitimately block the
  // control task for seconds, which would otherwise read as a dead heartbeat.
  i2c_init();                // Phase 5.2: bus mutex, before any task can use Wire
  safety_init(safetyWheelVel, WHEEL_SAT_LIMIT, safetyStop);
  safety_setPowerMonitor(safetyReadPower);
  // Thresholds from MEASURED values (2026-08-08), not guessed:
  //   idle 12.07-12.08 V; under a T90 slew min 11.65 V, peak |I| 813 mA.
  // 10.0 V is ~14% below nominal and well under the 11.65 V working sag -- it
  // catches a genuinely collapsing battery, not normal load. 2500 mA is ~3x the
  // observed peak, clear of an aggressive slew but far below a hard short.
  // Both debounced 2 checks. Tighten once T180 / stall data exists.
  safety_setPowerLimits(10.0f, 2500.0f);

  // Phase 6: commsTask (prio 2) becomes the SOLE serial READER. Created before the
  // control task; harmless if bytes arrive early, they just queue.
  commands_init(Serial, hc05Serial, commsEmergencyStop);

  // Translation 3.1: the Pi pose link. piSerial is passed to pi_init, NEVER to
  // commands_init -- it carries a binary stream, and commsTask's line assembly
  // would read a 0x58 payload byte as 'X' and stop the wheel. See pi_link.h /
  // decision B18. pi_poll shares commsTask's existing 2 ms poll.
  pi_init(piSerial, PI_BAUD);
  commands_setAuxPoll(pi_poll);
  // Translation 3.2: terminal action for a dead link. DELIBERATELY NOT
  // stopMotor() in Phase 3 -- decision B21. Nothing consumes pose until Phase 6,
  // so a lost link endangers nothing today, while dumping a spinning flywheel
  // DOES spin the platform at ~42 rad/s^2 against a 4.24 breakaway. Tiers 1 and
  // 2 (invalidate, fans off) fire for real; this one announces itself so the
  // ladder is fully exercised, and Phase 6 repoints it at the real stop.
  pi_setDeadHook(piLinkDead);

  est_init();
  trans_init();

  // Phase 1.2 (translation): fanTask (prio 2) becomes the SOLE fan writer. Touches
  // only TIM1 + DMA2_S5 + PA8..PA11 -- nothing SimpleFOC, the encoder or I2C owns.
  // All four channels start at DSHOT 0 and the task spends its first ~1 s sending
  // zero frames to arm the ESC before it will accept any throttle at all.
  fans_init();

  // Phase 1.3: fans into EVERY fault path, ahead of the wheel. faults_halt() calls
  // this after masking interrupts and before pulling the DRV8313 enable low, so
  // assert / stack-overflow / malloc-fail / heartbeat / wheel-sat all kill the props
  // first. fans_stopAll() takes no lock and calls no FreeRTOS API, which is exactly
  // the contract that path requires.
  faults_setHwKillHook(fans_stopAll);

  // Phase 1.3: safetyTask watches fanTask for a stall. Not a hazard (ESCs disarm when
  // frames stop) but it IS a silent loss of translation authority, so it gets caught.
  safety_setFanMonitor(fans_frames, fans_armedAndLive);

  configASSERT(xTaskCreate(controlTask, "ctrl", 768, NULL, 3, &hControl) == pdPASS);
  vTaskSetThreadLocalStoragePointer(hControl, 0, (void*)(uintptr_t)TRACE_ID_CTRL);

  vTaskStartScheduler();
  faults_safeStop(FAULT_SCHEDULER_RETURNED);   // only if the heap was too small
}

void loop() {}   // never runs under the scheduler