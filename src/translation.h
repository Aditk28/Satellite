#pragma once
#include <Arduino.h>
#include "estimator.h"

/*
  Translation Phase 6 -- x/y control.

  Chain:  EstState (dock frame) -> PD + Coulomb feedforward -> a_desired (dock)
          -> rotate by psi -> a_desired (body) -> allocate to opposing fan pairs
          -> square-law inversion -> throttle percent

  MASS NEVER APPEARS. The guide's Step 6.1 writes K_p = omega_n^2 * m, but B14
  identified this plant in ACCELERATION units precisely so that mass and force
  cancel: x_ddot = A(throttle) - A_c*sign(v). The control output IS an
  acceleration, so the gains are just omega_n^2 and 2*zeta*omega_n -- exactly as
  on the wheel axis, where the same reasoning let CONTROL_README work in
  A_1/A_2/A_FRICTION and never pin down J_w.

  ---------------------------------------------------------------------------
  FAN GEOMETRY -- measured 2026-08-20, and the single most dangerous constant
  in this file.

  Angles are the direction each fan PUSHES THE PLATFORM, measured from the
  camera axis, CCW POSITIVE (toward the camera's left), in the BODY frame:

        fan 1  -130 deg          fan 4   +50 deg     <- opposing pair
        fan 2   -40 deg          fan 3  +140 deg     <- opposing pair

  Two orthogonal axes, rotated 50 deg off the camera axis.

  ⚠️ THIS TABLE WAS WRONG TWICE, and the second time was a wrong FIX. The
  history is worth keeping because both errors look identical from the outside
  -- the platform moves confidently in the wrong direction:

    as first written    the platform went 166 deg from commanded
    "corrected" +180    the platform went  75 deg from commanded
    correct: -90 deg from the ORIGINAL remembered table

  HOW THE WRONG FIX HAPPENED, because the method matters more than the number.
  Four single-fan `I50` thrust steps were read off the body-frame accelerometer
  -- a good instrument, immune to psi, to the magnet lever arm, and to the free
  yaw that ruins a vision-displacement estimate. That gave the four fans'
  directions RELATIVE to each other correctly. It could not give their
  relationship to the CAMERA AXIS, so that anchor was taken from two hand-shove
  `I0` captures, which appeared to put body-forward at +76 deg in the
  accelerometer's axes. That anchor was an artifact of picking a burst out of a
  sloppy two-shove record. The accelerometer frame IS the body frame here, to
  within a few degrees.

  WHAT FINALLY SETTLED IT is the closed-loop run itself, which is the only
  measurement that samples the whole chain -- estimator, transform, allocation,
  actuators -- at once:

        commanded body direction   +2.4 deg   (allocated fans 3 and 4, 57/60%)
        achieved  body direction  +77.7 deg   (from dv/dt over 0.30 s)
        magnitude 0.637 m/s2 net vs 0.72 predicted -> both fans WERE thrusting

  Right magnitude, wrong direction = a rotated table, and the rotation is
  measured directly. Cross-check, raw `I50` angles against the adopted values:

        measured  fan1 -127.2  fan2 -49.2  fan3 +126.5  fan4 +47.6
        adopted   fan1 -130    fan2 -40    fan3 +140    fan4 +50
        uniform 5.6 deg -- inside the yaw contamination of the I50 runs

  The adopted values are the remembered table rotated -90 deg: the remembered
  angles were right in structure AND spacing, and only their zero was wrong.
  Measurement confirms the geometry; it does not out-resolve it.

  LESSON: a relative measurement plus a shaky anchor is a shaky answer, and it
  fails in a way that looks exactly like a correct answer. Anchor on the
  closed loop, which cannot lie about the end-to-end sign, and keep the
  divergence guard armed while you do.

  ⚠️ A SIGN ERROR HERE IS A RUNAWAY, not a debugging inconvenience. If a fan
  pushes opposite to what this table says, the controller sees the error GROW
  and pushes HARDER. With unguarded props (B7) that is the failure mode worth
  fearing. The divergence guard below is what actually caught it.

  ⚠️ FANS 2 AND 4 ARE WEAK, measured 2026-08-20 at a nominal 50%:
        fan 1  0.263 m/s2 -> A = 0.523   (model predicts 0.525)   healthy
        fan 3  0.245      -> A = 0.505                            healthy
        fan 2  0.101      -> A = 0.361   behaves like 41% throttle
        fan 4  0.118      -> A = 0.378   behaves like 42%, and produced
                                         NOTHING on 3 of 4 attempts
  That is the T3 backwards-prop / B17 reversed-direction signature on both
  channels. Fan 4 is the one whose spin direction must be re-sent over DSHOT at
  every boot because this ESC ignores SAVE_SETTINGS. Allocation currently
  assumes all four are equal and they are not.
  ---------------------------------------------------------------------------

  UNIDIRECTIONAL ACTUATORS ARE THE GENUINELY NEW PROBLEM. Fans only push. An
  opposing pair can produce +/- along its axis only if both idle above zero, so
  the idle bias is not a tuning nicety -- it is what makes the axis bidirectional
  at all. It also costs power continuously and must stay above the commutation
  floor, or the motor stops and takes time to restart mid-manoeuvre (T12).
*/

/* Plant, from Phase 2 (B14). A(pct) = TRANS_K_A * pct^2, metres/s^2. */
#define TRANS_K_A        2.1e-4f
#define TRANS_A_COULOMB  0.26f     /* breakaway ~35% throttle                 */

/* Below this the platform is treated as stopped, so the Coulomb feedforward
   pushes to break stiction rather than trying to cancel drag that is not there.
   The two branches have OPPOSITE SIGNS and getting the moving one backwards is
   worse than no feedforward at all (CONTROL_README section 6). */
#define TRANS_V_MOVING   0.015f    /* m/s                                     */
#define TRANS_DEADZONE   0.010f    /* m -- inside this, hold idle bias only   */

/* Idle bias, PERCENT throttle. Must clear the loaded commutation floor -- the
   ~2% measured bare is optimistic with props fitted (T12, guide 6.3 trap 2). */
#define TRANS_IDLE_PCT   12.0f

void trans_init(void);

/* Command a dock-frame target for the platform's MAGNET (not its centre) --
   that is the thing that has to land on the dock magnet, and it sits 9.46 cm
   ahead of the centre on an arm that rotates with heading. */
void trans_setTarget(float mag_x, float mag_y);
void trans_getTarget(float* mag_x, float* mag_y);

/* Run one control cycle. Returns false and commands nothing if the estimate is
   unusable -- no fix yet, or pose gone stale. Fans are left to the caller's
   inhibit logic in that case; this function never silently coasts on a pose it
   does not believe (T9). */
bool trans_update(const EstState* st, bool poseFresh);

void trans_enable(bool on);
bool trans_enabled(void);

void trans_setGains(float kp, float kd, float ffFrac);
void trans_getGains(float* kp, float* kd, float* ffFrac);

/* Last commanded values, for G and for capture. */
void trans_lastCommand(float* ax_dock, float* ay_dock,
                       float* pct1, float* pct2, float* pct3, float* pct4);
float trans_lastErr(void);

/* ---- divergence guard -------------------------------------------------------
   Catches the error class that makes an unverified fan map dangerous: if a fan
   pushes OPPOSITE to what the allocation table claims, the controller sees the
   error grow and responds by pushing HARDER. That is a runaway, and with
   unguarded props (B7) it is the failure worth engineering against rather than
   inspecting for.

   The guard is a mechanism, not a check: it does not care WHY the error is
   growing -- wrong fan angle, backwards prop, reversed ESC direction, a sign
   slip in the frame transform (T11) -- it only cares that commanded thrust is
   making things worse, which is never correct behaviour for a stable loop.

   WHILE APPROACHING the test is relative: the error must not exceed the best
   seen since enable by TRANS_DIVERGE_MULT plus a slack term. That is what
   catches a wrong angle within a few centimetres.

   ONCE ARRIVED (error has been inside the deadzone at least once) the test
   becomes a FIXED distance, TRANS_RUNAWAY_M. It has to: `bestErr` keeps
   falling, so on a successful move the relative threshold falls with it. A run
   that reached 1.6 mm ended up with a 32 mm trip threshold and was killed by
   ordinary station-keeping drift (measured 2026-08-20). A guard that fires
   because the controller did WELL is worse than no guard -- it teaches you to
   switch it off.

   TRANS_TIMEOUT_MS likewise stops applying after arrival. It asks "did this
   move ever get there", not "how long may it hold station"; leaving it armed
   would have killed the first working run 14 s after it succeeded. Station
   keeping ends with TS or X, not with a clock. */
#define TRANS_DIVERGE_MULT  1.5f
#define TRANS_DIVERGE_SLACK 0.03f    /* m                                      */
#define TRANS_RUNAWAY_M     0.15f    /* m, post-arrival abort distance         */
#define TRANS_TIMEOUT_MS    15000

bool        trans_tripped(void);
const char* trans_tripReason(void);
