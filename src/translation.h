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

        fan 1  +140 deg          fan 4   -40 deg     <- opposing pair
        fan 2  +230 deg          fan 3   +50 deg     <- opposing pair

  Two orthogonal axes. SETTLED 2026-08-21 at the remembered mounting + 180 deg,
  verified on hardware by sweeping TA0/90/180/270 against the real loop.

  ⚠️ THE HISTORY IS THE LESSON, not the number. Four values were tried over two
  days. The first three were each diagnosed from the platform's measured motion
  -- and the position estimate doing the measuring was itself broken:

     it put the platform 55 cm off-centre on a 61 cm table
     it reported four differently-angled fans all pushing along dock X, y flat
     it reported a COASTING platform accelerating to 0.57 m/s, fans off

  So every "correction" inherited that error and every one looked convincing. A
  wrong fan table and a broken position estimate produce the SAME symptom -- the
  platform moves confidently in the wrong direction -- so the symptom can never
  distinguish them. Only after vision-at-rest gave a reference worth trusting
  did a two-minute sweep settle it.

  The accelerometer measured the four fans consistently across three sessions
  (pooled: fan1 -130, fan2 -40, fan3 +121, fan4 +42), which pinned the GEOMETRY
  -- the pairing, the 90 deg spacing -- but not its relationship to the camera
  axis. That anchor is what took four attempts, and it is 180 deg from what was
  reasoned. The same flip appears in EST_ACC_ROT_DEG (270, not the reasoned 90):
  one inverted convention showing up in two places, not two separate errors.

  RULE THIS EARNED: fix the instrument, THEN identify the plant. Hours spent
  inferring this constant from a broken estimator were all wasted.

  ⚠️ A SIGN ERROR HERE IS A RUNAWAY. If a fan pushes opposite to what this table
  says, the controller sees the error GROW and pushes HARDER. With unguarded
  props (B7) that is the failure worth engineering against rather than
  inspecting for -- the divergence guard below is what actually caught it, twice.

  ⚠️ PER-FAN STRENGTH IS UNEQUAL AND MOVES AROUND. See FAN_K_A in the .cpp: the
  {2,3} axis has run at 63-69% of {1,4}, and fan 3 alone went 0.245 -> 0.104
  across one session's mechanical work. Re-measure all four with `TC` after ANY
  prop or ESC-direction change; nothing in firmware can detect it.
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

/* Approach speed cap, m/s. The docking approach has to be SLOW: the magnet has
   a couple of centimetres of tolerance, the estimate has more error than that,
   and an arrival at 0.3 m/s is a collision rather than a dock. Enforced by
   stripping the ACCELERATING component of the demand once the platform is at
   the cap -- braking and cross-track correction still get through, so this
   bounds speed without disabling control. */
#define TRANS_V_MAX      0.045f   /* default; runtime-settable with TL       */

/* Minimum commanded acceleration WHILE STUCK, m/s^2. Static breakaway exceeds
   the kinetic A_c the plant ID measured -- the rotation axis found the same
   thing and had to split A_static 60 from A_moving 34, a factor of 1.76, before
   small corrections would start at all (CONTROL_README section 12).

   It matters more here than it did there, because the velocity-shaped approach
   deliberately commands SMALL accelerations near the target: at 12 cm of error
   the profile asks for 0.38 m/s^2, and through the weak {2,3} axis only about
   0.25 of that arrives -- just under breakaway, so the fans spin and nothing
   moves. Guaranteeing a floor while stuck fixes that without making the whole
   approach aggressive, which is the trap a bigger gain would fall into. */
#define TRANS_A_STATIC   0.45f

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

/* Runtime rotation added to every FAN_ANGLE_DEG entry, degrees. The table has
   now been wrong three times, each time diagnosed from a position estimate that
   was itself broken, so it is exposed as a knob rather than re-argued: with a
   trustworthy stationary reference available, sweeping 0/90/180/270 against the
   real loop settles it in a couple of minutes. Whatever value wins should be
   folded into FAN_ANGLE_DEG and this returned to zero. */
/* Approach speed cap, runtime. Lowering it is the first thing to try against
   overshoot: arrival speed is what the last centimetres have to absorb, and the
   platform's only brake is friction plus whatever authority is left after the
   feedforward has cancelled it. */
/* OPTIONAL: stop cancelling Coulomb drag while BRAKING. Default OFF -- with it
   off the feedforward behaves exactly as before.

   Rationale: the MOVING branch cancels 0.26 m/s^2 of friction so the platform
   can cruise, but when the demand OPPOSES current velocity the controller is
   trying to stop, and friction is then the best brake the platform has.
   Cancelling it there throws that away at the one moment it is most useful,
   which is what lets a stick-slip release run all the way to the dock.
   Left as an option rather than made default because it was tried once and made
   things worse -- though that attempt was while the estimator was integrating
   backwards (EST_ACC_ROT_DEG was 180 deg out), so it never had a fair test. */
void  trans_setBrakeAssist(bool on);
bool  trans_getBrakeAssist(void);

void  trans_setVmax(float v);
float trans_getVmax(void);

void  trans_setFanRot(float deg);
float trans_getFanRot(void);

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

/* Largest move TT will accept. Not a control parameter -- an operator guard.
   TX/TY are offsets from the current magnet position, so a target is only
   meaningful once BOTH have been set; an unset axis still holds the power-on
   0.0, which in the dock frame is the WALL. Sending only TX therefore commands
   a full-authority charge at the dock, observed 2026-08-20. Nothing on a 61 cm
   table legitimately needs a 40 cm single move. */
#define TRANS_MAX_MOVE      0.40f

bool        trans_tripped(void);
const char* trans_tripReason(void);
