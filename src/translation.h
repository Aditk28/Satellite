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

        fan 1   -40 deg          fan 4  +140 deg     <- opposing pair
        fan 2   +50 deg          fan 3  +230 deg     <- opposing pair

  Two orthogonal axes, rotated 40 deg off the camera axis.

  ⚠️ A SIGN ERROR HERE IS A RUNAWAY, not a debugging inconvenience. If a fan
  pushes opposite to what this table says, the controller sees the error GROW
  and pushes HARDER. With unguarded props (B7) that is the failure mode worth
  fearing. VERIFY OPEN-LOOP with S<n>/L<pct> before ever closing the loop, and
  if one is wrong fix it HERE rather than compensating downstream (T11).

  ⚠️ FAN 2 produced no measurable thrust in the Step 2.4 runs. Spin it and watch
  before trusting any allocation that depends on it.
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

   Trips when the error exceeds the best seen since enable by TRANS_DIVERGE_MULT
   plus a slack term (so noise near the target cannot trip it), or when the move
   simply fails to converge within TRANS_TIMEOUT_MS. Both kill the fans and
   latch, requiring a deliberate re-enable. */
#define TRANS_DIVERGE_MULT  1.5f
#define TRANS_DIVERGE_SLACK 0.03f    /* m  */
#define TRANS_TIMEOUT_MS    15000

bool        trans_tripped(void);
const char* trans_tripReason(void);
