#pragma once
#include <Arduino.h>
#include "estimator.h"

/*
  x/y translation control on the four fans.

  EstState (dock frame) -> velocity-shaped approach + Coulomb feedforward
  -> a_desired (dock) -> rotate by psi -> body frame -> opposing fan pairs
  -> square-law inversion -> throttle percent.

  The plant is identified in acceleration units, so mass never appears.
  Fans only push, so each axis is an opposing pair that both idle above zero;
  the idle bias is what makes an axis bidirectional.

  Fan push directions, body frame, deg CCW from the camera axis:
      fan 1 +140, fan 4 -40   (opposing pair)
      fan 2 +230, fan 3 +50   (opposing pair)
  A sign error in this table is a runaway (error grows, controller pushes
  harder); the divergence guard below is the protection. Per-fan strength is
  unequal and drifts with mechanical changes; re-measure with TC after any prop
  or ESC change.
*/

/* Plant: A(pct) = TRANS_K_A * pct^2, m/s^2. */
#define TRANS_K_A        2.1e-4f
#define TRANS_A_COULOMB  0.26f     /* breakaway ~35% throttle                 */

/* Below this speed the platform is treated as stopped, so the Coulomb
   feedforward pushes to break stiction instead of cancelling drag. */
#define TRANS_V_MOVING   0.015f    /* m/s                                     */
#define TRANS_DEADZONE   0.010f    /* m, inside this hold idle bias only      */

/* Idle bias, percent throttle. Must clear the loaded commutation floor. */
#define TRANS_IDLE_PCT   12.0f

/* Approach speed cap, m/s. Docking has to arrive slowly. Runtime: TL. */
#define TRANS_V_MAX      0.045f

/* Minimum commanded acceleration while stuck, m/s^2. Static breakaway exceeds
   the kinetic A_c, and the approach profile asks for small accelerations near
   the target, which the weak {2,3} axis cannot turn into motion. */
#define TRANS_A_STATIC   0.45f

void trans_init(void);

/* Dock-frame target for the platform's magnet, not its centre. */
void trans_setTarget(float mag_x, float mag_y);
void trans_getTarget(float* mag_x, float* mag_y);

/* Run one control cycle. Returns false if disabled, or (after zeroing the fans)
   if the guard trips or the estimate is not valid. poseFresh is currently
   unused since position is dead-reckoned. */
bool trans_update(const EstState* st, bool poseFresh);

void trans_enable(bool on);
bool trans_enabled(void);

/* When on, skip the Coulomb drag cancellation while the demand opposes
   velocity, so friction helps braking. Default off. */
void  trans_setBrakeAssist(bool on);
bool  trans_getBrakeAssist(void);

void  trans_setVmax(float v);
float trans_getVmax(void);

/* Runtime rotation added to every fan angle, deg. Normally zero. */
void  trans_setFanRot(float deg);
float trans_getFanRot(void);

void trans_setGains(float kp, float kd, float ffFrac);
void trans_getGains(float* kp, float* kd, float* ffFrac);

/* Last commanded values, for G and for capture. */
void trans_lastCommand(float* ax_dock, float* ay_dock,
                       float* pct1, float* pct2, float* pct3, float* pct4);
float trans_lastErr(void);

/* Divergence guard. While approaching, trip if the error exceeds
   bestErr * TRANS_DIVERGE_MULT + TRANS_DIVERGE_SLACK. Once the error has been
   inside the deadzone, switch to the fixed TRANS_RUNAWAY_M distance, because a
   relative threshold shrinks with a good result and would trip on ordinary
   station-keeping drift. The timeout also only applies before arrival. */
#define TRANS_DIVERGE_MULT  1.5f
#define TRANS_DIVERGE_SLACK 0.03f    /* m */
#define TRANS_RUNAWAY_M     0.15f    /* m, post-arrival abort distance */
#define TRANS_TIMEOUT_MS    15000

/* Largest single move TT will accept, m. An operator guard: an unset TX/TY
   axis is 0.0, which in the dock frame is the wall. */
#define TRANS_MAX_MOVE      0.40f

bool        trans_tripped(void);
const char* trans_tripReason(void);
