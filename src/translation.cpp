/* Fan translation controller. See translation.h. */
#include "translation.h"
#include "fans.h"
#include <math.h>

/* Velocity-shaped approach gains (see trans_update):
     s_kp  m/s of commanded speed per metre of error; the taper starts at
           TRANS_V_MAX/s_kp from the target
     s_kd  m/s^2 per m/s of velocity error
     s_ffFrac  fraction of the Coulomb feedforward applied */
static float s_kp     = 0.50f;
static float s_kd     = 6.00f;
static float s_ffFrac = 1.00f;

/* Fan push directions, deg CCW from the camera axis (see translation.h).
   Settled empirically by sweeping a runtime rotation against the real loop. */
static const float FAN_ANGLE_DEG[4] = { +140.0f, +230.0f, +50.0f, -40.0f };

static float s_vmax   = TRANS_V_MAX;
static bool  s_brakeAssist = false;
static float s_fanRot = 0.0f;      /* runtime rotation on the whole table, deg */
static bool  s_enabled = false;
static float s_tx = 0.0f, s_ty = 0.0f;
static float s_lastAx = 0.0f, s_lastAy = 0.0f, s_lastErr = 0.0f;
static float s_lastPct[4] = {0, 0, 0, 0};
static float s_bestErr = 1e9f;
static bool s_converged = false;   /* latched once err has been inside deadzone */
static uint32_t s_enableMs = 0;
static bool s_tripped = false;
static const char* s_tripWhy = "";

void trans_init(void) {
  s_enabled = false;
  s_tx = 0.0f;
  s_ty = 0.0f;
  s_lastAx = s_lastAy = s_lastErr = 0.0f;
  for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
}

void trans_setTarget(float mx, float my) { s_tx = mx; s_ty = my; }
void trans_getTarget(float* mx, float* my) { if (mx) *mx = s_tx; if (my) *my = s_ty; }

void trans_enable(bool on) {
  s_enabled = on;
  s_bestErr = 1e9f;
  s_converged = false;
  s_enableMs = millis();
  s_tripped = false;
  s_tripWhy = "";
  if (!on) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
  }
}
bool trans_enabled(void) { return s_enabled; }

void  trans_setBrakeAssist(bool on) { s_brakeAssist = on; }
bool  trans_getBrakeAssist(void)    { return s_brakeAssist; }

void  trans_setVmax(float v) { if (v > 0.0f && v <= 0.30f) s_vmax = v; }
float trans_getVmax(void)    { return s_vmax; }

void  trans_setFanRot(float deg) { s_fanRot = deg; }
float trans_getFanRot(void)      { return s_fanRot; }

void trans_setGains(float kp, float kd, float ff) {
  if (kp >= 0.0f) s_kp = kp;
  if (kd >= 0.0f) s_kd = kd;
  if (ff >= 0.0f && ff <= 1.5f) s_ffFrac = ff;
}
void trans_getGains(float* kp, float* kd, float* ff) {
  if (kp) *kp = s_kp; if (kd) *kd = s_kd; if (ff) *ff = s_ffFrac;
}

/* Per-fan thrust constants, A(pct) = K * pct^2. Measured from single-fan
   probes as K = (net |a| + A_c) / 50^2; the {2,3} axis is noticeably weaker.
   Compensating matters mainly for breakaway along the weak axis. fan 3 is
   trimmed down from the measured 1.46e-4 (lower K = more throttle). Re-measure
   after any prop or ESC change. */
static const float FAN_K_A[4] = { 2.07e-4f, 1.33e-4f, 1.20e-4f, 1.98e-4f };

/* Acceleration -> throttle percent for one fan (square-law inversion). */
static inline float accelToPct(int fan, float a) {
  if (a <= 0.0f) return 0.0f;
  float pct = sqrtf(a / FAN_K_A[fan]);
  if (pct > FAN_THROTTLE_MAX) pct = FAN_THROTTLE_MAX;
  return pct;
}

/* Thrust a fan produces at idle bias. The opposing fan has to overcome it. */
static inline float idleAccel(int fan) {
  return FAN_K_A[fan] * TRANS_IDLE_PCT * TRANS_IDLE_PCT;
}

bool trans_update(const EstState* st, bool poseFresh) {
  if (!s_enabled) return false;

  /* No usable pose: zero throttle but stay enabled, so control resumes as soon
     as the estimate returns. poseFresh is not checked because position is
     dead-reckoned from the IMU and stays valid without vision. */
  (void)poseFresh;
  if (!st || !st->valid) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
    /* A blind interval is not lack of progress; reset the timeout clock. */
    s_enableMs = millis();
    return false;
  }

  /* Error on the magnet, dock frame. */
  float ex = s_tx - st->mag_x;
  float ey = s_ty - st->mag_y;
  float err = sqrtf(ex * ex + ey * ey);
  s_lastErr = err;

  /* Divergence guard, checked before any thrust is computed. Relative while
     approaching, fixed distance once arrived (see translation.h). */
  if (err < s_bestErr) s_bestErr = err;
  if (err < TRANS_DEADZONE) s_converged = true;

  float trip = s_converged ? TRANS_RUNAWAY_M
                           : (s_bestErr * TRANS_DIVERGE_MULT + TRANS_DIVERGE_SLACK);
  if (err > trip) {
    s_tripped = true;
    s_tripWhy = s_converged
        ? "RUNAWAY: arrived, then lost the target while station-keeping."
        : "DIVERGING: error growing under thrust. Suspect a fan pushing "
          "opposite to the allocation table, a backwards prop, or a "
          "frame-transform sign.";
  } else if (!s_converged && millis() - s_enableMs > TRANS_TIMEOUT_MS) {
    s_tripped = true;
    s_tripWhy = "TIMEOUT: did not converge. Suspect too little authority, "
                "friction feedforward too low, or a stuck platform.";
  }
  if (s_tripped) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
    s_enabled = false;
    return false;
  }

  if (err < TRANS_DEADZONE) {
    /* Inside tolerance: idle bias only, keeping the motors spinning. */
    fans_setAll(TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT);
    for (int i = 0; i < 4; i++) s_lastPct[i] = TRANS_IDLE_PCT;
    s_lastAx = s_lastAy = 0.0f;
    return true;
  }

  /* Velocity-shaped approach: cruise at s_vmax far out, taper linearly inside
     s_vmax/s_kp, and command the acceleration needed to track that profile.
     No sign inversion: +x acceleration moves the platform +x (unlike the
     reaction wheel axis). */
  float vdes = s_kp * err;
  if (vdes > s_vmax) vdes = s_vmax;
  float vdx = vdes * (ex / err), vdy = vdes * (ey / err);

  float ax = s_kd * (vdx - st->vx);
  float ay = s_kd * (vdy - st->vy);

  /* Coulomb feedforward. Plant: x_ddot = A_cmd - A_c*sign(v), so cancelling
     drag needs +A_c*sign(v). This is positive, unlike the rotation axis where
     the wheel pushes the platform the opposite way. With the correct sign the
     moving and stuck branches agree when heading toward the target, so the
     TRANS_V_MOVING crossing is smooth. */
  float speed = sqrtf(st->vx * st->vx + st->vy * st->vy);
  float ffx = 0.0f, ffy = 0.0f;
  if (speed > TRANS_V_MOVING) {
    bool braking = s_brakeAssist && (ax * st->vx + ay * st->vy <= 0.0f);
    if (!braking) {
      ffx = +TRANS_A_COULOMB * (st->vx / speed); /* moving: cancel drag       */
      ffy = +TRANS_A_COULOMB * (st->vy / speed);
    }
  } else {
    float m = sqrtf(ax * ax + ay * ay);
    if (m > 1e-6f) {
      ffx = +TRANS_A_COULOMB * (ax / m);         /* stuck: push to break free */
      ffy = +TRANS_A_COULOMB * (ay / m);
    }
  }
  ax += s_ffFrac * ffx;
  ay += s_ffFrac * ffy;

  /* Breakaway floor while stuck. */
  if (speed <= TRANS_V_MOVING) {
    float m = sqrtf(ax * ax + ay * ay);
    if (m > 1e-6f && m < TRANS_A_STATIC) {
      float g = TRANS_A_STATIC / m;
      ax *= g; ay *= g;
    }
  }
  s_lastAx = ax; s_lastAy = ay;

  /* Dock frame -> body frame. At psi = 0 the camera faces the wall, so
     body-forward is (0,-1) and body-right is (+1,0) in dock coordinates. */
  float sp = sinf(st->psi), cp = cosf(st->psi);
  float fwd = ax * (-sp) + ay * (-cp);
  float rgt = ax * ( cp) + ay * (-sp);

  /* Allocate onto the two opposing pairs. The sign of the projection picks
     which fan of the pair pushes; the other stays at idle. */
  float pct[4] = {0, 0, 0, 0};
  const int pairs[2][2] = { {0, 3}, {1, 2} };    /* {fan1,fan4}, {fan2,fan3}  */

  for (int p = 0; p < 2; p++) {
    int ia = pairs[p][0], ib = pairs[p][1];
    float th = (FAN_ANGLE_DEG[ia] + s_fanRot) * (float)M_PI / 180.0f;
    /* CCW from forward is toward the left, i.e. negative right. */
    float dfwd = cosf(th), drgt = -sinf(th);
    float c = fwd * dfwd + rgt * drgt;           /* projection onto this axis */

    /* The pushing fan also has to overcome its partner's idle thrust. */
    if (c >= 0.0f) {
      pct[ia] = accelToPct(ia, c + idleAccel(ib));
      pct[ib] = TRANS_IDLE_PCT;
    } else {
      pct[ib] = accelToPct(ib, -c + idleAccel(ia));
      pct[ia] = TRANS_IDLE_PCT;
    }
  }

  for (int i = 0; i < 4; i++) s_lastPct[i] = pct[i];
  fans_setAll(pct[0], pct[1], pct[2], pct[3]);
  return true;
}

void trans_lastCommand(float* axd, float* ayd,
                       float* p1, float* p2, float* p3, float* p4) {
  if (axd) *axd = s_lastAx;  if (ayd) *ayd = s_lastAy;
  if (p1) *p1 = s_lastPct[0]; if (p2) *p2 = s_lastPct[1];
  if (p3) *p3 = s_lastPct[2]; if (p4) *p4 = s_lastPct[3];
}

float trans_lastErr(void) { return s_lastErr; }
bool        trans_tripped(void)    { return s_tripped; }
const char* trans_tripReason(void) { return s_tripWhy; }
