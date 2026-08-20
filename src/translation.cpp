#include "translation.h"
#include "fans.h"
#include <math.h>

/* ---- gains -----------------------------------------------------------------
   Double integrator, acceleration input, so straight from the standard form:
       K_p = omega_n^2      K_d = 2*zeta*omega_n      omega_n = 5.714/t_settle

   Starting at t_settle = 3 s, zeta = 0.8. DELIBERATELY SLOW TO START, which is
   the opposite of the rotation-axis advice to "work DOWN the table" -- and the
   reason is the actuator, not the plant. On the wheel axis a wrong gain wound a
   flywheel and hit an abort. Here a wrong gain drives four unguarded props into
   a runaway, so the first closed loop is the cautious one and we speed it up
   from data. CONTROL_README section 9's argument still applies once the signs
   are proven: slow gains keep the feedforward active longer, which is worse on
   both accuracy and saturation.

       omega_n = 5.714/3.0 = 1.90    K_p = 3.63    K_d = 2*0.8*1.90 = 3.05   */
static float s_kp     = 3.63f;
static float s_kd     = 3.05f;
static float s_ffFrac = 0.90f;

/* Fan push directions, degrees CCW from the camera axis. See translation.h --
   this is the constant that turns a bug into a runaway. */
static const float FAN_ANGLE_DEG[4] = { -40.0f, +50.0f, +230.0f, +140.0f };

static bool  s_enabled = false;
static float s_tx = 0.0f, s_ty = 0.0f;
static float s_lastAx = 0.0f, s_lastAy = 0.0f, s_lastErr = 0.0f;
static float s_lastPct[4] = {0, 0, 0, 0};
static float s_bestErr = 1e9f;
static uint32_t s_enableMs = 0;
static bool s_tripped = false;
static const char* s_tripWhy = "";

void trans_init(void) {
  s_enabled = false;
  s_tx = 0.0f;
  s_ty = TRANS_A_COULOMB * 0.0f;      /* target set explicitly before enable  */
  s_lastAx = s_lastAy = s_lastErr = 0.0f;
  for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
}

void trans_setTarget(float mx, float my) { s_tx = mx; s_ty = my; }
void trans_getTarget(float* mx, float* my) { if (mx) *mx = s_tx; if (my) *my = s_ty; }

void trans_enable(bool on) {
  s_enabled = on;
  s_bestErr = 1e9f;
  s_enableMs = millis();
  s_tripped = false;
  s_tripWhy = "";
  if (!on) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
  }
}
bool trans_enabled(void) { return s_enabled; }

void trans_setGains(float kp, float kd, float ff) {
  if (kp >= 0.0f) s_kp = kp;
  if (kd >= 0.0f) s_kd = kd;
  if (ff >= 0.0f && ff <= 1.5f) s_ffFrac = ff;
}
void trans_getGains(float* kp, float* kd, float* ff) {
  if (kp) *kp = s_kp; if (kd) *kd = s_kd; if (ff) *ff = s_ffFrac;
}

/* Acceleration -> throttle percent. The square-law inversion is the fan
   analogue of the wheel axis's feedback linearisation, but STATIC -- there is
   no actuator state to cancel because the ESCs give us nothing back
   (no RPM, no thrust, no per-channel current). */
static inline float accelToPct(float a) {
  if (a <= 0.0f) return 0.0f;
  float pct = sqrtf(a / TRANS_K_A);
  if (pct > FAN_THROTTLE_MAX) pct = FAN_THROTTLE_MAX;
  return pct;
}

bool trans_update(const EstState* st, bool poseFresh) {
  if (!s_enabled) return false;

  /* No usable pose -> withdraw thrust, but SOFTLY: zero throttle with the ESC
     left armed, so authority returns the instant vision does. Staying enabled
     is deliberate -- losing sight of the dock is a normal operating condition
     (SEARCH, close approach) and must not require an operator to re-arm
     (B21b). What is NOT acceptable is coasting on a pose we no longer believe
     (T9), so nothing is commanded until it comes back. */
  if (!st || !st->valid || !poseFresh) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
    /* The divergence guard measures progress, and a blind interval is not
       lack of progress -- forgive the clock so a dropout cannot trip it. */
    s_enableMs = millis();
    return false;
  }

  /* ---- error on the MAGNET, in the dock frame. Not the platform centre: the
     magnet is what has to land, and it sits on an arm that rotates with psi. */
  float ex = s_tx - st->mag_x;
  float ey = s_ty - st->mag_y;
  float err = sqrtf(ex * ex + ey * ey);
  s_lastErr = err;

  /* ---- divergence guard. Checked BEFORE any thrust is computed, so a trip
     cannot be followed by one more push in the wrong direction. */
  if (err < s_bestErr) s_bestErr = err;
  if (err > s_bestErr * TRANS_DIVERGE_MULT + TRANS_DIVERGE_SLACK) {
    s_tripped = true;
    s_tripWhy = "DIVERGING -- error growing under thrust. Suspect a fan "
                "pushing opposite to the allocation table, a backwards prop, "
                "or a frame-transform sign.";
  } else if (millis() - s_enableMs > TRANS_TIMEOUT_MS) {
    s_tripped = true;
    s_tripWhy = "TIMEOUT -- did not converge. Suspect too little authority, "
                "friction feedforward too low, or a stuck platform.";
  }
  if (s_tripped) {
    fans_setAll(0, 0, 0, 0);
    for (int i = 0; i < 4; i++) s_lastPct[i] = 0.0f;
    s_enabled = false;
    return false;
  }

  if (err < TRANS_DEADZONE) {
    /* Inside tolerance: command nothing but keep the idle bias so the motors
       stay above the commutation floor and authority is instant if we drift. */
    fans_setAll(TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT, TRANS_IDLE_PCT);
    for (int i = 0; i < 4; i++) s_lastPct[i] = TRANS_IDLE_PCT;
    s_lastAx = s_lastAy = 0.0f;
    return true;
  }

  /* ---- PD. No sign inversion here, unlike the wheel axis: commanding +x
     acceleration moves the platform +x, whereas the reaction wheel moves the
     platform OPPOSITE to the wheel (CONTROL_README section 8's minus sign).
     Do not copy that sign across. */
  float ax = s_kp * ex - s_kd * st->vx;
  float ay = s_kp * ey - s_kd * st->vy;

  /* ---- Coulomb feedforward. Structure ported verbatim from the rotation axis
     (CONTROL_README section 6): friction here is 0.26 m/s^2 against manoeuvres
     of a similar size, so it is a dominant term, not a correction.
     THE TWO BRANCHES HAVE OPPOSITE SIGNS and getting the moving one backwards
     is worse than no feedforward at all. */
  float speed = sqrtf(st->vx * st->vx + st->vy * st->vy);
  float ffx = 0.0f, ffy = 0.0f;
  if (speed > TRANS_V_MOVING) {
    ffx = -TRANS_A_COULOMB * (st->vx / speed);   /* MOVING: cancel drag       */
    ffy = -TRANS_A_COULOMB * (st->vy / speed);
  } else {
    float m = sqrtf(ax * ax + ay * ay);
    if (m > 1e-6f) {
      ffx = +TRANS_A_COULOMB * (ax / m);         /* STUCK: push to break free */
      ffy = +TRANS_A_COULOMB * (ay / m);
    }
  }
  ax += s_ffFrac * ffx;
  ay += s_ffFrac * ffy;
  s_lastAx = ax; s_lastAy = ay;

  /* ---- dock frame -> body frame.
     At psi = 0 the camera faces the wall, so body-forward is (0,-1) in dock
     coordinates and body-right is (+1,0) -- "right as you face the wall" is
     how the dock frame's +X was defined, and the camera faces the wall. */
  float sp = sinf(st->psi), cp = cosf(st->psi);
  float fwd = ax * (-sp) + ay * (-cp);
  float rgt = ax * ( cp) + ay * (-sp);

  /* ---- allocate onto the two opposing pairs. Each pair spans one axis; the
     sign of the projection picks which fan of the pair does the pushing, and
     the other stays at idle. This is the whole unidirectional-actuator
     problem: we cannot ask a fan for negative thrust, so we ask its opposite. */
  float pct[4] = {0, 0, 0, 0};
  const int pairs[2][2] = { {0, 3}, {1, 2} };    /* {fan1,fan4}, {fan2,fan3}  */

  for (int p = 0; p < 2; p++) {
    int ia = pairs[p][0], ib = pairs[p][1];
    float th = FAN_ANGLE_DEG[ia] * (float)M_PI / 180.0f;
    /* Fan direction in body coords: CCW from forward means turning toward the
       LEFT, which is negative "right". */
    float dfwd = cosf(th), drgt = -sinf(th);
    float c = fwd * dfwd + rgt * drgt;           /* projection onto this axis */

    float a_idle = TRANS_K_A * TRANS_IDLE_PCT * TRANS_IDLE_PCT;
    if (c >= 0.0f) {
      pct[ia] = accelToPct(a_idle + c);
      pct[ib] = TRANS_IDLE_PCT;
    } else {
      pct[ib] = accelToPct(a_idle - c);
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
