/* Dock-relative pose estimator. See estimator.h. */
#include "estimator.h"
#include <math.h>

/* Gains are applied per vision correction, not per control cycle.
   s_alphaPos  position pull toward vision (only applied at rest)
   s_betaVel   velocity from the position innovation. Zero because velocity
               comes from the accelerometer, and differencing position
               amplifies noise by 1/dt.
   s_alphaPsi  heading pull toward vision yaw. Slow on purpose so per-frame yaw
               noise averages down (~1.8 s time constant at 28 Hz). */
static float s_alphaPos = 0.50f;
static float s_betaVel  = 0.0f;
static float s_alphaPsi = 0.02f;

/* Reject fixes with a range outside anything reachable on this table. */
#define EST_RANGE_MIN   0.05f
#define EST_RANGE_MAX   2.50f

static EstState  s_st;
static float     s_psiOffset = 0.0f;   /* psi = EST_THETA_SIGN*theta + this  */
static float     s_accRot = EST_ACC_ROT_DEG * (float)M_PI / 180.0f;
static bool      s_haveOffset = false;
static uint16_t  s_lastSeq = 0;
static bool      s_haveSeq = false;
static uint32_t  s_fixes = 0, s_rejects = 0;

static float s_faX = 0.0f, s_faY = 0.0f;   /* low-passed accel, sensor axes  */
static float s_bX  = 0.0f, s_bY  = 0.0f;   /* residual accel bias            */
static int    s_snapWant = 0, s_snapGot = 0;
static double s_snapSumX = 0.0, s_snapSumY = 0.0;

static inline float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

void est_init(void) {
  s_st = (EstState){0};
  s_psiOffset = 0.0f;
  s_faX = s_faY = s_bX = s_bY = 0.0f;
  s_snapWant = s_snapGot = 0;
  s_haveOffset = false;
  s_haveSeq = false;
  s_fixes = s_rejects = 0;
}

static float s_magLat = EST_MAG_LAT;

/* body forward in dock coords = (-sin psi, -cos psi)
   body left    in dock coords = (-cos psi, +sin psi)
   magnet = centre + L_MAG*forward + magLat*left */
static void updateMagnet(void) {
  float sp = sinf(s_st.psi), cp = cosf(s_st.psi);
  s_st.mag_x = s_st.x - EST_L_MAG * sp - s_magLat * cp;
  s_st.mag_y = s_st.y - EST_L_MAG * cp + s_magLat * sp;
}

void  est_setMagLat(float m) { s_magLat = m; updateMagnet(); }
float est_getMagLat(void)    { return s_magLat; }

void est_setMagnetPose(float mx, float my) {
  float sp = sinf(s_st.psi), cp = cosf(s_st.psi);
  s_st.x = mx + EST_L_MAG * sp + s_magLat * cp;
  s_st.y = my + EST_L_MAG * cp - s_magLat * sp;
  s_st.vx = s_st.vy = 0.0f;
  s_st.valid = true;
  s_st.lastFixMs = millis();
  updateMagnet();
}

/* At rest acceleration is also zero, so the filtered reading is a direct
   measurement of residual bias. This is the only bias correction after boot. */
void est_zupt(void) {
  s_st.vx = 0.0f; s_st.vy = 0.0f;
  s_bX += s_faX;  s_bY += s_faY;
  s_faX = 0.0f;   s_faY = 0.0f;
}
void est_snapBegin(int n) {
  s_snapWant = (n > 0) ? n : 1;
  s_snapGot = 0;
  s_snapSumX = s_snapSumY = 0.0;
}
bool est_snapDone(void) { return s_snapWant == 0; }
void est_snapCancel(void) { s_snapWant = 0; s_snapGot = 0; }
float est_psiOffset(void) { return s_psiOffset; }

void est_setAccRot(float deg) { s_accRot = deg * (float)M_PI / 180.0f; }
float est_getAccRot(void)     { return s_accRot * 180.0f / (float)M_PI; }

void est_predict(float dt, float theta, float ax_body, float ay_body) {
  /* Heading is not integrated here; the rotation controller already does that.
     We only carry the offset from platform-relative to dock-relative. */
  float th = EST_THETA_SIGN * theta;
  if (s_haveOffset) s_st.psi = wrapPi(th + s_psiOffset);

  /* Dead reckoning. Low-pass before integrating, then rotate sensor axes ->
     body (forward, left) by s_accRot, then body -> dock by psi. */
  float k = dt / (EST_ACC_TAU + dt);
  s_faX += k * ((ax_body - s_bX) - s_faX);
  s_faY += k * ((ay_body - s_bY) - s_faY);

  float cr = cosf(s_accRot), sr = sinf(s_accRot);
  float fwd  = s_faX * cr - s_faY * sr;
  float left = s_faX * sr + s_faY * cr;

  float sp = sinf(s_st.psi), cp = cosf(s_st.psi);
  float ax_dock = fwd * (-sp) + left * (-cp);
  float ay_dock = fwd * (-cp) + left * ( sp);

  /* Integrate only once there is a pose to integrate from. */
  if (s_st.valid) {
    s_st.vx += ax_dock * dt;
    s_st.vy += ay_dock * dt;
    float leak = 1.0f - dt / EST_V_LEAK_TAU;
    s_st.vx *= leak;
    s_st.vy *= leak;
    s_st.x  += s_st.vx * dt;
    s_st.y  += s_st.vy * dt;
  }
  updateMagnet();
}

bool est_correct(const PiPose* p, float theta, float omega_p) {
  if (!p) return false;

  /* Both gyro quantities arrive CW-positive; convert to the CCW dock frame. */
  float th = EST_THETA_SIGN * theta;
  float wz = EST_THETA_SIGN * omega_p;

  /* Never apply the same measurement twice. */
  if (s_haveSeq && p->seq == s_lastSeq) return false;
  s_lastSeq = p->seq;
  s_haveSeq = true;

  if (!(p->flags & PI_FLAG_VALID)) return false;
  if (p->range_m < EST_RANGE_MIN || p->range_m > EST_RANGE_MAX) {
    s_rejects++;
    return false;
  }

  /* Latency compensation: the frame describes the pose age_us ago, so the
     measurement is propagated forward to now. */
  float age = (float)p->age_us * 1e-6f;
  if (age > 0.5f) age = 0.5f;

  float psi_meas = wrapPi(p->relyaw_rad + wz * age);

  if (!s_haveOffset) {
    /* First fix: adopt it outright. */
    s_psiOffset = wrapPi(psi_meas - th);
    s_haveOffset = true;
    s_st.psi = psi_meas;
  } else {
    /* Slow blend; trust vision yaw less when the solver reports ambiguity. */
    float g = (p->flags & PI_FLAG_AMBIGUOUS) ? (s_alphaPsi * 0.25f)
                                             : s_alphaPsi;
    s_psiOffset = wrapPi(s_psiOffset +
                         g * wrapPi(psi_meas - wrapPi(th + s_psiOffset)));
    s_st.psi = wrapPi(th + s_psiOffset);
  }

  /* Camera -> platform centre in the dock frame. Uses the filtered psi, not raw
     vision yaw, which is too noisy and correlates with bearing. */
  float psi = s_st.psi;
  float cam_x = p->range_m * sinf(psi - p->bearing_rad);
  float cam_y = p->range_m * cosf(psi - p->bearing_rad);
  float mx = cam_x + EST_L_CAM * sinf(psi);
  float my = cam_y + EST_L_CAM * cosf(psi);

  mx += s_st.vx * age;
  my += s_st.vy * age;

  if (!s_st.valid) {
    s_st.x = mx; s_st.y = my;
    s_st.vx = s_st.vy = 0.0f;
    s_st.valid = true;
  } else {
    /* Time since the last fix, not the control period. */
    float dtFix = (float)(millis() - s_st.lastFixMs) * 1e-3f;
    if (dtFix < 0.005f) dtFix = 0.005f;
    if (dtFix > 0.5f)   dtFix = 0.5f;

    if (s_snapWant > 0) {
      s_snapSumX += mx; s_snapSumY += my;
      if (++s_snapGot >= s_snapWant) {
        s_st.x = (float)(s_snapSumX / s_snapGot);
        s_st.y = (float)(s_snapSumY / s_snapGot);
        s_st.vx = s_st.vy = 0.0f;
        s_snapWant = 0;
      }
      s_st.lastFixMs = millis();
      updateMagnet();
      s_fixes++;
      return true;
    }

    float ix = mx - s_st.x, iy = my - s_st.y;
    float sp2 = sqrtf(s_st.vx * s_st.vx + s_st.vy * s_st.vy);
    float gp = (sp2 < EST_VIS_VMAX) ? s_alphaPos : 0.0f;
    s_st.x  += gp * ix;
    s_st.y  += gp * iy;
    s_st.vx += s_betaVel  * ix / dtFix;
    s_st.vy += s_betaVel  * iy / dtFix;
  }

  s_st.lastFixMs = millis();
  updateMagnet();
  s_fixes++;
  return true;
}

bool est_get(EstState* out) {
  if (!out) return false;
  *out = s_st;
  return s_st.valid;
}

void est_setGains(float a, float b, float c) {
  if (a > 0.0f && a <= 1.0f) s_alphaPos = a;
  if (b >= 0.0f && b <= 1.0f) s_betaVel = b;
  if (c > 0.0f && c <= 1.0f) s_alphaPsi = c;
}

void est_getGains(float* a, float* b, float* c) {
  if (a) *a = s_alphaPos;
  if (b) *b = s_betaVel;
  if (c) *c = s_alphaPsi;
}

uint32_t est_fixes(void)   { return s_fixes; }
uint32_t est_rejects(void) { return s_rejects; }
