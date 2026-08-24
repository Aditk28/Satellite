#include "estimator.h"
#include "timebase.h"
#include <math.h>

/* ---- gains, and why each is what it is -----------------------------------
   These are applied PER CORRECTION (~28 Hz), not per control cycle, so a gain
   g gives a time constant of roughly 1/(28*g) seconds.

   EST_ALPHA_POS 0.35 -> ~0.10 s. Vision position is the TRUSTED source here
       (range matches a tape measure), and with no accelerometer prediction the
       between-frame model is only constant velocity, so there is nothing to be
       gained by leaning on the prediction. Correct hard and fast.

   EST_BETA_VEL 0.08 -> velocity inferred from the position innovation. Kept
       small deliberately: velocity from differenced position amplifies noise by
       1/dt, and at 36 ms a 1 cm position wobble is 0.28 m/s of phantom speed.

   EST_ALPHA_PSI 0.02 -> ~1.8 s. This one is SLOW ON PURPOSE. It is the whole
       mechanism for averaging down noisy vision yaw: sqrt(N) over ~50 frames
       turns +-10 deg of per-frame noise into ~+-1.4 deg, while the gyro carries
       the fast changes. Raising this re-admits the noise that was corrupting
       position in the first place.                                          */
/* DROPPED 0.35 -> 0.05 when position went IMU-led. Vision is now a slow drift
   corrector, not the source of truth: at ~10 fixes/s a gain of 0.05 pulls the
   estimate toward vision with a ~2 s time constant, enough to bound accelerometer
   drift over a docking run while being far too slow to yank the estimate when
   vision produces one of its 50 cm outliers. */
static float s_alphaPos = 0.50f;
/* 0.08 -> 0.0. Velocity now comes from integrating the accelerometer, so
   inferring it AGAIN from the vision position innovation would double-count the
   same information (T10's family) and inject the exact 1/dt noise amplification
   the original comment warned about. */
static float s_betaVel  = 0.0f;
static float s_alphaPsi = 0.02f;

/* Reject a fix whose range is outside anything physically reachable on this
   table, or whose yaw the solver admits it cannot resolve. Cheap gate; the
   estimator has no way to recover from swallowing a wild outlier. */
#define EST_RANGE_MIN   0.05f
#define EST_RANGE_MAX   2.50f

static EstState  s_st;
static float     s_psiOffset = 0.0f;   /* psi = theta + this                 */
static float     s_accRot = EST_ACC_ROT_DEG * (float)M_PI / 180.0f;
static bool      s_haveOffset = false;
static uint16_t  s_lastSeq = 0;
static bool      s_haveSeq = false;
static uint32_t  s_fixes = 0, s_rejects = 0;

static float s_faX = 0.0f, s_faY = 0.0f;   /* low-passed body accel          */
static float s_bX  = 0.0f, s_bY  = 0.0f;   /* residual accel bias, body frame */
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

/* Magnet position follows from the centre and the heading -- recomputed rather
   than filtered separately, so the two can never disagree. */
static float s_magLat = EST_MAG_LAT;

/* body forward in dock coords = (-sin psi, -cos psi)
   body left    in dock coords = (-cos psi, +sin psi)
   magnet = centre + L_MAG*forward + magLat*left                              */
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

/* Zero-velocity AND zero-acceleration update. Velocity is known to be zero, and
   so is ACCELERATION -- which makes the low-passed reading at this instant a
   direct measurement of the residual bias that `B` left behind at boot. Folding
   it in here is the only bias correction the system gets after startup, and it
   is free: the platform tells us every time it stops.
   Why this matters more than filtering: the drift is a DC error. A low-pass
   removes vibration and does nothing at all to a constant offset, and a
   double integrator turns 0.015 m/s^2 of offset into 0.13 m/s of phantom speed
   in 9 s -- which the velocity-shaped control law then subtracts straight off
   the command, so the platform sits there believing it is already moving. */
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

void est_setPose(float x, float y) {
  s_st.x = x; s_st.y = y;
  s_st.vx = s_st.vy = 0.0f;
  s_st.valid = true;
  s_st.lastFixMs = millis();
  updateMagnet();
}

void est_predict(float dt, float theta, float ax_body, float ay_body) {
  /* Heading is NOT integrated here. The rotation controller already integrates
     the gyro into `theta` and that path is proven; duplicating it would mean
     two integrators that can silently diverge. We only carry the offset that
     turns platform-relative heading into dock-relative heading.

     theta is CW-positive and psi is CCW-positive -- converted ONCE, here, see
     EST_THETA_SIGN in estimator.h. */
  float th = EST_THETA_SIGN * theta;
  if (s_haveOffset) s_st.psi = wrapPi(th + s_psiOffset);

  /* ---- DEAD RECKONING. The accelerometer arrives in its own axes; rotate by
     s_accRot to get body (forward, left), then by psi to get the dock frame.
     Checked at psi = 0: a pure forward push gives dock (0,-1), i.e. toward the
     wall, and a pure left push gives (-1,0), i.e. -X. */
  /* Low-pass BEFORE integrating, not after. Filtering the position afterwards
     cannot undo a velocity random walk that has already been committed. */
  float k = dt / (EST_ACC_TAU + dt);
  s_faX += k * ((ax_body - s_bX) - s_faX);
  s_faY += k * ((ay_body - s_bY) - s_faY);

  float cr = cosf(s_accRot), sr = sinf(s_accRot);
  float fwd  = s_faX * cr - s_faY * sr;
  float left = s_faX * sr + s_faY * cr;

  float sp = sinf(s_st.psi), cp = cosf(s_st.psi);
  float ax_dock = fwd * (-sp) + left * (-cp);
  float ay_dock = fwd * (-cp) + left * ( sp);

  /* Integrate acceleration into velocity, velocity into position. Only runs
     once there is a pose to integrate FROM -- before est_setPose() or the first
     vision fix there is no origin, and integrating from (0,0) would look like a
     platform sitting on the dock. */
  if (s_st.valid) {
    s_st.vx += ax_dock * dt;
    s_st.vy += ay_dock * dt;
    /* Leak (see EST_V_LEAK_TAU): friction forbids a persistent unforced
       velocity, so bound what bias can accumulate rather than trusting it. */
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

  /* Both gyro quantities arrive CW-positive; the dock frame is CCW-positive.
     Converted once, here (EST_THETA_SIGN, estimator.h). omega_p needs it just
     as much as theta does -- it advances the VISION heading over the frame's
     age below, so a wrong sign there pushes psi_meas the wrong way by
     omega_p*age, which at 140 ms of age and 1 rad/s is ~8 deg per fix. */
  float th = EST_THETA_SIGN * theta;
  float wz = EST_THETA_SIGN * omega_p;

  /* Never apply the same measurement twice -- trap T10. The symptom is an
     overconfident estimate that drifts, which reads as a tuning problem. */
  if (s_haveSeq && p->seq == s_lastSeq) return false;
  s_lastSeq = p->seq;
  s_haveSeq = true;

  if (!(p->flags & PI_FLAG_VALID)) return false;
  if (p->range_m < EST_RANGE_MIN || p->range_m > EST_RANGE_MAX) {
    s_rejects++;
    return false;
  }

  /* ---- LATENCY COMPENSATION. The measurement describes where we were
     age_us ago, not where we are. Ignoring that injects an error proportional
     to speed, which is the trap the guide's Step 5.1 warns about -- and the
     whole reason age_us exists in the Phase 3 payload.

     Propagating the MEASUREMENT forward (rather than rewinding the state) is
     the cheap version and is right to first order at these speeds. */
  float age = (float)p->age_us * 1e-6f;
  if (age > 0.5f) age = 0.5f;              /* refuse to extrapolate absurdly */

  /* Heading first: psi_meas is where vision says we were pointing, advanced by
     the gyro rate over the measurement's age. */
  float psi_meas = wrapPi(p->relyaw_rad + wz * age);

  if (!s_haveOffset) {
    /* First fix: adopt it outright rather than easing in from zero. There is
       nothing to average yet and a slow ramp would just be wrong for seconds. */
    s_psiOffset = wrapPi(psi_meas - th);
    s_haveOffset = true;
    s_st.psi = psi_meas;
  } else {
    /* Slow blend. AMBIGUOUS means the solver could not separate the two planar
       poses, so lean harder on the gyro instead of arguing with it. */
    float g = (p->flags & PI_FLAG_AMBIGUOUS) ? (s_alphaPsi * 0.25f)
                                             : s_alphaPsi;
    s_psiOffset = wrapPi(s_psiOffset +
                         g * wrapPi(psi_meas - wrapPi(th + s_psiOffset)));
    s_st.psi = wrapPi(th + s_psiOffset);
  }

  /* ---- position. Vision gives where the DOCK is relative to the CAMERA; we
     want where the PLATFORM CENTRE is relative to the dock. Uses the FILTERED
     psi, not the raw measured one -- that is the point of the whole design.
     Feeding raw vision yaw in here is what compressed x to ~35% of true,
     because x = range*sin(psi - bearing) collapses when psi tracks bearing. */
  float psi = s_st.psi;
  float cam_x = p->range_m * sinf(psi - p->bearing_rad);
  float cam_y = p->range_m * cosf(psi - p->bearing_rad);
  float mx = cam_x + EST_L_CAM * sinf(psi);
  float my = cam_y + EST_L_CAM * cosf(psi);

  /* Advance the measurement to now using the current velocity estimate. */
  mx += s_st.vx * age;
  my += s_st.vy * age;

  if (!s_st.valid) {
    s_st.x = mx; s_st.y = my;
    s_st.vx = s_st.vy = 0.0f;
    s_st.valid = true;
  } else {
    /* alpha-beta: position corrected directly, velocity inferred from the
       position innovation. dt is time since the LAST fix, not the control
       period -- getting that wrong scales the velocity gain by ~7x here. */
    float dtFix = (float)(millis() - s_st.lastFixMs) * 1e-3f;
    if (dtFix < 0.005f) dtFix = 0.005f;
    if (dtFix > 0.5f)   dtFix = 0.5f;

    /* Snap in progress: accumulate, and once enough frames are in, ADOPT the
       mean rather than blending toward it. */
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
    /* Routine correction only while slow -- above EST_VIS_VMAX vision position
       contributes nothing at all. */
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
