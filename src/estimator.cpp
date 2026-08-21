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
static float s_alphaPos = 0.35f;
static float s_betaVel  = 0.08f;
static float s_alphaPsi = 0.02f;

/* Reject a fix whose range is outside anything physically reachable on this
   table, or whose yaw the solver admits it cannot resolve. Cheap gate; the
   estimator has no way to recover from swallowing a wild outlier. */
#define EST_RANGE_MIN   0.05f
#define EST_RANGE_MAX   2.50f

static EstState  s_st;
static float     s_psiOffset = 0.0f;   /* psi = theta + this                 */
static bool      s_haveOffset = false;
static uint16_t  s_lastSeq = 0;
static bool      s_haveSeq = false;
static uint32_t  s_fixes = 0, s_rejects = 0;

static inline float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

void est_init(void) {
  s_st = (EstState){0};
  s_psiOffset = 0.0f;
  s_haveOffset = false;
  s_haveSeq = false;
  s_fixes = s_rejects = 0;
}

/* Magnet position follows from the centre and the heading -- recomputed rather
   than filtered separately, so the two can never disagree. */
static void updateMagnet(void) {
  s_st.mag_x = s_st.x - EST_L_MAG * sinf(s_st.psi);
  s_st.mag_y = s_st.y - EST_L_MAG * cosf(s_st.psi);
}

void est_predict(float dt, float theta) {
  /* Heading is NOT integrated here. The rotation controller already integrates
     the gyro into `theta` and that path is proven; duplicating it would mean
     two integrators that can silently diverge. We only carry the offset that
     turns platform-relative heading into dock-relative heading.

     theta is CW-positive and psi is CCW-positive -- converted ONCE, here, see
     EST_THETA_SIGN in estimator.h. */
  float th = EST_THETA_SIGN * theta;
  if (s_haveOffset) s_st.psi = wrapPi(th + s_psiOffset);

  /* Constant-velocity propagation. Between vision frames (~36 ms) this is
     plenty; accelerometer prediction is 5.1b and needs its axis signs checked
     on hardware before it can be trusted (T11). */
  s_st.x += s_st.vx * dt;
  s_st.y += s_st.vy * dt;
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

    float ix = mx - s_st.x, iy = my - s_st.y;
    s_st.x  += s_alphaPos * ix;
    s_st.y  += s_alphaPos * iy;
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
