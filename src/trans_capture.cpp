#include "trans_capture.h"
#include "translation.h"
#include <math.h>

static uint32_t s_t[TCAP_MAX];
static float    s_x[TCAP_MAX], s_y[TCAP_MAX], s_psi[TCAP_MAX];
static float    s_vx[TCAP_MAX], s_vy[TCAP_MAX];
static float    s_ax[TCAP_MAX], s_ay[TCAP_MAX];
static float    s_ix[TCAP_MAX], s_iy[TCAP_MAX], s_wp[TCAP_MAX];
static uint8_t  s_p1[TCAP_MAX], s_p2[TCAP_MAX], s_p3[TCAP_MAX], s_p4[TCAP_MAX];
static uint16_t s_age[TCAP_MAX];

static int         s_n       = 0;
static int         s_div     = 0;
static bool        s_active  = false;
static bool        s_pending = false;   /* frozen, dump not yet finished        */
static bool        s_queued  = false;   /* already handed to telemTask          */
static float       s_tx = 0.0f, s_ty = 0.0f;
static int         s_test = 0;
static const char* s_reason = "unknown";
static int         s_probeFan = 0;      /* 0 = a normal TT move, 1-4 = TC probe */
static float       s_probePct = 0.0f;

void tcap_start(float tx, float ty, int testNum) {
  s_n = 0;
  s_div = 0;
  s_tx = tx;
  s_ty = ty;
  s_test = testNum;
  s_reason = "running";
  s_active = true;
  s_pending = false;
  s_queued = false;
  s_probeFan = 0;
  s_probePct = 0.0f;
}

void tcap_startProbe(int fan, float pct, int testNum) {
  tcap_start(0.0f, 0.0f, testNum);
  s_probeFan = fan;
  s_probePct = pct;
}

static void tcap_store(const EstState* st, uint32_t poseAgeUs,
                       float ax, float ay,
                       float p1, float p2, float p3, float p4,
                       float ix, float iy, float wp) {
  s_t[s_n]   = micros();
  s_x[s_n]   = st->x;
  s_y[s_n]   = st->y;
  s_psi[s_n] = st->psi;
  s_vx[s_n]  = st->vx;
  s_vy[s_n]  = st->vy;
  s_ax[s_n]  = ax;
  s_ay[s_n]  = ay;
  s_ix[s_n]  = ix;
  s_iy[s_n]  = iy;
  s_wp[s_n]  = wp;
  s_p1[s_n]  = (uint8_t)(p1 + 0.5f);
  s_p2[s_n]  = (uint8_t)(p2 + 0.5f);
  s_p3[s_n]  = (uint8_t)(p3 + 0.5f);
  s_p4[s_n]  = (uint8_t)(p4 + 0.5f);
  /* UINT32_MAX means "never had a fix"; clamp so the column stays numeric. */
  uint32_t ms = (poseAgeUs == UINT32_MAX) ? 65535u : (poseAgeUs / 1000u);
  s_age[s_n] = (ms > 65535u) ? 65535u : (uint16_t)ms;
  s_n++;
}

/* Shared front end: decimate, and stop cleanly when the buffer fills rather
   than wrapping -- a wrapped buffer of a run still in progress is a trajectory
   with no beginning, which is the half that matters. */
static bool tcap_tick(void) {
  if (!s_active) return false;
  if (++s_div < TCAP_DECIM) return false;
  s_div = 0;
  if (s_n >= TCAP_MAX) { tcap_stop("buffer_full"); return false; }
  return true;
}

void tcap_sample(const EstState* st, uint32_t poseAgeUs,
                 float imu_ax, float imu_ay, float omega_p) {
  if (!st || !tcap_tick()) return;
  float p1, p2, p3, p4, ax, ay;
  trans_lastCommand(&ax, &ay, &p1, &p2, &p3, &p4);
  tcap_store(st, poseAgeUs, ax, ay, p1, p2, p3, p4, imu_ax, imu_ay, omega_p);
}

void tcap_sampleProbe(const EstState* st, uint32_t poseAgeUs, const float pct[4],
                      float imu_ax, float imu_ay, float omega_p) {
  if (!st || !pct || !tcap_tick()) return;
  /* No commanded acceleration exists during a probe -- the fans are being
     driven open loop -- so those columns are honestly zero rather than stale. */
  tcap_store(st, poseAgeUs, 0.0f, 0.0f, pct[0], pct[1], pct[2], pct[3],
             imu_ax, imu_ay, omega_p);
}

void tcap_stop(const char* reason) {
  if (!s_active) return;
  s_active = false;
  s_pending = (s_n > 0);
  s_queued = false;
  s_reason = reason ? reason : "unknown";
}

bool tcap_active(void)  { return s_active; }
bool tcap_pending(void) { return s_pending; }

bool tcap_takePending(void) {
  if (!s_pending || s_queued) return false;
  s_queued = true;
  return true;
}

void tcap_dumpTo(Print& out) {
  float kp, kd, ff;
  trans_getGains(&kp, &kd, &ff);

  /* Same marker protocol as dumpCaptureTo -- capture_calibration.py keys on
     this line and on the "t_us," header, never on the metadata (a regex over
     metadata silently lost a full sweep once, CONTROL_README section 16). */
  out.print("--- capture start (test ");
  out.print(s_test);
  out.print("/");
  out.print(s_test);
  if (s_probeFan) {
    out.print(": fan probe fan");
    out.print(s_probeFan);
    out.print(" at ");
    out.print(s_probePct, 0);
    out.println(" pct) ---");
  } else {
    out.print(": translate to ");
    out.print(s_tx, 3);
    out.print(" ");
    out.print(s_ty, 3);
    out.println(" m) ---");
  }

  out.print(s_probeFan ? "mode=fan_probe" : "mode=translate");
  out.print(" probe_fan=");  out.print(s_probeFan);
  out.print(" probe_pct=");  out.print(s_probePct, 1);
  out.print(" target_x=");   out.print(s_tx, 4);
  out.print(" target_y=");   out.print(s_ty, 4);
  out.print(" trans_kp=");   out.print(kp, 3);
  out.print(" trans_kd=");   out.print(kd, 3);
  out.print(" trans_ff=");   out.print(ff, 3);
  out.print(" idle_pct=");   out.print(TRANS_IDLE_PCT, 1);
  out.print(" deadzone_m="); out.print(TRANS_DEADZONE, 4);
  out.print(" v_moving=");   out.print(TRANS_V_MOVING, 4);
  out.print(" K_A=");        out.print(TRANS_K_A, 6);
  out.print(" A_c=");        out.print(TRANS_A_COULOMB, 4);
  out.print(" l_mag=");      out.print(EST_L_MAG, 4);
  out.print(" rate_hz=");    out.print(200 / TCAP_DECIM);
  out.print(" stop_reason="); out.println(s_reason);

  out.println("t_us,x,y,psi_deg,vx,vy,magx,magy,errm,ax_cmd,ay_cmd,"
              "ax_imu,ay_imu,omega_p,pct1,pct2,pct3,pct4,poseage_ms");

  for (int i = 0; i < s_n; i++) {
    /* Recomputed here rather than stored -- see the header. The magnet arm
       rotates with heading, which is the whole reason it cannot be a constant
       offset applied in analysis. */
    float mx = s_x[i] - EST_L_MAG * sinf(s_psi[i]);
    float my = s_y[i] - EST_L_MAG * cosf(s_psi[i]);
    float ex = s_tx - mx, ey = s_ty - my;

    out.print(s_t[i]);                       out.print(",");
    out.print(s_x[i], 4);                    out.print(",");
    out.print(s_y[i], 4);                    out.print(",");
    out.print(degrees(s_psi[i]), 2);         out.print(",");
    out.print(s_vx[i], 4);                   out.print(",");
    out.print(s_vy[i], 4);                   out.print(",");
    out.print(mx, 4);                        out.print(",");
    out.print(my, 4);                        out.print(",");
    out.print(sqrtf(ex * ex + ey * ey), 4);  out.print(",");
    out.print(s_ax[i], 3);                   out.print(",");
    out.print(s_ay[i], 3);                   out.print(",");
    out.print(s_ix[i], 4);                   out.print(",");
    out.print(s_iy[i], 4);                   out.print(",");
    out.print(s_wp[i], 4);                   out.print(",");
    out.print(s_p1[i]);                      out.print(",");
    out.print(s_p2[i]);                      out.print(",");
    out.print(s_p3[i]);                      out.print(",");
    out.print(s_p4[i]);                      out.print(",");
    out.println(s_age[i]);
  }
  out.println("--- capture end ---");
  s_pending = false;
  s_queued  = false;
}
