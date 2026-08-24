#pragma once
#include <Arduino.h>
#include "pi_link.h"

/*
  Translation Phase 5, Step 5.1 -- dock-relative pose estimator.

  Fuses the two sensors by giving each the job it is actually good at:

    GYRO   excellent at heading, drifts slowly (0.8 deg/min measured).
           Already integrated into `theta` by the rotation controller.
    VISION excellent at POSITION -- range matches a tape measure. BAD at yaw
           for a coplanar tag array near square-on, where the two PnP solutions
           are nearly degenerate and the solver flips between them (T30).
           Measured: yaw noise corrupted x badly enough to compress it to ~35%
           of true, because x = range*sin(psi - bearing) and a psi that drifts
           WITH bearing cancels the difference.

  So: vision supplies x and y directly, and supplies heading only as a SLOW
  absolute reference. The gyro supplies heading moment to moment.

  WHY THAT ANSWERS "WHERE DOES THE GYRO START". It never has to start correctly.
  psi = theta + psi_offset, and psi_offset is nudged toward vision on every
  frame, forever. Noise averages down as sqrt(N) -- at 28 fps, +-10 deg of
  per-frame yaw noise becomes under +-1 deg in about three seconds, far better
  than the gyro drifts in minutes. A bad first frame is simply outvoted.

  CAVEAT WORTH KEEPING: noise averages out, BIAS DOES NOT. Vision yaw error that
  correlates with position is bias, and this filter will believe it. Two things
  make it survivable: yaw conditioning is BEST near square-on and close, which is
  the docking configuration, and the `quality` field lets us widen the gate when
  the solver says it cannot separate the two poses. If terminal alignment ever
  turns out to be limited by this, the structural fix is ~2 cm standoffs under
  the flanking tags -- non-coplanar points kill the ambiguity outright.

  NOT DOING ACCELEROMETER PREDICTION YET (5.1b). At 28 Hz the platform barely
  moves between frames, so constant-velocity prediction is adequate. Accel needs
  its body-frame axis signs verified on hardware first (T11), and an unverified
  sign there would be worse than no accel at all.

  FRAME (2D, horizontal plane; the vertical axis is irrelevant):
      origin  tag 0, projected to the table
        +X    right, as you FACE the wall
        +Y    out from the wall, toward the approach side
      psi     platform heading; 0 = facing the wall square-on

  All of x, y, psi refer to the platform's CENTRE OF ROTATION (the wheel axis),
  not the camera and not the magnet -- the camera sits 13.46 cm ahead of it and
  the transform below walks that lever arm, which ROTATES with heading. A 10 deg
  heading error swings the lens 2.3 cm, twice the docking tolerance.
*/

typedef struct {
  float x, y;        /* platform centre in the dock frame, metres            */
  float psi;         /* heading, rad. 0 = square-on to the wall              */
  float vx, vy;      /* dock-frame velocity, m/s                             */
  float mag_x, mag_y;/* docking magnet position -- what actually has to land */
  bool  valid;       /* false until the first vision fix has been applied    */
  uint32_t lastFixMs;
} EstState;

/* Geometry, metres. Measured 2026-08-20. */
#define EST_L_CAM   0.1346f   /* lens, ahead of the platform centre          */
#define EST_L_MAG   0.0946f   /* docking magnet, ahead of the centre         */
/* Lateral offset of the magnet from the camera axis, metres, POSITIVE = to the
   platform's LEFT. The magnet was modelled as sitting purely ahead of the
   centre; measured 2026-08-21 it is 2.3 cm off-axis, which showed up as the
   dock consistently landing left of centre.
   This is a BODY-frame offset and it ROTATES with heading, which is why it
   cannot be a constant subtracted from x -- that would only be right at psi = 0
   and would grow wrong as the platform turned. Trim at runtime with TM<m>. */
#define EST_MAG_LAT   0.023f
#define EST_D_DOCK  0.35f     /* ground magnet, out from the wall            */

/* ---- THE TWO HEADING CONVENTIONS DISAGREE, AND THIS IS WHERE THEY MEET ----
   The rotation controller's `theta` is CLOCKWISE-positive seen from above; the
   dock frame's `psi` is COUNTER-CLOCKWISE-positive. Both are internally
   consistent, and rotation-only control never had to care -- but psi = theta +
   offset is an identity that requires them to turn the SAME way, and they do
   not.

   MEASURED 2026-08-20, twice, by hand-rotating the platform 30 deg CCW:
       theta  -6.19 -> -36.19  (-30.0)      relyaw  4.19 -> 33.30  (+29.1)
       theta   0.00 -> -30.07  (-30.1)      relyaw  4.20 -> 31.05  (+26.9)

   psi is the one that matches the dock frame: translation.cpp puts body-forward
   at (-sin psi, -cos psi), so psi = +90 aims at -X_dock, the observer's left,
   i.e. CCW. So `theta` is the quantity that gets flipped, here, once, rather
   than anywhere downstream (T11) -- and NOT by touching GYRO_SIGN, which would
   invert the tuned and working rotation axis.

   WHY PHASE 5 PASSED ANYWAY: every 5.1 test was static. With theta constant the
   offset absorbs any sign error and psi converges to the vision heading
   regardless, which is why psi tracks relyaw almost exactly in every static G.
   The error only appears while TURNING -- the gyro then contributes heading in
   the wrong direction, the offset has to chase 2x the rotation through a
   0.02-per-frame filter, and psi is wrong for seconds. That is a direct attack
   on the whole point of the T39 split, and it rotates the thrust vector. */
#define EST_THETA_SIGN  (-1.0f)

/* ---- IMU-LED OPERATION (2026-08-20) ---------------------------------------
   Vision position stopped being trustworthy: it reported the platform 55 cm
   off-centre on a 61 cm table, reported four differently-angled fans all
   pushing along dock X with y flat, and reported a coasting platform
   ACCELERATING to 0.57 m/s with the fans off. Range and bearing are fine; the
   derived x/y are not. So position now dead-reckons on the accelerometer from a
   known start pose, and vision is demoted to a slow drift corrector.

   EST_ACC_ROT_DEG is the rotation from the accelerometer's own axes to the body
   frame (CCW from the camera axis).

   VERIFIED 270 ON HARDWARE 2026-08-21. It was reasoned to +90 from the fan
   directions and that was 180 deg wrong -- the same flip the fan table needed,
   which in hindsight is the tell: one inverted convention showing up in two
   places, not two independent errors.

   How the wrong value presented, because it looks nothing like a sign error:
   the platform physically moved the RIGHT way, but dead reckoning integrated it
   backwards, so `err` GREW during every hop. The controller responded by
   pushing harder -- correct behaviour on a lie -- and the divergence guard fired
   on 13 of 15 runs. At rest, vision kept correcting position back to truth, so
   the estimate looked fine every time anyone stopped to check it.
   Swept with TR0/90/180/270 against the real loop once vision-at-rest gave a
   reference worth trusting. Still runtime-settable with TR. */
#define EST_ACC_ROT_DEG  270.0f

void est_init(void);

/* Place the estimate at a known pose and zero its velocity. This is what makes
   dead reckoning usable: put the platform on a marked spot, declare where it
   is, and integrate from there. psi keeps coming from the gyro. */
void est_setPose(float x, float y);

/* Place the estimate so the MAGNET lands exactly here, back-solving the centre
   through both lever arms at the current heading. This is what TI wants: the
   ground truth at dock time is where the magnet is, not where the centre is. */
void est_setMagnetPose(float mag_x, float mag_y);

void  est_setMagLat(float m);
float est_getMagLat(void);

/* Zero-velocity update. Call when the platform is KNOWN to be stationary.
   Double-integrated acceleration drifts as t^2, and the velocity term dominates:
   a 0.02 m/s^2 residual bias is 0.2 m/s of phantom speed after 10 s, which then
   sprays position error at 0.2 m/s forever. Clamping velocity the moment we know
   it is zero removes that entire term, and costs nothing -- it is the one instant
   where the truth is known exactly without a sensor. */
void est_zupt(void);

/* Collect the next N valid vision fixes, average them, and ADOPT that as the
   position outright -- not a blend, a replacement.

   Why averaged rather than a single frame: one fix carries the full per-frame
   noise, and the whole reason to stop is that we intend to believe this number.
   Averaging N frames cuts that by sqrt(N) -- 8 frames at ~10 fps is under a
   second and roughly a 3x improvement -- and it costs nothing, because the
   platform is stationary anyway while it settles.

   Why a replacement rather than a correction: after a hand-move the dead-reckoned
   position may be tens of centimetres out. Blending a good measurement with a
   bad estimate produces something between the two; there is no reason to keep
   any of the dead-reckoned value once a stationary vision fix exists.

   Heading is deliberately untouched -- psi stays gyro-led (T30/T39). */
#define EST_SNAP_FRAMES  8
void est_snapBegin(int nframes);
bool est_snapDone(void);        /* false while still collecting               */
void est_snapCancel(void);      /* abandon a snap that has not completed      */

/* Accelerometer low-pass, rad/s equivalent. Prop vibration puts +-1.2 m/s^2 of
   noise on the raw signal (measured with fans at 60%), and a double integrator
   turns that into a velocity random walk. Translation bandwidth is omega_n ~1.9
   rad/s, so a 1.6 Hz corner removes the vibration while touching nothing the
   controller cares about. */
#define EST_ACC_TAU  0.10f

/* Vision corrects POSITION only when the platform is essentially stationary.
   This is the whole lesson of the last few hours: vision x/y is good at rest
   (tag framed, no blur, no lever-arm dynamics, latency irrelevant) and actively
   destructive while moving -- it reported four differently-angled fans all
   pushing along dock X, and a coasting platform accelerating to 0.57 m/s.
   Gating on speed keeps the good half and discards the bad half, instead of
   trading one against the other with a single blended gain.
   HEADING is untouched by this: psi stays gyro-led with vision as a slow
   absolute reference, exactly as before (T30/T39). */
#define EST_VIS_VMAX  0.020f    /* m/s -- above this, position ignores vision  */

/* Velocity leak time constant, seconds. Bounds bias-driven velocity at
   bias*tau instead of letting it integrate without limit.

   ⚠️ LENGTHENED 2.0 -> 10.0. The leak cannot tell a bias-driven velocity from a
   real one, so at tau = 2 s it was eating genuine motion: applied every 5 ms
   cycle it decays velocity by 0.9975^200 = 39% PER SECOND, so a 3 s hand-slide
   was integrated at 22% of true and the estimate badly under-reported where the
   platform had gone. That is fatal for this demo, where the hand move IS the
   measurement.

   At 10 s the leak costs ~5%/s -- still enough to keep a stuck-at-rest bias from
   running away over tens of seconds, but it no longer competes with real motion.
   The real defence against phantom velocity is the zero-velocity update, which
   sets it to exactly zero whenever the platform is known still. A leak is a
   blunt instrument standing in for a measurement; keep it weak and let the ZUPT
   do the work. */
#define EST_V_LEAK_TAU  10.0f

/* Dock-frame heading offset, so the caller can command the WHEEL to a heading
   that makes psi = 0 (square-on to the dock). */
float est_psiOffset(void);

void  est_setAccRot(float deg);
float est_getAccRot(void);

/* Call every control cycle (200 Hz) with the elapsed time and the rotation
   controller's own integrated heading. Propagates position at constant
   velocity and recomputes psi from theta + the current offset. */
void est_predict(float dt, float theta, float ax_body, float ay_body);

/* Call when a NEW vision frame has arrived (check the seq has changed -- never
   apply the same measurement twice, that is trap T10 and it produces an
   overconfident, drifting estimate). Returns false if the frame was unusable. */
bool est_correct(const PiPose* p, float theta, float omega_p);

bool est_get(EstState* out);

/* Tuning. Defaults are in estimator.cpp with the reasoning for each. */
void  est_setGains(float alphaPos, float betaVel, float alphaPsi);
void  est_getGains(float* alphaPos, float* betaVel, float* alphaPsi);
uint32_t est_fixes(void);      /* vision corrections applied                 */
uint32_t est_rejects(void);    /* frames rejected as unusable                */
