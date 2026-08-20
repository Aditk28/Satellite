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
#define EST_D_DOCK  0.35f     /* ground magnet, out from the wall            */

void est_init(void);

/* Call every control cycle (200 Hz) with the elapsed time and the rotation
   controller's own integrated heading. Propagates position at constant
   velocity and recomputes psi from theta + the current offset. */
void est_predict(float dt, float theta);

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
