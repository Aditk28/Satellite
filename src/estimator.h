#pragma once
#include <Arduino.h>
#include "pi_link.h"

/*
  Dock-relative pose estimator.

  Position dead-reckons on the accelerometer and is corrected by vision only
  while the platform is at rest. Heading comes from the rotation controller's
  gyro-integrated theta plus an offset that is slowly pulled toward vision yaw,
  so per-frame yaw noise averages out while the gyro carries fast changes.

  Dock frame (2D, horizontal plane):
      origin  tag 0, projected to the table
      +X      right, as you face the wall
      +Y      out from the wall, toward the approach side
      psi     platform heading, CCW-positive; 0 = facing the wall square-on

  x, y, psi refer to the platform's centre of rotation (the wheel axis). The
  camera and magnet sit on lever arms ahead of it that rotate with heading.
*/

typedef struct {
  float x, y;         /* platform centre in the dock frame, m        */
  float psi;          /* heading, rad                                */
  float vx, vy;       /* dock-frame velocity, m/s                    */
  float mag_x, mag_y; /* docking magnet position, m                  */
  bool  valid;        /* false until a pose has been set or fixed    */
  uint32_t lastFixMs;
} EstState;

/* Geometry, metres. */
#define EST_L_CAM   0.1346f   /* lens, ahead of the platform centre          */
#define EST_L_MAG   0.0946f   /* docking magnet, ahead of the centre         */
/* Lateral offset of the magnet from the camera axis, positive = platform's
   left. Body-frame, so it rotates with heading. Runtime trim: TM<m>. */
#define EST_MAG_LAT   0.023f
#define EST_D_DOCK  0.35f     /* ground magnet, out from the wall            */

/* The rotation controller's theta is CW-positive seen from above; psi is
   CCW-positive. theta is flipped once, here, rather than changing GYRO_SIGN,
   which would invert the tuned rotation axis. */
#define EST_THETA_SIGN  (-1.0f)

/* Rotation from the accelerometer's axes to the body frame, deg CCW from the
   camera axis. Verified on hardware. Runtime-settable with TR. */
#define EST_ACC_ROT_DEG  270.0f

void est_init(void);

/* Place the estimate so the magnet lands at (mag_x, mag_y), back-solving the
   centre through both lever arms at the current heading. Zeroes velocity. */
void est_setMagnetPose(float mag_x, float mag_y);

void  est_setMagLat(float m);
float est_getMagLat(void);

/* Zero-velocity update. Call only when the platform is known to be stationary.
   Also folds the current filtered accel reading into the bias estimate. */
void est_zupt(void);

/* Average the next N valid vision fixes and replace the position with the mean
   (not a blend). Heading is untouched. */
#define EST_SNAP_FRAMES  8
void est_snapBegin(int nframes);
bool est_snapDone(void);        /* false while still collecting               */
void est_snapCancel(void);

/* Accelerometer low-pass time constant, s (~1.6 Hz corner). Removes prop
   vibration, well above the ~1.9 rad/s translation bandwidth. */
#define EST_ACC_TAU  0.10f

/* Vision corrects position only below this speed; vision x/y is reliable at
   rest and not while moving. */
#define EST_VIS_VMAX  0.020f    /* m/s */

/* Velocity leak time constant, s. Bounds bias-driven velocity without eating
   much real motion (~5%/s); ZUPT is the main defence against phantom speed. */
#define EST_V_LEAK_TAU  10.0f

/* Dock-frame heading offset (psi = theta' + offset), so the caller can command
   the wheel to a heading that makes psi = 0. */
float est_psiOffset(void);

void  est_setAccRot(float deg);
float est_getAccRot(void);

/* Call every control cycle (200 Hz) with the rotation controller's theta and
   the body-frame accelerometer reading. */
void est_predict(float dt, float theta, float ax_body, float ay_body);

/* Apply a vision frame. Repeated seq numbers are ignored. Returns false if the
   frame was a repeat or unusable. */
bool est_correct(const PiPose* p, float theta, float omega_p);

bool est_get(EstState* out);

void  est_setGains(float alphaPos, float betaVel, float alphaPsi);
void  est_getGains(float* alphaPos, float* betaVel, float* alphaPsi);
uint32_t est_fixes(void);      /* vision corrections applied                 */
uint32_t est_rejects(void);    /* frames rejected by the range gate          */
