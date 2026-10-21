#pragma once
#include <Arduino.h>
#include "estimator.h"

/*
  Recorder for a closed-loop TT move or a TC single-fan probe.

  Separate from the main 200 Hz capture buffer to save RAM and to keep this
  variable-length run out of the fixed-window capture state machine. Samples at
  20 Hz; throttles are stored as uint8 percent. Magnet position and error are
  not stored but recomputed in the dump.
*/

#define TCAP_MAX      400        /* 20 s at 20 Hz                     */
#define TCAP_DECIM    10         /* every 10th 200 Hz cycle -> 20 Hz  */

/* Arm the recorder. testNum must be unique across all captures in a session,
   because capture_calibration.py names the output file from it. */
void tcap_start(float target_x, float target_y, int testNum);

/* Arm for a TC probe, so the capture label (and filename) names the fan and
   throttle. */
void tcap_startProbe(int fan, float pct, int testNum);

/* Call every control cycle while a move runs; decimates internally. Throttles
   come from trans_lastCommand(). imu_ax/imu_ay are the bias-corrected
   body-frame accelerometer and omega_p is the gyro rate. */
void tcap_sample(const EstState* st, uint32_t poseAgeUs,
                 float imu_ax, float imu_ay, float omega_p);

/* Same, for a TC probe where the controller is not driving: throttles are
   passed in because trans_lastCommand() would be stale. */
void tcap_sampleProbe(const EstState* st, uint32_t poseAgeUs, const float pct[4],
                      float imu_ax, float imu_ay, float omega_p);

/* Freeze and record why. reason is stored by pointer, so pass a literal. */
void tcap_stop(const char* reason);

bool tcap_active(void);
bool tcap_pending(void);         /* frozen, not yet dumped */

/* Returns true once per frozen capture, for the caller that enqueues the dump.
   tcap_pending() stays true for the whole dump, so use it only as a guard
   against starting a new run. */
bool tcap_takePending(void);

/* Writes the CSV and clears pending. Call only from telemTask (via
   telem_run()), the sole serial writer after activation. */
void tcap_dumpTo(Print& out);
