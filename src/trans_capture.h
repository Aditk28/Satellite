#pragma once
#include <Arduino.h>
#include "estimator.h"

/*
  Translation capture -- a recorder for a closed-loop TT move.

  WHY THIS EXISTS. `G` is a snapshot, and a translation move is a trajectory.
  The first closed-loop run ended with err=0.033 m and a=(-0.44,+1.00), from
  which the velocity had to be BACK-CALCULATED to discover the platform was
  still closing at ~0.3 m/s -- i.e. the one number that said whether the move
  had settled was not in the output at all. You cannot tune a controller from
  snapshots taken by hand while watching for a collision.

  WHY A SEPARATE BUFFER instead of more columns on the existing one. Two
  reasons, one arithmetic and one structural.

    ARITHMETIC. The main buffer is 1500 samples at 200 Hz for a 7.5 s rotation
    step. Eight more float columns on it is 48 KB, and only ~43 KB of SRAM is
    free. Translation does not need 200 Hz anyway: the designed settle is 3 s
    and the vision that feeds it arrives at 7-14 Hz, so anything above ~20 Hz
    is recording the estimator's own interpolation. 20 Hz for 20 s is 400
    samples and ~15 KB.

    STRUCTURAL. T22: adding a new capture TYPE to the single-capture-type code
    path produced three separate bugs at once -- a missing telem_busy() guard,
    a post-capture transition into CTRL_HOLD, and stepCount not advancing. A
    move that runs until it converges (or trips, or the operator stops it) does
    not fit the fixed-window state machine those bugs live in. A separate
    buffer with its own start/stop/dump touches none of it.

  WHAT IS NOT STORED, because it is derivable and RAM is the binding
  constraint: magnet position (x,y,psi and EST_L_MAG give it) and the error
  (that plus the target, which is in the metadata line). Both are recomputed in
  the dump, so the CSV still carries them.

  Throttles are stored as uint8 percent. 1% resolution against a 60% ceiling is
  finer than the ESC's own response and saves 4.8 KB.
*/

#define TCAP_MAX      400        /* 20 s at 20 Hz -- the 15 s timeout plus slack */
#define TCAP_DECIM    10         /* every 10th 200 Hz cycle -> 20 Hz             */

/* Arm the recorder. `testNum` must be unique across ALL captures in a session:
   capture_calibration.py builds the filename from "test N: label" and opens it
   "w", so a repeated pair silently overwrites (T23). Pass the same stepCount
   the main capture uses. */
void tcap_start(float target_x, float target_y, int testNum);

/* Call every control cycle while a move is running. Decimates internally. */
void tcap_sample(const EstState* st, uint32_t poseAgeUs);

/* Freeze and record why. `reason` is copied by pointer -- pass a literal. */
void tcap_stop(const char* reason);

bool tcap_active(void);
bool tcap_pending(void);         /* frozen, not yet dumped                      */

/* Writes the CSV. Call ONLY from telemTask via telem_run() -- telemTask is the
   sole serial writer after activation (RTOS invariant B15). Clears pending. */
void tcap_dumpTo(Print& out);
