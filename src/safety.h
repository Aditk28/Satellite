#pragma once
#include <Arduino.h>

/*
  Phase 5 — the safety task.

  WHY THIS EXISTS. The WHEEL_SAT abort currently lives inside controlUpdate(), so
  it only fires if the control task is running. That is circular: the check that
  saves you depends on the thing that might have failed. safetyTask is an
  INDEPENDENT watchdog at priority 2 -- it keeps checking even if the control task
  is wedged, and it is the only thing that can notice the control task is wedged.

  DEFENCE IN DEPTH, not replacement. The inline 200 Hz WHEEL_SAT check STAYS in
  the control law (guide Step 5.1 Trap): this task samples at 20 Hz, which is up
  to 50 ms of extra exposure, and at this wheel's accelerations that matters. The
  two catch different failures -- inline is fast, this one covers "control stopped
  running at all".

  PRIORITY 2 is deliberate: above telemTask (1) so telemetry can never delay a
  safety check, below controlTask (3) and focTask (4) so safety can never delay
  the control law or commutation. It runs in the gaps, like telemetry.

  THE HEARTBEAT, and the trap it has to survive. A naive "counter must advance"
  watchdog false-trips every time the control task legitimately blocks: B (gyro
  bias, ~1 s), E (3 s), V (until keypress), and hwSetup itself (initFOC spins the
  wheel for hundreds of ms). So the signal is an EXPLICIT safety_kick() meaning
  "the control task is alive and making progress" -- called from controlStep() and
  from every long-running loop -- and the watchdog does not run at all until
  safety_arm() is called at the end of hwSetup.
*/

/* Create safetyTask (prio 2, 50 ms period). Call pre-scheduler.
     wheelVel   : returns the current wheel speed, rad/s. Read directly from the
                  FOC layer (focTask keeps it fresh) rather than from a value the
                  control task publishes -- a dead control task must not be able
                  to freeze the reading the watchdog relies on.
     satLimit   : |wheelVel| above this triggers a stop, rad/s.
     safeStopFn : recoverable stop (motor off, operator sends R). May be NULL.
   Heartbeat failure does NOT use safeStopFn -- a dead control task cannot be
   recovered by clearing flags it will never read, so that path goes to
   faults_safeStop(FAULT_HEARTBEAT): hardware kill + latched reason + LED. */
void safety_init(float (*wheelVel)(void), float satLimit,
                 void (*safeStopFn)(const char*));

/* Start checking. Call once, after hwSetup, alongside telem_activate(). */
void safety_arm(void);

/* "The control task is alive." Call from controlStep() and from any loop in the
   control task that runs longer than the heartbeat timeout. */
void safety_kick(void);

/* Phase 5.2 — power monitoring via INA219, read on the safety task under the I2C
   mutex (the second bus user, and the reason the mutex exists).
     readPower : fills busV / mA, returns false if the read failed or the sensor is
                 absent. Provided by the sketch so safety.* stays sensor-free.
   MONITOR ONLY for now: min voltage / max current are recorded and reported, but
   nothing trips. Thresholds are set from MEASURED values in a follow-up rather than
   guessed -- a wrong guess means nuisance safe-stops on a spinning flywheel. */
void safety_setPowerMonitor(bool (*readPower)(float* busV, float* mA));

/* Arm the power trips. Thresholds are chosen from MEASURED values, not guessed --
   see the Step 5.2 result note. Both are DEBOUNCED (must hold for 2 consecutive
   50 ms checks) so a single noisy sample can never kill a run. Passing 0 for
   either disables that trip. Trips use the recoverable stop, not faults_safeStop:
   a sagging battery should leave the operator able to swap it and send R, not
   force a reset. */
void safety_setPowerLimits(float minBusV, float maxMilliAmps);

/* Diagnostics for G. */
uint32_t safety_checks(void);      /* safety task iterations completed */
uint32_t safety_stackFreeWords(void);
bool     safety_powerStats(float* minV, float* maxV, float* maxA, uint32_t* fails);
