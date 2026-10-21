#pragma once
#include <Arduino.h>

/*
  Safety task: an independent 20 Hz watchdog at priority 2 (above telemetry,
  below control and FOC). It backs up the inline 200 Hz wheel-saturation check in
  the control law and is the only thing that can notice the control task has
  stopped running. It does nothing until safety_arm() is called after hwSetup.
*/

/* Create safetyTask (prio 2, 50 ms period). Call pre-scheduler.
   wheelVel is read straight from the FOC layer so a dead control task cannot
   freeze the value the watchdog relies on. safeStopFn is the recoverable stop
   (operator sends R) and may be NULL. A heartbeat failure always goes to
   faults_safeStop(FAULT_HEARTBEAT), since a dead control task cannot be recovered. */
void safety_init(float (*wheelVel)(void), float satLimit,
                 void (*safeStopFn)(const char*));

/* Start checking. Call once, after hwSetup. */
void safety_arm(void);

/* Heartbeat. Call from controlStep() and from any control-task loop that runs
   longer than the heartbeat timeout. */
void safety_kick(void);

/* INA219 power monitoring. readPower takes the I2C mutex itself and returns false
   if the read failed or the sensor is absent. Records min/max only. */
void safety_setPowerMonitor(bool (*readPower)(float* busV, float* mA));

/* Arm the power trips (0 disables either one). A trip needs 2 consecutive checks
   and uses the recoverable stop, so a sagging battery can be swapped without a reset. */
void safety_setPowerLimits(float minBusV, float maxMilliAmps);

/* fanTask liveness. fanFrames is the DSHOT frame counter, fanRunning is true only
   while frames should be flowing. A stall uses the recoverable stop because the ESCs
   disarm on their own when frames stop. Pass NULL for either to disable. */
void safety_setFanMonitor(uint32_t (*fanFrames)(void), bool (*fanRunning)(void));

/* Diagnostics for G. */
uint32_t safety_checks(void);
uint32_t safety_stackFreeWords(void);
bool     safety_powerStats(float* minV, float* maxV, float* maxA, uint32_t* fails);
