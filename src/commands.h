#pragma once
#include <Arduino.h>

/*
  Phase 6 — the comms task.

  WHAT MOVES: serial RX and line assembly leave the control task and run on
  commsTask (prio 2). WHAT DOES NOT MOVE: command EXECUTION. Complete lines are
  queued and handleCommand() still runs ON THE CONTROL TASK, verbatim.

  Why execution stays put: handleCommand writes controller state (target, ctrlMode,
  gains, capture flags). Running it on commsTask would let those writes land in the
  middle of controlUpdate() -- a torn read of the very state the control law is
  using. Queueing the line instead means the control task applies commands at a
  known point in its cycle, and handleCommand needs no changes at all.

  THE SINGLE-READER RULE (the mirror of telemetry's single-writer rule, B15). Only
  commsTask reads the serial ports. Two readers race on the RX ring buffer's tail,
  and the practical symptom is one of them silently eating the other's bytes -- V's
  stop-key would vanish into commsTask. Hence commands_rxBytes(): anything that
  wants "has the operator pressed something" watches that counter instead of
  calling Serial.available() itself.

  RX/TX split is safe: HardwareSerial keeps separate ring buffers and separate
  head/tail variables per direction, so commsTask reading while telemTask writes
  touches disjoint state. (This already held in Phase 3-5 with the control task
  reading; Phase 6 only changes WHICH task does the reading.)
*/

/* Create the queue + commsTask (prio 2). Call pre-scheduler.
     usb, bt        : the two channels to read.
     emergencyHook  : run IMMEDIATELY on seeing X, from commsTask, before the line
                      is even queued -- typically motor.target = 0. That store is
                      an aligned 32-bit float write (atomic on Cortex-M4), so
                      focTask applies it within one tick (<=250 us) rather than
                      waiting up to a full 5 ms control period. May be NULL. */
void commands_init(Stream& usb, Stream& bt, void (*emergencyHook)(void));

/* Drain one queued command line. Call from the control task at the top of its
   cycle. Returns false when the queue is empty. */
bool commands_next(String& out);

/* Total bytes received on either channel. Monotonic. Use this for "did the
   operator press anything" instead of reading the port -- see the single-reader
   rule above. */
uint32_t commands_rxBytes(void);

/* Lines lost because the queue was full. Silent loss is worse than reported loss. */
uint32_t commands_drops(void);

uint32_t commands_stackFreeWords(void);
