#pragma once
#include <Arduino.h>

/*
  Comms task (priority 2): reads the serial ports and assembles command lines.
  Lines are queued and executed by handleCommand() on the control task, so
  controller state is never changed in the middle of controlUpdate().

  commsTask is the only reader of the serial ports; two readers would race on the
  RX buffer and eat each other's bytes. Code that needs "did the operator press
  something" should watch commands_rxBytes() instead of calling available().
*/

/* Create the queue and commsTask. Call pre-scheduler. emergencyHook runs on
   commsTask as soon as an X line arrives, before it is queued (typically
   motor.target = 0, an atomic float store). May be NULL. */
void commands_init(Stream& usb, Stream& bt, void (*emergencyHook)(void));

/* Function called once per poll after the operator ports are read (used for the
   Pi pose link). Runs on commsTask and must not block. */
void commands_setAuxPoll(void (*fn)(void));

/* Pop one queued command line. Call from the control task. Returns false when empty. */
bool commands_next(String& out);

/* Total bytes received on either port. Monotonic. */
uint32_t commands_rxBytes(void);

/* Lines dropped because the queue was full. */
uint32_t commands_drops(void);

uint32_t commands_stackFreeWords(void);
