#pragma once
#include <Arduino.h>

/*
  Telemetry task (priority 1). After telem_activate(), telemTask is the only task
  that writes the serial ports, because HardwareSerial is not reentrant. Text goes
  out through telem_print(); bulk writers use telem_run(fn), which runs fn on
  telemTask so it may use the ports directly.
*/

// Create the queue and telemTask. Call once, pre-scheduler.
void telem_init(Print& usb, Print& bt);

// Switch from direct writes (boot) to queued output. Call once after hwSetup,
// right before the control loop starts blocking.
void telem_activate();

// Queue a line. Never blocks; drops and counts if the queue is full.
void telem_print(const String& s);

// Before activation telem_print() writes from the calling task, which is only safe
// while the control task is the sole writer. Other tasks must check this before
// printing during boot.
bool telem_isActive();

// Run fn() on telemTask. Never blocks; drops and counts if the queue is full.
void telem_run(void (*fn)());

// True while queued work is outstanding (e.g. a capture dump still being written).
bool telem_busy();

uint32_t telem_drops();
uint32_t telem_stackFreeWords();   // telemTask min-ever free stack, words
