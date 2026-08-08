#pragma once
#include <Arduino.h>

/*
  Phase 3 — telemetry extraction (Option C, Appendix B7).

  THE INVARIANT, and the whole reason this file exists:
      After telem_activate(), telemTask is the ONLY task that writes the serial
      ports. No exceptions.

  HardwareSerial is not reentrant. Step 3.1 failed because it was half-applied:
  text was queued to telemTask while the capture dump, the HOLD stream, M, E and
  V still wrote directly from the control task. Two tasks in the driver at once
  corrupted it -- the board froze mid-dump and the sensor reads went to garbage.
  A partial single-writer design is worse than none, because it looks correct.

  Two ways to get output out, and between them they cover every writer:
    telem_print(s)  -- queue a line of text (timeout 0; never blocks the caller)
    telem_run(fn)   -- run fn() ON telemTask. Since telemTask owns the ports, fn
                       may use Serial/hc05Serial directly. This is how the bulk
                       writers (capture dump, timing stats) move off the control
                       path without being rewritten.

  telemTask is priority 1 -- below control (3) and foc (4) -- so it only ever
  runs in the CPU the control task leaves idle while blocked on its 200 Hz
  notification. It cannot delay commutation or control. Verified safe in
  isolation before this file did anything (bisect Test A).
*/

// Fixed-size telemetry sample. POD, queued by value (used by the HOLD stream).
struct TelemSample {
  uint32_t t_us;
  float    target_deg, theta_deg, omega_p, omega_w, alpha, u;
};

// Create the queue + telemTask. Call once, pre-scheduler, before the control task.
void telem_init(Print& usb, Print& bt);

// Flip from direct-write (boot) to queued. Call once, after hwSetup, right before
// the control loop starts blocking -- until then telemTask can't drain, so boot
// output must go straight out.
void telem_activate();

// Queue a line. Direct-writes before activation. Never blocks; drops + counts if full.
void telem_print(const String& s);

// Run fn() on telemTask (which owns the ports). Never blocks; drops + counts if full.
void telem_run(void (*fn)());

// True while queued work is outstanding -- used to refuse a new capture while the
// previous dump is still being written out of the capture buffer.
bool telem_busy();

uint32_t telem_drops();
uint32_t telem_stackFreeWords();   // telemTask min-ever free stack, words
