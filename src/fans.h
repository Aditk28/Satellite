#pragma once
#include <Arduino.h>

/*
  Phase 1.2 — the fan subsystem (DSHOT300 on TIM1 + DMA burst).

  THE OWNERSHIP RULE, and the whole reason this file exists:
      Only fanTask writes fan outputs. Everything else REQUESTS a throttle.

  Same discipline as telemetry's one-writer and comms' one-reader, but with a
  wrinkle neither of those had: a DSHOT frame is a SET of four values clocked out
  together. Each uint16_t store is atomic on Cortex-M4, but four of them are not
  atomic AS A SET -- a preemption between the ch1 and ch2 writes would put a torn
  allocation on the wire. So requests land in one array and fanTask snapshots them
  into its own under a short critical section (see fans.cpp for the cost).

  THE DRIVER ITSELF was proven standalone first (src/fan_dma_test.cpp, env fandma,
  Step 1.1): 22,278 frames, 0 overruns, all four channels spinning, every register
  verified against the arithmetic. dshotFrame/buildBuffer/dshotHwInit/dshotSendFrame
  are ported from it CHARACTER-FOR-CHARACTER so the proven code and this code stay
  diffable. Do not tidy them.

  PRIORITY 2, alongside safety and comms: below controlTask (3) and focTask (4), so
  fans can never preempt the control law or commutation, and above telemTask (1).
  It shares no mutex, no I2C and no serial with the control path, so the only
  coupling to the 200 Hz loop is the snapshot critical section.

  FAIL-SAFE IS FREE. ESCs disarm when DSHOT frames stop arriving, so a hung, killed
  or starved fanTask means the fans stop by themselves. That is a property of the
  protocol, not something we had to engineer.

  ---------------------------------------------------------------------------
  SAFETY: the props are UNGUARDED and staying that way (decision B7), so the
  mitigations are in firmware. FAN_THROTTLE_MAX is a COMPILED-IN ceiling applied
  inside fans_setThrottle(), which means a runaway control law cannot exceed it
  either -- that is the point of clamping at the setter rather than at the caller.
  Raise it deliberately, when Phase 2 plant ID needs the range.
  ---------------------------------------------------------------------------
*/

#define FAN_THROTTLE_MAX  30.0f   /* percent. Bring-up ceiling -- see B7.      */
/* Resend period. 3 ms, not 2, and the extra millisecond is load-bearing.

   vTaskDelayUntil schedules from the WAKE time, but a frame is emitted when the task
   actually gets the CPU -- and those differ by up to a tick, because controlTask
   (prio 3) holds the CPU ~2.4 ms of every 5 ms for its MPU read and commsTask shares
   priority 2. At a 2 ms period a task released at tick N but running at N+0.95 emits,
   then wakes at N+2 and emits again only ~50 us later -- inside the 60 us the previous
   frame needs to clock out. Measured: NDTR 4..16 of 72 with CEN=1, i.e. the transfer
   was 1-4 bursts from done. At 3 ms the same jitter leaves ~600 us of margin.

   333 Hz is still an order of magnitude above anything the ESC needs; BLHeli_S/Bluejay
   disarm on the order of 250 ms of silence. */
#define FAN_FRAME_MS      3
#define FAN_ARM_FRAMES    350     /* ~1 s of DSHOT 0 before throttle is taken  */

/* A frame needs 18 bit periods = 60 us to clock out. Emitting another inside that
   window would abort it, so we skip instead. Measured against the TIM5 microsecond
   timebase, NOT tick arithmetic -- the whole bug this replaces was tick granularity
   hiding a 50 us reality. 100 us gives margin over the 60 us frame. */
#define FRAME_MIN_GAP_US  100

/* Dead-man timeout on commanded throttle, milliseconds. If nothing calls
   fans_setThrottle()/fans_setAll() for this long while a fan is spinning, fanTask
   zeroes all four and says so.

   WHY: with the props unguarded (B7) a fan left running because the operator got
   distracted is precisely the hazard a guard would have covered. This is the cheapest
   available substitute.

   WHY IT COSTS NOTHING IN CLOSED LOOP: a controller calls fans_setAll() every cycle,
   which refreshes the timer continuously. It can only fire on a MANUAL command that
   nobody is tending -- which is exactly the case it exists for. */
#define FAN_CMD_TIMEOUT_MS 10000

/* Configure TIM1 + DMA2_Stream5 + PA8..PA11, and create fanTask (prio 2).
   Call ONCE, pre-scheduler, from setup(). Touches nothing SimpleFOC owns:
   TIM2/TIM3 (motor PWM), SPI2 (encoder) and I2C1 are all untouched, and DMA1/DMA2
   were previously unused by this project. All four channels start at DSHOT 0. */
void fans_init(void);

/* Request throttle on one channel, 0..100 percent, CLAMPED to FAN_THROTTLE_MAX.
   ch is 1..4. Ignored (and counted in fans_rejects()) before arming completes or
   after a hard kill. Safe to call from any task. */
void fans_setThrottle(int ch, float pct);

/* Request all four at once. Applied as a SET -- the four values reach the wire in
   the same frame, which matters once these come from a thrust allocation. */
void fans_setAll(float f1, float f2, float f3, float f4);

/* HARD KILL. Callable from ANY context including an ISR and faults_safeStop()
   (which runs with interrupts already disabled), so it uses NO FreeRTOS API, takes
   no lock, and never blocks. Stops the DMA, stops the counter, clears MOE, and
   drives PA8..PA11 low as plain GPIO -- deliberately NOT relying on the DMA path,
   which is already dead by the time a fault path calls this.
   LATCHES: nothing restarts until fans_rearm(). */
void fans_stopAll(void);

/* Undo a hard kill: re-run hardware init and repeat the arming ramp (~1 s).
   Task context only. */
void fans_rearm(void);

/* True only while fans SHOULD be clocking out frames: armed and not hard-killed.
   This is what safetyTask's stall watchdog gates on, so that neither the ~1 s arming
   ramp nor a deliberate hard kill reads as a stalled task. */
bool     fans_armedAndLive(void);

/* Diagnostics for G. */
bool     fans_armed(void);
bool     fans_killed(void);
uint32_t fans_frames(void);
/* Frames NOT emitted because the previous one was still clocking out. BENIGN and
   expected: release jitter means two emissions can land microseconds apart even on a
   correct schedule. A skipped frame just means the ESC gets the next one a few ms
   later. Non-zero here is normal; watch the RATE, not the count. */
uint32_t fans_skips(void);

/* Transfers that had still not drained after FRAME_MIN_GAP_US had genuinely elapsed --
   a 60 us frame given 100+ us. Unlike a skip this is a REAL anomaly (timer stopped,
   clock gated, ARR clobbered). This one should be 0. */
uint32_t fans_overruns(void);
uint32_t fans_rejects(void);       /* throttle requests refused (not armed / killed) */
uint32_t fans_stackFreeWords(void);

/* Step 1.4 overrun diagnostic. NDTR observed at the moment a transfer was found
   still in flight, which discriminates the two candidate causes: LARGE means the
   timer was not advancing the DMA (fanTask preempted mid-frame), SMALL (1..3) means
   frame desync from a stale update request. cenWasSet says whether TIM1 was even
   running. Reported by G; delete once the cause is fixed and confirmed. */
void fans_overrunDetail(uint32_t* lastNDTR, uint32_t* minNDTR, uint32_t* maxNDTR,
                        uint32_t* cenWasSet);
float    fans_pct(int ch);         /* last accepted request, percent */
