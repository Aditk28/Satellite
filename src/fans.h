#pragma once
#include <Arduino.h>

/*
  Fan subsystem: four ESCs driven with DSHOT300 from TIM1 + DMA2 burst on PA8..PA11.

  Only fanTask (priority 2) writes fan outputs; other tasks request throttle through
  the setters. Requests are snapshotted into fanTask under a short critical section
  so all four values of a frame come from the same request set. ESCs disarm when
  frames stop, so a hung or killed fanTask stops the fans on its own.
*/

/* Per-channel throttle ceiling, percent, applied inside the setters so a runaway
   controller cannot exceed it. The props are unguarded; raise this only with reason. */
#define FAN_THROTTLE_MAX  60.0f

/* Prop current scales roughly with throttle^3, so four channels at the cap (~10 A)
   would blow the 10 A fuse. The commanded set is scaled down together to stay under
   this estimated total. Replace the cube-law estimate with the measured ESC current
   (A4) once its scaling is known. */
#define FAN_CURRENT_BUDGET_A  7.0f
#define FAN_AMPS_AT_50PCT     1.5f   /* per motor, measured */

/* Resend period. A frame goes out when fanTask actually gets the CPU, which can be up
   to a tick after its wake time, so at 2 ms two frames could land closer than the
   60 us a frame needs. 3 ms leaves ~600 us of margin; ESCs only disarm after ~250 ms. */
#define FAN_FRAME_MS      3
#define FAN_ARM_FRAMES    350     /* ~1 s of DSHOT 0 before throttle is accepted */

/* A frame takes 18 bit periods (60 us). A send attempted sooner than this after the
   previous one is skipped. Measured on the TIM5 microsecond timebase. */
#define FRAME_MIN_GAP_US  100

/* Dead-man timeout. If no setter is called for this long while a fan is spinning,
   fanTask zeroes all four. A closed-loop controller refreshes it every cycle, so it
   only fires on an untended manual command. */
#define FAN_CMD_TIMEOUT_MS 10000

/* Channels that get DSHOT command 21 (spin direction reversed) at every boot.
   Bit (ch-1); bit3 = fan 4. This ESC ignores SAVE_SETTINGS (12), so the reversal does
   not persist across power cycles. Set to 0 if the ESC firmware ever honours SAVE. */
#define FAN_REVERSE_MASK   0x08
#define DSHOT_CMD_DIR_REVERSED 21
#define FAN_CFG_REPS       10

/* Configure TIM1, DMA2_Stream5 and PA8..PA11 and create fanTask. Call once from
   setup(), before the scheduler starts. Does not touch TIM2/TIM3, SPI2 or I2C1. */
void fans_init(void);

/* Request throttle on channel 1..4, 0..100 percent, clamped to FAN_THROTTLE_MAX.
   Rejected (and counted) before arming completes or after a hard kill. Any task. */
void fans_setThrottle(int ch, float pct);

/* Request all four at once; the values reach the wire in the same frame. */
void fans_setAll(float f1, float f2, float f3, float f4);

/* Send DSHOT special command cmd (1..47) to one channel reps times (0 -> 10), with the
   other channels held at 0. Returns false if not armed, killed, a burst is already
   pending, or any channel is spinning (ESCs only honour commands while stopped).
   Command frames set the telemetry bit, which is what separates them from throttle.
   Useful commands: 7/8 direction 1/2, 20 normal, 21 reversed, 12 save settings. */
bool fans_sendCommand(int ch, uint16_t cmd, uint16_t reps);

/* Hard kill. Safe from any context, including an ISR or faults_safeStop() with
   interrupts disabled: no FreeRTOS calls, no locks, no blocking. Stops DMA and TIM1,
   clears MOE and drives PA8..PA11 low as GPIO. Latches until fans_rearm(). */
void fans_stopAll(void);

/* Undo a hard kill: redo hardware init and the ~1 s arming ramp. Task context only. */
void fans_rearm(void);

/* Armed and not killed, i.e. frames should be flowing. Used to gate the stall watchdog
   so the arming ramp and a hard kill do not read as a stalled task. */
bool     fans_armedAndLive(void);

bool     fans_armed(void);
bool     fans_killed(void);
uint32_t fans_frames(void);
/* Sends skipped because the previous frame was still inside FRAME_MIN_GAP_US.
   Expected from scheduling jitter; watch the rate, not the count. */
uint32_t fans_skips(void);
/* Transfers still not drained after FRAME_MIN_GAP_US. Should stay 0. */
uint32_t fans_overruns(void);
uint32_t fans_rejects(void);       /* throttle requests refused (not armed / killed) */
uint32_t fans_budgetHits(void);    /* sets scaled down by the current budget */
uint32_t fans_stackFreeWords(void);

/* DMA NDTR and TIM1 CEN captured on overruns. Large NDTR with CEN=0 means the timer
   was not advancing the DMA; small NDTR with CEN=1 means the frame was still clocking. */
void fans_overrunDetail(uint32_t* lastNDTR, uint32_t* minNDTR, uint32_t* maxNDTR,
                        uint32_t* cenWasSet);
float    fans_pct(int ch);         /* last accepted request, percent */
