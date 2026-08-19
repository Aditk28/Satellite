#pragma once
#include <Arduino.h>

/*
  Translation Phase 3 -- the Raspberry Pi pose link. USART6, PC6 (TX) / PC7 (RX).

  WHY THIS IS NOT A THIRD ARGUMENT TO commands_init().
  commsTask's contract is line assembly plus "any unrecognised input stops the
  motor", with an X fast path that fires BEFORE the line is even queued
  (commands.cpp). This port carries a BINARY stream, so: a payload byte of 0x58
  is 'X' and would stop the wheel mid-slew, a 0x0A mid-frame is a line
  terminator, and Pi boot chatter arrives as a scatter of commands. The operator
  parser and the pose parser must never share a code path. Decision B18.

  THE SERIAL INVARIANTS ARE PER PORT, NOT GLOBAL. B15 (sole writer) and the
  sole-reader rule are stated over Serial and hc05Serial, but what they protect
  is that HardwareSerial is not reentrant PER PORT. USART6 is touched by nothing
  else, so pi_link is both its sole reader and its sole writer -- the same
  requirement met by ownership instead of arbitration.

  WHERE IT RUNS. pi_poll() is called from commsTask through
  commands_setAuxPoll(), so the link costs no new task and no new stack. Note
  that commsTask polls every POLL_PERIOD_MS UNCONDITIONALLY -- it does not wait
  for bytes -- which is why the staleness ladder below can live here and still
  be evaluated when the Pi is dead. That is the whole point of a timeout.

  ---------------------------------------------------------------------------
  THE WIRE FORMAT                                       (decisions B19/B20)

      [0xA5][0x5A][len][ ---- payload, len bytes ---- ][crc_lo][crc_hi]

  len       payload bytes ONLY (not magic, not CRC). Must equal
            PI_PAYLOAD_LEN or the frame is rejected -- a free version check.
  crc       CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, over len + payload.
            Magic excluded. Transmitted LITTLE-ENDIAN (low byte first).
  endian    little-endian throughout. ARM on both ends, so no swapping. This is
            an ASSUMPTION, stated here because a silent mismatch rejects every
            frame and looks exactly like a wiring fault.

  Payload, 28 bytes, 4-byte aligned by construction (the 3 pad bytes exist to
  keep it that way and to leave room to grow without a format bump):

     off  type      field         meaning
      0   uint16    seq           increments per TX frame; gaps = dropped frames
      2   uint8     flags         see PI_FLAG_* below
      3   uint8     tag_id        primary tag in view
      4   float32   range_m       camera -> tag centre
      8   float32   bearing_rad   tag centre off camera boresight
     12   float32   relyaw_rad    tag face normal vs boresight
     16   float32   quality       0..1, from the PnP ambiguity ratio
     20   uint32    age_us        capture -> transmit, on the PI's clock
     24   uint8     n_tags        tags contributing to this fix
     25   uint8[3]  pad

  WHY POLAR (range/bearing) AND NOT CARTESIAN. Identical information, different
  NOISE. AprilTag range error grows roughly with distance squared while bearing
  error is roughly constant in angle; in polar those are axis-aligned, so the
  estimator's measurement covariance R is DIAGONAL. Converting to Cartesian on
  the Pi produces the classic banana-shaped correlated uncertainty, which needs
  a full covariance to represent honestly -- and sending it as diagonal anyway
  would make the filter quietly overconfident.

  WHY AGE AND NOT A TIMESTAMP (this DEPARTS from the guide's original text).
  The two clocks are unsynchronised, so an absolute Pi timestamp means nothing
  here without a sync protocol to build and keep correct. age_us is measured
  entirely on the Pi's own clock (capture -> transmit), where the difference is
  valid regardless of offset. The STM32 adds the transit it already measured.
  Note CAPTURE, not detection-complete: AprilTag on a Pi 3B+ carries 50-150 ms,
  which at docking speeds is the DOMINANT position error, not a rounding one.

  WHY relyaw HAS A QUALITY FIELD. Relative yaw from a planar target is
  ill-conditioned at range -- two orientations project near-identically and the
  solver flips between them. quality + PI_FLAG_AMBIGUOUS is what lets the
  estimator WIDEN R on yaw instead of believing a flipped solution. See T30.
  ---------------------------------------------------------------------------
*/

#define PI_MAGIC1        0xA5
#define PI_MAGIC2        0x5A
#define PI_PAYLOAD_LEN   28
#define PI_MAX_PAYLOAD   64      /* parser sanity bound, not the format */

#define PI_FLAG_VALID     0x01   /* a tag was actually detected this frame   */
#define PI_FLAG_AMBIGUOUS 0x02   /* PnP could not separate the two solutions */
#define PI_FLAG_MULTITAG  0x04   /* fix used >1 tag -- yaw is trustworthy    */
#define PI_FLAG_OOP       0x08   /* out-of-plane pitch/roll large -> suspect */

/* Staleness ladder. Thresholds and their reasoning are in pi_link.cpp.
   NEVER is distinct from DEAD on purpose: if pose has never been valid, its
   absence is not a loss, and nothing should fire. Without that, every rotation
   test would trip a timeout at boot simply because no Pi is attached. */
typedef enum {
  PI_NEVER = 0,   /* no valid frame since boot -- ladder inert               */
  PI_FRESH,       /* < PI_STALE_MS                                           */
  PI_STALE,       /* pose marked invalid, estimator coasts                   */
  PI_LOST,        /* + fans zeroed: translation dead-reckoning has run out   */
  PI_DEAD         /* + terminal hook fired                                   */
} PiState;

typedef struct {
  uint16_t seq;
  uint8_t  flags;
  uint8_t  tag_id;
  uint8_t  n_tags;
  float    range_m;
  float    bearing_rad;
  float    relyaw_rad;
  float    quality;
  uint32_t age_us;     /* Pi-side capture -> transmit                        */
  uint32_t rx_us;      /* us_now() when the frame finished arriving here     */
} PiPose;

/* Open the port. Call pre-scheduler from setup(), beside commands_init().
   Does NOT print: it runs before telem_activate(), where telem_print() writes
   the port directly from the CALLING task -- trap T18. */
void pi_init(HardwareSerial& port, uint32_t baud);

/* Drain RX, parse frames, evaluate the staleness ladder. Called from commsTask
   every POLL_PERIOD_MS. Never blocks. */
void pi_poll(void);

/* Copy the most recent pose. Returns false if none has ever arrived. Safe from
   any task: the copy is taken under a critical section because a PiPose is many
   words and a preemption mid-copy yields a torn mix of two frames. Cost is
   ~40 words at 180 MHz, well under 1 us against the 250 us FOC tick -- the same
   trade fans.cpp accepts for its dshotValue[] snapshot, priced rather than
   waved away. */
bool pi_getPose(PiPose* out);

PiState  pi_state(void);
uint32_t pi_ageUs(void);        /* since last VALID frame; UINT32_MAX if never */

/* Terminal action for PI_DEAD. NOT wired to stopMotor() in Phase 3, by
   decision B21: nothing consumes pose until Phase 6, so a lost link endangers
   nothing today -- while dumping a spinning flywheel DOES spin the platform at
   ~42 rad/s^2 against a 4.24 breakaway. Phase 6 points this at the real stop
   once there is something to protect. Contract: called from commsTask (prio 2),
   must not block, fires ONCE per loss episode. */
void pi_setDeadHook(void (*fn)(void));

/* Diagnostics for G. crcErrors and resyncs are allowed to be non-zero on a
   noisy link; frames not climbing while rxBytes does means the format or the
   CRC disagree between the two ends. Split benign from real -- trap T21. */
uint32_t pi_rxBytes(void);
uint32_t pi_frames(void);       /* accepted frames                            */
uint32_t pi_crcErrors(void);
uint32_t pi_badLen(void);       /* len field not PI_PAYLOAD_LEN               */
uint32_t pi_resyncs(void);      /* magic desync -- hunted for a new frame     */
/* Sequence numbers that never ARRIVED. NOTE THAT THIS INCLUDES CRC-REJECTED
   FRAMES, and that is deliberate, not a conflation: a rejected frame never had
   its seq decoded, so at the sequence layer it is indistinguishable from one
   that was never sent. More importantly it is the right primitive for the
   Phase 5 estimator, which needs "how many measurements am I missing" in order
   to widen its covariance -- and the answer to that does not depend on whether
   the frame was lost or corrupted.

   The two ARE separable when diagnosing, by subtraction:
       true transport loss  =  pi_seqGaps() - pi_crcErrors()
   Do not "fix" this into non-overlapping counters. It cannot be done reliably:
   a CRC failure cannot be attributed to a specific missing seq (its seq was
   never decoded), so the subtraction would have to ASSUME one gap per CRC
   error, which stops being true the moment a corrupt length field desyncs the
   parser and eats the following frame as well. */
uint32_t pi_seqGaps(void);

/* Sequence went BACKWARDS or jumped absurdly far -- the sender restarted (or,
   far less likely, a garbage frame passed CRC by chance). Counted separately
   because it is an expected artifact of re-running the Pi-side sender, not
   packet loss, and folding it into pi_seqGaps() would add ~65000 to that
   counter in a single frame and destroy its usefulness. Expect this to be
   non-zero during development and zero in a clean run. */
uint32_t pi_seqRestarts(void);
uint32_t pi_maxBurst(void);     /* most bytes drained in one poll             */
