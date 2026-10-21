#pragma once
#include <Arduino.h>

/*
  Raspberry Pi pose link on USART6 (PC6 TX / PC7 RX). Binary frames, so it is kept
  apart from the operator command parser (a payload byte could look like a command).
  pi_poll() runs on commsTask via commands_setAuxPoll(); it is the only code that
  touches USART6. commsTask polls on a fixed period, so timeouts are evaluated even
  when no bytes arrive.

  Frame:  [0xA5][0x5A][len][payload, len bytes][crc_lo][crc_hi]
    len  payload bytes only; must equal PI_PAYLOAD_LEN.
    crc  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over len + payload,
         sent little-endian.
  All fields little-endian.

  Payload (28 bytes):
     off  type      field
      0   uint16    seq           increments per frame
      2   uint8     flags         PI_FLAG_*
      3   uint8     tag_id        primary tag in view
      4   float32   range_m       camera to tag centre
      8   float32   bearing_rad   tag centre off camera boresight
     12   float32   relyaw_rad    tag face normal vs boresight
     16   float32   quality       0..1, from the PnP ambiguity ratio
     20   uint32    age_us        capture to transmit, on the Pi clock
     24   uint8     n_tags        tags contributing to this fix
     25   uint8[3]  pad

  Range/bearing is sent instead of x/y because its errors are roughly uncorrelated,
  so the estimator can use a diagonal measurement covariance. age_us is a duration
  rather than a timestamp because the two clocks are not synchronised.
*/

#define PI_MAGIC1        0xA5
#define PI_MAGIC2        0x5A
#define PI_PAYLOAD_LEN   28
#define PI_MAX_PAYLOAD   64      /* parser buffer bound */

#define PI_FLAG_VALID     0x01   /* a tag was detected this frame */
#define PI_FLAG_AMBIGUOUS 0x02   /* PnP could not separate the two solutions */
#define PI_FLAG_MULTITAG  0x04   /* fix used more than one tag */
#define PI_FLAG_OOP       0x08   /* large out-of-plane pitch/roll, suspect */

/* Pose freshness (time since last valid frame) and link health (time since any
   frame) are tracked separately. Losing sight of the tag is normal during search
   and close approach; the Pi keeps sending frames with PI_FLAG_VALID clear. Only a
   silent link is a fault. */
typedef enum {
  PI_NEVER = 0,   /* no valid pose since boot */
  PI_FRESH,       /* valid pose newer than PI_STALE_MS */
  PI_STALE,       /* pose stale, link fine; estimator coasts */
  PI_LOST,        /* pose older than PI_LOST_MS; fans zeroed (soft) */
  PI_DEAD         /* no frames at all; fans hard-killed */
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
  uint32_t age_us;     /* Pi-side capture to transmit */
  uint32_t rx_us;      /* us_now() when the frame finished arriving */
} PiPose;

/* Open the port. Call from setup() before the scheduler starts. Does not print. */
void pi_init(HardwareSerial& port, uint32_t baud);

/* Drain RX (bounded), parse frames, update the state. Called from commsTask. */
void pi_poll(void);

/* Copy the latest accepted frame (check flags for PI_FLAG_VALID). Returns false
   until a valid pose has arrived. Safe from any task (copied in a critical section). */
bool pi_getPose(PiPose* out);

PiState  pi_state(void);
uint32_t pi_ageUs(void);        /* since last valid frame; UINT32_MAX if never */

/* True while frames of any kind are arriving. This is the check a fault path should
   use; PI_LOST only means the dock is not in view. */
bool     pi_linkAlive(void);
uint32_t pi_linkAgeUs(void);    /* since last frame of any kind */

/* Called once per link loss, from commsTask, after the fans are stopped.
   Must not block. */
void pi_setDeadHook(void (*fn)(void));

/* Diagnostics. CRC errors and resyncs can be non-zero on a noisy link; rxBytes
   climbing with frames flat means the two ends disagree on format or CRC. */
uint32_t pi_rxBytes(void);
uint32_t pi_frames(void);       /* accepted frames */
uint32_t pi_crcErrors(void);
uint32_t pi_badLen(void);       /* len field not PI_PAYLOAD_LEN */
uint32_t pi_resyncs(void);      /* lost magic sync */
/* Sequence numbers that never arrived, including CRC-rejected frames.
   Transport loss is roughly pi_seqGaps() - pi_crcErrors(). */
uint32_t pi_seqGaps(void);
/* Sequence went backwards or jumped by PI_SEQ_MAX_GAP or more, usually a restart of
   the Pi-side sender. Kept out of pi_seqGaps() so a restart does not add ~65000. */
uint32_t pi_seqRestarts(void);
uint32_t pi_maxBurst(void);     /* most bytes drained in one poll */
