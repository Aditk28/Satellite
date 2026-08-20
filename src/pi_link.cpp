#include "pi_link.h"
#include "timebase.h"
#include "fans.h"
#include <STM32FreeRTOS.h>
#include <task.h>
#include <string.h>

/* ---- staleness thresholds ------------------------------------------------
   The two axes dead-reckon COMPLETELY differently, which is why this is a
   ladder and not one number:

     heading  coasts on the gyro at ~0.8 deg/min. After 3 s that is 0.04 deg.
              Effectively free.
     position coasts on double-integrated accelerometer, error ~ 0.5*a*t^2.
              Bias is body-fixed and consistently 0.1658 / 0.0961 m/s^2
              (user-confirmed 2026-08-19, B15 closed), so it calibrates out and
              the residual is a small fraction of that -- but the t^2 still
              wins within about a second.

   So translation authority is withdrawn a full 2 s before attitude control is
   even questioned. A platform holding heading with the fans off is stable and
   recoverable; one that has also dropped attitude control just drifts.        */
#define PI_STALE_MS   250    /* ~7 missed detections at 28 Hz: tolerate dropout
                                without twitching                              */
#define PI_LOST_MS   1000    /* translation dead-reckoning budget is spent     */

/* LINK death is a DIFFERENT thing from pose staleness, and keying both off the
   same clock was a real bug (found on hardware 2026-08-20 by covering the tags:
   the link was perfectly healthy, frames arriving with crc=0, and the ladder
   reported DEAD).

   Losing sight of the tag is NORMAL -- during SEARCH the tag is not visible by
   definition, and B2 says it is also routinely lost at close range. Losing the
   LINK is a fault. Conflating them would have fired the terminal action on every
   search sweep and during every close approach.

   So: pose freshness (last VALID frame) drives the estimator and translation
   authority; link health (last frame of ANY kind, valid or not) drives the fault
   path. The Pi deliberately keeps sending frames with PI_FLAG_VALID clear when
   it cannot see the dock, which is what makes the distinction observable. */
#define PI_LINK_DEAD_MS 500  /* ~14 missed frames at 28 Hz -- silence, not blindness */

/* Bytes drained per pi_poll(). Bounded work per poll: a Pi streaming
   continuously must not hold commsTask in a while(available()) spin. 128 is ~4x
   what a 2 ms poll can accumulate at 115200 -- slack in normal operation, a
   hard stop in pathological operation. */
#define PI_MAX_DRAIN  128

/* Largest forward sequence jump still treated as loss rather than as a restart.
   At 10-30 Hz, 1000 missing frames is 30-100 s of outage -- the staleness ladder
   fired at 3 s, so anything beyond this is not "loss" in any useful sense. */
#define PI_SEQ_MAX_GAP 1000

typedef enum { S_MAGIC1 = 0, S_MAGIC2, S_LEN, S_PAYLOAD, S_CRC_LO, S_CRC_HI } RxState;

static HardwareSerial* s_port = nullptr;

static RxState  s_st    = S_MAGIC1;
static uint8_t  s_buf[PI_MAX_PAYLOAD];
static uint8_t  s_len   = 0;
static uint8_t  s_idx   = 0;
static uint16_t s_crc   = 0;
static uint16_t s_rxCrc = 0;

static PiPose   s_pose;
static bool     s_haveSeq  = false;
static uint16_t s_lastSeq  = 0;
static bool     s_everValid = false;
static uint32_t s_lastValidUs = 0;
static bool     s_everFrame = false;
static uint32_t s_lastFrameUs = 0;    /* ANY accepted frame, valid or not */
static PiState  s_state = PI_NEVER;
static bool     s_lostActionDone = false;
static bool     s_deadActionDone = false;
static void   (*s_deadHook)(void) = nullptr;

static volatile uint32_t s_rxBytes = 0, s_frames = 0, s_crcErrors = 0;
static volatile uint32_t s_badLen = 0, s_resyncs = 0, s_seqGaps = 0;
static volatile uint32_t s_seqRestarts = 0, s_maxBurst = 0;

/* CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection, no final XOR.
   Bitwise rather than table-driven: 28 payload bytes x 8 = ~230 iterations per
   frame at 30 Hz is nothing, and it costs no flash for a 512-byte table. The
   PARAMETERS are the part that matters -- a mismatch with the Pi rejects 100%
   of frames and presents identically to a wiring fault. */
static uint16_t crc16_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

/* A whole frame passed CRC. Decode and publish. */
static void commit(uint32_t now) {
  /* memcpy rather than a packed-struct overlay: the payload is 4-byte aligned
     by construction, but s_buf's alignment is not guaranteed by the language
     and an unaligned float load is not something to leave to chance. */
  PiPose p;
  memcpy(&p.seq,         s_buf +  0, 2);
  p.flags     = s_buf[2];
  p.tag_id    = s_buf[3];
  memcpy(&p.range_m,     s_buf +  4, 4);
  memcpy(&p.bearing_rad, s_buf +  8, 4);
  memcpy(&p.relyaw_rad,  s_buf + 12, 4);
  memcpy(&p.quality,     s_buf + 16, 4);
  memcpy(&p.age_us,      s_buf + 20, 4);
  p.n_tags    = s_buf[24];
  p.rx_us     = now;

  /* Sequence gap = frames the Pi sent that never landed. uint16 arithmetic
     wraps correctly at rollover, so 65535 -> 0 needs no special case.
     A BACKWARDS jump does, though: the same wrap that makes rollover free turns
     "stream restarted at 1 after seq 150" into 1-151 = 65386, added in one shot,
     permanently poisoning the counter. That is not exotic -- it is what happens
     every time the Pi-side sender is restarted without resetting this board.
     So bound it, and count a restart as its own event rather than as loss. */
  if (s_haveSeq) {
    uint16_t expected = (uint16_t)(s_lastSeq + 1);
    uint16_t gap      = (uint16_t)(p.seq - expected);
    if (gap != 0) {
      if (gap < PI_SEQ_MAX_GAP) s_seqGaps += gap;
      else                      s_seqRestarts++;
    }
  }
  s_lastSeq = p.seq;
  s_haveSeq = true;
  s_frames++;

  taskENTER_CRITICAL();
  s_pose = p;
  taskEXIT_CRITICAL();

  /* EVERY accepted frame proves the LINK is alive, whether or not it saw a tag.
     That is what clears the fault path. */
  s_lastFrameUs    = now;
  s_everFrame      = true;
  s_deadActionDone = false;

  /* Only a frame that actually SAW a tag refreshes the POSE clock. A detector
     reporting "nothing in view" proves the link is alive but does NOT make the
     pose fresh -- conflating those is trap T9, a stale pose driving control
     while every counter looks healthy. */
  if (p.flags & PI_FLAG_VALID) {
    s_lastValidUs    = now;
    s_everValid      = true;
    s_lostActionDone = false;
  }
}

static void feed(uint8_t b, uint32_t now) {
  switch (s_st) {
    case S_MAGIC1:
      if (b == PI_MAGIC1) s_st = S_MAGIC2;
      break;

    case S_MAGIC2:
      if      (b == PI_MAGIC2) s_st = S_LEN;
      else if (b == PI_MAGIC1) { /* 0xA5 0xA5 -- hold, this may be the real start */ }
      else                     { s_st = S_MAGIC1; s_resyncs++; }
      break;

    case S_LEN:
      /* Reject the length HERE rather than after buffering: a corrupt length is
         how a parser gets walked off the end of its buffer. Requiring exactly
         PI_PAYLOAD_LEN also makes this a free format-version check. */
      if (b != PI_PAYLOAD_LEN) { s_badLen++; s_st = S_MAGIC1; break; }
      s_len = b;
      s_idx = 0;
      s_crc = crc16_update(0xFFFF, b);     /* CRC covers len + payload */
      s_st  = S_PAYLOAD;
      break;

    case S_PAYLOAD:
      s_buf[s_idx++] = b;
      s_crc = crc16_update(s_crc, b);
      if (s_idx >= s_len) s_st = S_CRC_LO;
      break;

    case S_CRC_LO:
      s_rxCrc = b;
      s_st    = S_CRC_HI;
      break;

    case S_CRC_HI:
      s_rxCrc |= (uint16_t)b << 8;
      if (s_rxCrc == s_crc) commit(now);
      else                  s_crcErrors++;
      s_st = S_MAGIC1;
      break;
  }
}

/* The staleness ladder. Runs every poll REGARDLESS of whether bytes arrived --
   that is the property a timeout needs, and it is why this lives on commsTask's
   unconditional 2 ms tick rather than being driven by frame arrival.

   Gated on us_now() (TIM5, 1 MHz), never on tick arithmetic: 1 ms granularity
   structurally cannot resolve what this is measuring, which is trap T20's
   lesson from the fan period investigation. */
static void ladder(uint32_t now) {
  /* ---- fault path: is the LINK alive? Checked FIRST because link death
     subsumes pose staleness -- if no frames are arriving, no pose can be
     fresh either, and the fault response is the one that matters. */
  if (s_everFrame && (uint32_t)((now - s_lastFrameUs) / 1000u) >= PI_LINK_DEAD_MS) {
    s_state = PI_DEAD;
    if (!s_deadActionDone) {
      s_deadActionDone = true;
      /* Kill translation too -- a dead link means we have no idea where we are,
         and unlike a lost tag this is not a condition we expect to recover
         from on its own. */
      s_lostActionDone = true;
      fans_stopAll();
      if (s_deadHook) s_deadHook();
    }
    return;
  }

  /* ---- normal path: how old is the POSE? Losing sight of the tag is an
     expected operating condition (SEARCH, close approach), NOT a fault. */
  if (!s_everValid) { s_state = PI_NEVER; return; }

  uint32_t age_ms = (now - s_lastValidUs) / 1000u;

  if (age_ms < PI_STALE_MS) { s_state = PI_FRESH; return; }
  if (age_ms < PI_LOST_MS)  { s_state = PI_STALE; return; }

  /* Pose older than the translation dead-reckoning budget. Withdraw translation
     authority but say nothing about the link, which is fine. */
  s_state = PI_LOST;
  if (!s_lostActionDone) {
    s_lostActionDone = true;
    /* ⚠️ PHASE 6 MUST CHANGE THIS. fans_stopAll() is the HARD kill and it
       LATCHES until `R` (B10) -- correct for a fault, wrong for a condition we
       expect to enter and leave repeatedly while searching. It also only zeroes
       once, so a running controller would simply re-command the fans next
       cycle: what Phase 6 actually needs is an INHIBIT flag the allocator
       checks, plus a soft zero that leaves the ESC armed so authority returns
       the instant a pose does. Safe today only because nothing commands the
       fans yet. */
    fans_stopAll();
  }
}

void pi_init(HardwareSerial& port, uint32_t baud) {
  s_port = &port;
  memset(&s_pose, 0, sizeof(s_pose));
  s_port->begin(baud);
}

void pi_poll(void) {
  if (!s_port) return;

  uint32_t now = us_now();
  uint32_t n   = 0;

  while (s_port->available() && n < PI_MAX_DRAIN) {
    uint8_t b = (uint8_t)s_port->read();
    s_rxBytes++;
    n++;
    feed(b, now);
  }
  if (n > s_maxBurst) s_maxBurst = n;

  ladder(now);
}

bool pi_getPose(PiPose* out) {
  if (!out || !s_everValid) return false;
  taskENTER_CRITICAL();
  *out = s_pose;
  taskEXIT_CRITICAL();
  return true;
}

PiState pi_state(void) { return s_state; }

uint32_t pi_ageUs(void) {
  if (!s_everValid) return UINT32_MAX;
  return us_now() - s_lastValidUs;
}

uint32_t pi_linkAgeUs(void) {
  if (!s_everFrame) return UINT32_MAX;
  return us_now() - s_lastFrameUs;
}

bool pi_linkAlive(void) {
  return s_everFrame && (pi_linkAgeUs() / 1000u) < PI_LINK_DEAD_MS;
}

void pi_setDeadHook(void (*fn)(void)) { s_deadHook = fn; }

uint32_t pi_rxBytes(void)   { return s_rxBytes; }
uint32_t pi_frames(void)    { return s_frames; }
uint32_t pi_crcErrors(void) { return s_crcErrors; }
uint32_t pi_badLen(void)    { return s_badLen; }
uint32_t pi_resyncs(void)   { return s_resyncs; }
uint32_t pi_seqGaps(void)     { return s_seqGaps; }
uint32_t pi_seqRestarts(void) { return s_seqRestarts; }
uint32_t pi_maxBurst(void)  { return s_maxBurst; }
