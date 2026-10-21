/* Pi pose link: frame parser, sequence accounting and staleness state. Runs on
   commsTask; pi_getPose() may be called from any task. */
#include "pi_link.h"
#include "timebase.h"
#include "fans.h"
#include <STM32FreeRTOS.h>
#include <task.h>
#include <string.h>

/* Pose staleness. Heading coasts well on the gyro, but position dead-reckoning from
   the accelerometer degrades as t^2 within about a second, so translation authority
   is withdrawn at PI_LOST_MS. */
#define PI_STALE_MS   250    /* ~7 missed detections at 28 Hz */
#define PI_LOST_MS   1000

/* Link is dead after this long with no frames of any kind (~14 frames at 28 Hz). */
#define PI_LINK_DEAD_MS 500

/* Max bytes drained per poll so a streaming Pi cannot hold commsTask. About 4x what
   a 2 ms poll accumulates at 115200. */
#define PI_MAX_DRAIN  128

/* Forward sequence jumps at or beyond this count as a sender restart, not loss. */
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
static uint32_t s_lastFrameUs = 0;    /* any accepted frame, valid or not */
static PiState  s_state = PI_NEVER;
static bool     s_lostActionDone = false;
static bool     s_deadActionDone = false;
static void   (*s_deadHook)(void) = nullptr;

static volatile uint32_t s_rxBytes = 0, s_frames = 0, s_crcErrors = 0;
static volatile uint32_t s_badLen = 0, s_resyncs = 0, s_seqGaps = 0;
static volatile uint32_t s_seqRestarts = 0, s_maxBurst = 0;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
   Must match the Pi sender exactly. */
static uint16_t crc16_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

/* A frame passed CRC: decode and publish. memcpy avoids unaligned float loads. */
static void commit(uint32_t now) {
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

  /* uint16 subtraction handles rollover. A backwards jump (sender restarted) also
     wraps to a huge gap, so large gaps are counted as restarts instead. */
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

  /* Any accepted frame proves the link is alive. */
  s_lastFrameUs    = now;
  s_everFrame      = true;
  s_deadActionDone = false;

  /* Only a frame that saw a tag refreshes the pose clock. */
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
      else if (b == PI_MAGIC1) { /* 0xA5 0xA5: stay, this may be the real start */ }
      else                     { s_st = S_MAGIC1; s_resyncs++; }
      break;

    case S_LEN:
      /* Check the length before buffering so a corrupt value cannot overrun s_buf. */
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

/* Runs every poll whether or not bytes arrived, so timeouts fire on a silent link.
   Timed with us_now(); tick granularity is too coarse. */
static void ladder(uint32_t now) {
  /* Link death first: with no frames, no pose can be fresh either. */
  if (s_everFrame && (uint32_t)((now - s_lastFrameUs) / 1000u) >= PI_LINK_DEAD_MS) {
    s_state = PI_DEAD;
    if (!s_deadActionDone) {
      s_deadActionDone = true;
      s_lostActionDone = true;
      fans_stopAll();
      if (s_deadHook) s_deadHook();
    }
    return;
  }

  if (!s_everValid) { s_state = PI_NEVER; return; }

  uint32_t age_ms = (now - s_lastValidUs) / 1000u;

  if (age_ms < PI_STALE_MS) { s_state = PI_FRESH; return; }
  if (age_ms < PI_LOST_MS)  { s_state = PI_STALE; return; }

  s_state = PI_LOST;
  if (!s_lostActionDone) {
    s_lostActionDone = true;
    /* Soft zero, not fans_stopAll(): the hard kill latches until rearm, and losing
       the tag is expected while searching. The controller also withdraws thrust on
       a stale pose; this is a backstop. */
    fans_setAll(0, 0, 0, 0);
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
