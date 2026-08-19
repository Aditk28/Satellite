#include "pi_link.h"

/* Bytes drained per pi_poll() call. Bounded work per poll: a Pi streaming
   continuously must not be able to hold commsTask in a while(available()) spin.
   128 is ~4x the 23 bytes a 2 ms poll can accumulate at 115200, so it is slack
   in normal operation and a hard stop in pathological operation. */
#define PI_MAX_DRAIN  128

static HardwareSerial*   s_port     = nullptr;
static volatile uint32_t s_rx       = 0;
static volatile uint32_t s_tx       = 0;
static volatile uint32_t s_txDrops  = 0;
static volatile uint32_t s_maxBurst = 0;
static volatile uint8_t  s_last     = 0;

void pi_init(HardwareSerial& port, uint32_t baud) {
  s_port = &port;
  s_port->begin(baud);
}

void pi_poll(void) {
  if (!s_port) return;

  uint32_t n = 0;
  while (s_port->available() && n < PI_MAX_DRAIN) {
    uint8_t b = (uint8_t)s_port->read();
    s_rx++;
    n++;
    s_last = b;

    /* STEP 3.1 ONLY -- delete in 3.2 (see pi_link.h).
       availableForWrite() gate, not a bare write(): HardwareSerial::write()
       BLOCKS when the 64-byte TX ring is full, which would stall commsTask for
       milliseconds. Control (3) and FOC (4) are above it and would be fine, but
       stalling the operator's X path for a diagnostic is not acceptable. Drop
       and count instead -- the same policy as telemetry's timeout-0 sends. */
    if (s_port->availableForWrite() > 0) { s_port->write(b); s_tx++; }
    else                                 { s_txDrops++; }
  }

  if (n > s_maxBurst) s_maxBurst = n;
}

uint32_t pi_rxBytes(void)  { return s_rx; }
uint32_t pi_txBytes(void)  { return s_tx; }
uint32_t pi_txDrops(void)  { return s_txDrops; }
uint8_t  pi_lastByte(void) { return s_last; }
uint32_t pi_maxBurst(void) { return s_maxBurst; }
