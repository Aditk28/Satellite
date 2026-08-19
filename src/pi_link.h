#pragma once
#include <Arduino.h>

/*
  Phase 3 -- the Raspberry Pi pose link. USART6, PC6 (TX) / PC7 (RX).

  WHY THIS IS NOT A THIRD ARGUMENT TO commands_init().
  commsTask's contract is line assembly plus "any unrecognised input stops the
  motor", with an X fast path that fires BEFORE the line is even queued
  (commands.cpp). The Pi sends a BINARY pose stream, so: a payload byte of 0x58
  is 'X' and would stop the wheel mid-slew, a 0x0A mid-frame is a line
  terminator, and Pi boot chatter arrives as a scatter of commands. The operator
  parser and the pose parser must never share a code path. Decision B18.

  THE SERIAL INVARIANTS ARE PER PORT, NOT GLOBAL. B15 (sole writer) and the
  sole-reader rule protect Serial and hc05Serial, the two ports telemTask and
  commsTask share. USART6 is touched by nothing else, so pi_link is BOTH its
  sole reader and its sole writer -- the same requirement met by ownership
  instead of arbitration. HardwareSerial is not reentrant per port; that is the
  property being protected, and it does not care which port.

  WHERE IT RUNS. pi_poll() is called from commsTask through
  commands_setAuxPoll(), so the link costs no new task and no new stack -- it
  shares the 2 ms poll that already exists. That is also where the guide's
  Appendix C puts the Pi link.

  STEP 3.1 SCOPE: count bytes and echo them back. No framing, no CRC, no pose.
  The echo is what lets ONE Pi-side script prove both directions at once and
  measure round trip, without spending a command letter (A-Z are all taken).
  STEP 3.2 MUST DELETE THE ECHO -- a framed protocol that echoes its own payload
  back at the sender is a feedback loop.
*/

/* Open the port. Call pre-scheduler from setup(), beside commands_init().
   Does NOT print: it runs before telem_activate(), where telem_print() writes
   the port directly from the CALLING task -- trap T18, a second concurrent
   writer into a non-reentrant HardwareSerial. */
void pi_init(HardwareSerial& port, uint32_t baud);

/* Drain RX (and, in 3.1, echo it). Called from commsTask every POLL_PERIOD_MS.
   Never blocks: the drain is capped per call and the echo is dropped rather
   than waited on when the TX ring is full. */
void pi_poll(void);

uint32_t pi_rxBytes(void);      /* total bytes received, monotonic          */
uint32_t pi_txBytes(void);      /* total bytes echoed                       */
uint32_t pi_txDrops(void);      /* echoes skipped because TX ring was full  */
uint8_t  pi_lastByte(void);     /* most recent byte, for eyeballing in G     */
uint32_t pi_maxBurst(void);     /* most bytes drained in one poll -- if this
                                   approaches the core's 64-byte RX ring, the
                                   poll is too slow or the rate is too high  */
