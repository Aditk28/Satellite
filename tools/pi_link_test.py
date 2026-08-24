#!/usr/bin/env python3
"""
Translation Phase 3, Step 3.1b -- two-board link proof.

Runs ON THE PI. Sends a known payload over USART6 and reads back the STM32's
echo (pi_link.cpp, Step 3.1 only -- the echo is deleted in 3.2).

One script proves BOTH directions at once:
  - bytes arriving at the STM32   -> Pi TX -> STM32 PC7 (USART6_RX) is good
  - bytes coming back             -> STM32 PC6 (USART6_TX) -> Pi RX is good
A one-directional test cannot distinguish "my TX is dead" from "their RX is
dead", which is exactly the ambiguity that makes serial bring-up expensive.

Expected round trip: ~1.0 ms serialisation each way for a 12-byte payload at
115200, plus 0-2 ms waiting for commsTask's 2 ms poll to come round. So roughly
2-4 ms, and it should be visibly JITTERY across that ~2 ms window rather than
constant -- the jitter IS the poll period, and seeing it is confirmation the
echo really is going through commsTask rather than some accident.

    python3 pi_link_test.py
"""

import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  sudo apt install -y python3-serial")

PORT    = "/dev/serial0"
BAUD    = 115200
PAYLOAD = b"PING-FROM-PI"
N       = 10

def main():
    try:
        s = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"could not open {PORT}: {e}\n"
                 f"check the console is off (raspi-config) and "
                 f"'ls -l /dev/serial0' points at ttyAMA0")

    print(f"{PORT} @ {BAUD}, payload {PAYLOAD!r} ({len(PAYLOAD)} bytes) x{N}\n")

    rtts, ok = [], 0
    for i in range(N):
        s.reset_input_buffer()          # start each trial from a known state
        t0 = time.perf_counter()
        s.write(PAYLOAD)
        got = s.read(len(PAYLOAD))      # blocks until len bytes or timeout
        t1 = time.perf_counter()

        if got == PAYLOAD:
            rtts.append((t1 - t0) * 1000.0)
            ok += 1
            print(f"  {i+1:2d}: OK        rtt = {rtts[-1]:6.2f} ms")
        elif not got:
            print(f"  {i+1:2d}: NOTHING   (timeout -- no bytes came back)")
        else:
            # Partial or corrupted: the link exists but something is wrong with
            # it. Worth seeing the actual bytes -- a consistent bit pattern is a
            # baud mismatch, random loss is more likely wiring or grounding.
            print(f"  {i+1:2d}: MISMATCH  got {got!r} ({len(got)} of "
                  f"{len(PAYLOAD)} bytes)")
        time.sleep(0.2)

    print(f"\n  {ok}/{N} clean")
    if rtts:
        print(f"  rtt min/mean/max = {min(rtts):.2f} / "
              f"{sum(rtts)/len(rtts):.2f} / {max(rtts):.2f} ms")
    if ok != N:
        print("\n  Not clean. Check, in this order:")
        print("    1. TX/RX crossed?  Pi pin 8 -> STM32 PC7(D9), "
              "Pi pin 10 -> STM32 PC6(CN10-4)")
        print("    2. common ground on Pi pin 6?")
        print("    3. STM32 running the 'rtos' build with pi_link in it?")
        print("    4. press G on the STM32 -- if rx>0 there, the Pi->STM32 "
              "direction is fine and only the return path is broken")

if __name__ == "__main__":
    main()
