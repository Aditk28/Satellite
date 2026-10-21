#!/usr/bin/env python3
"""
Synthetic pose frame generator for testing the STM32 pi_link parser without a
camera. Runs on the Pi.

Sends a fake approach (range closing, bearing and yaw settling) in the same
wire format as pi_pose_send.py. Can corrupt CRCs, drop frames and lead with
garbage to exercise CRC checks, sequence-gap counting and resync.

Wire format (must match src/pi_link.h):

    [0xA5][0x5A][len=28][ 28-byte payload ][crc_lo][crc_hi]

    off  type      field
     0   uint16    seq
     2   uint8     flags
     3   uint8     tag_id
     4   float32   range_m
     8   float32   bearing_rad
    12   float32   relyaw_rad
    16   float32   quality
    20   uint32    age_us
    24   uint8     n_tags
    25   uint8[3]  pad

    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over len + payload.
    Little-endian. Magic is not covered by the CRC.

    python3 pi_pose_sim.py                    # 10 Hz clean stream, 20 s
    python3 pi_pose_sim.py --corrupt 0.2      # 20% of frames get a bad CRC
    python3 pi_pose_sim.py --drop 0.2         # 20% of frames not sent (seq gaps)
    python3 pi_pose_sim.py --garbage          # random bytes first, tests resync
    python3 pi_pose_sim.py --seconds 5        # short, then watch the ladder
"""

import argparse
import random
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  sudo apt install -y python3-serial")

PORT = "/dev/serial0"
BAUD = 115200

MAGIC = b"\xA5\x5A"
PAYLOAD_LEN = 28

FLAG_VALID     = 0x01
FLAG_AMBIGUOUS = 0x02
FLAG_MULTITAG  = 0x04


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(seq, flags, tag_id, rng, bearing, relyaw, quality, age_us, n_tags):
    payload = struct.pack(
        "<HBBffffIB3x",           # little-endian, 3x = the three pad bytes
        seq & 0xFFFF, flags, tag_id,
        rng, bearing, relyaw, quality,
        age_us, n_tags,
    )
    assert len(payload) == PAYLOAD_LEN, len(payload)
    body = bytes([PAYLOAD_LEN]) + payload          # CRC covers len + payload
    crc = crc16_ccitt_false(body)
    return MAGIC + body + struct.pack("<H", crc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz",      type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--corrupt", type=float, default=0.0, help="fraction with bad CRC")
    ap.add_argument("--drop",    type=float, default=0.0, help="fraction not sent")
    ap.add_argument("--garbage", action="store_true", help="lead with random bytes")
    args = ap.parse_args()

    s = serial.Serial(PORT, BAUD, timeout=0.5)

    if args.garbage:
        # 40 random bytes; the parser must skip them and lock onto the first
        # real frame.
        junk = bytes(random.randrange(256) for _ in range(40))
        s.write(junk)
        print(f"sent {len(junk)} garbage bytes first (expect resync>0, frames still clean)")

    # Avoid a period that is a multiple of the STM32's 2 ms poll, so frames
    # arrive at varying phases.
    period = 1.0 / args.hz
    if abs((period * 1000) % 2.0) < 1e-6:
        period += 0.0003
        print(f"nudged period to {period*1000:.1f} ms to avoid aliasing with the 2 ms poll")

    n_sent = n_corrupt = n_dropped = 0
    t_end = time.time() + args.seconds
    seq = 0
    t0 = time.time()

    print(f"{PORT} @ {BAUD}, {args.hz} Hz for {args.seconds} s "
          f"(corrupt {args.corrupt:.0%}, drop {args.drop:.0%})\n")

    while time.time() < t_end:
        # Slow fake approach from 2 m so the STM32 readout visibly moves.
        t = time.time() - t0
        rng     = max(0.15, 2.0 - 0.05 * t)
        bearing = 0.30 * (1.0 - t / max(args.seconds, 1e-9))
        relyaw  = 0.20 * (1.0 - t / max(args.seconds, 1e-9))
        quality = 0.55 + 0.4 * min(1.0, t / max(args.seconds, 1e-9))

        flags = FLAG_VALID | FLAG_MULTITAG
        if quality < 0.7:
            flags |= FLAG_AMBIGUOUS

        t_capture = time.perf_counter()
        time.sleep(0.012)                # stand-in for detection latency
        age_us = int((time.perf_counter() - t_capture) * 1e6)

        seq = (seq + 1) & 0xFFFF
        frame = build(seq, flags, 7, rng, bearing, relyaw, quality, age_us, 3)

        if random.random() < args.drop:
            n_dropped += 1               # seq still advances, so the STM32 sees a gap
        else:
            if random.random() < args.corrupt:
                frame = frame[:-2] + b"\x00\x00"   # break only the CRC
                n_corrupt += 1
            s.write(frame)
            n_sent += 1

        time.sleep(period)

    s.flush()
    print(f"sent {n_sent} frames ({n_corrupt} deliberately corrupted), "
          f"{n_dropped} dropped to create seq gaps\n")
    print("Now on the STM32, press G:")
    print(f"    frames  should be {n_sent - n_corrupt}")
    print(f"    crc     should be {n_corrupt}")
    # A CRC-rejected frame never has its seq decoded, so it also counts as a gap.
    print(f"    seqgaps should be {n_dropped + n_corrupt}"
          f"   (= {n_dropped} dropped + {n_corrupt} CRC-rejected)")
    print("    state   FRESH now -> STALE after 250 ms -> LOST at 1 s "
          "(fans zeroed) -> DEAD at 3 s")
    print("\nWait 4 s and press G again to see the ladder at DEAD.")
    print("\nNote: seqgaps includes CRC-rejected frames. Transport loss alone")
    print("      is seqgaps - crc.")


if __name__ == "__main__":
    main()
