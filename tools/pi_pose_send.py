#!/usr/bin/env python3
"""
Translation Phase 4 -> Phase 3 join: the real pose sender. Runs ON THE PI.

Closes the chain. Camera -> detect -> bundle solvePnP -> Phase 3 frame -> USART6
-> the STM32's pi_link parser. Everything here already exists and is tested
separately; this wires the pieces together and adds nothing new to the protocol.

    pi_vision_threaded.py  ->  threaded capture + the capture TIMESTAMP
    pi_pose.py             ->  bundle solve, sign conventions, ambiguity ratio
    pi_pose_sim.py         ->  the frame format and CRC (byte-identical)
    pi_link.{h,cpp}        ->  the STM32 parser and staleness ladder

WHAT MAKES age_us REAL NOW. The synthetic sender guessed a detection latency;
this measures it. The capture thread stamps `perf_counter()` the instant grab()
returns, and age_us is the interval from there to the moment the frame is handed
to the UART. That is the number the estimator needs in order to propagate the
state forward from when the measurement was actually taken. It still EXCLUDES
sensor exposure and USB transit, which are invisible from userspace -- so it
remains a lower bound, under-reporting by a roughly constant offset (see the
Step 4.2 note). Do not treat it as the whole latency.

FRAMES ARE SENT EVEN WITH NO TAG IN VIEW, with PI_FLAG_VALID clear. That is
deliberate: it lets the STM32 distinguish "the link is alive but I cannot see the
dock" from "the link is dead", which are completely different failures needing
completely different responses. pi_link.cpp only refreshes its freshness clock on
VALID frames, so the staleness ladder still fires correctly while invalid frames
keep arriving -- conflating the two is trap T9.

    ~/vision/bin/python pi_pose_send.py
    ~/vision/bin/python pi_pose_send.py --port /dev/serial0 --quiet
"""

import argparse
import math
import struct
import sys
import threading
import time

import cv2
import numpy as np

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  sudo apt install -y python3-serial")

from pi_pose import solve, make_detector, F_PX_AT_1280
from pi_camcfg import lock_exposure, load_intrinsics

MAGIC = b"\xA5\x5A"
PAYLOAD_LEN = 28

FLAG_VALID     = 0x01
FLAG_AMBIGUOUS = 0x02
FLAG_MULTITAG  = 0x04
FLAG_OOP       = 0x08


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(seq, flags, tag_id, rng, bearing, relyaw, quality, age_us, n_tags):
    payload = struct.pack("<HBBffffIB3x", seq & 0xFFFF, flags, tag_id,
                          rng, bearing, relyaw, quality, age_us, n_tags)
    assert len(payload) == PAYLOAD_LEN
    body = bytes([PAYLOAD_LEN]) + payload       # CRC covers len + payload
    return MAGIC + body + struct.pack("<H", crc16_ccitt_false(body))


class Camera(threading.Thread):
    """Newest-frame-wins capture. See B23 -- a queue would preserve every frame
    at the cost of letting them age, which is backwards for a measurement
    feeding a control loop."""

    def __init__(self, device, width, height):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            sys.exit(f"cannot open {device}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        self.lock, self.latest, self.seq, self.stop = threading.Lock(), None, 0, False

    def run(self):
        while not self.stop:
            if not self.cap.grab():
                time.sleep(0.005)
                continue
            t_cap = time.perf_counter()     # as close to capture as we can see
            ok, buf = self.cap.retrieve()
            if not ok:
                continue
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            with self.lock:
                self.latest = (gray, t_cap)
                self.seq += 1

    def take(self, last):
        with self.lock:
            if self.latest is None or self.seq == last:
                return None
            g, t = self.latest
            return g, t, self.seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--port",   default="/dev/serial0")
    ap.add_argument("--baud",   type=int, default=115200)
    ap.add_argument("--width",  type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--decimate", type=float, default=2.0)
    ap.add_argument("--threads",  type=int, default=3)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    lock_exposure(args.device)   # must match the calibration (T35)

    ser = serial.Serial(args.port, args.baud, timeout=0)
    cam = Camera(args.device, args.width, args.height)
    cam.start()
    time.sleep(1.0)

    W = int(cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # Real intrinsics if we have them. The fallback is a MEASURED focal length
    # with the principal point assumed centred and no distortion model -- that
    # assumption is what made rel-yaw wander under pure translation, because
    # uncorrected barrel distortion shears an off-centre target and solvePnP
    # fits the shear as a rotation.
    K, dist, src = load_intrinsics(W, H)
    if K is None:
        f = F_PX_AT_1280 * (W / 1280.0)
        K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
        dist = np.zeros(5)
    det = make_detector(args.decimate, args.threads)

    print(f"{W}x{H} [{src}]  ->  {args.port} @ {args.baud}")
    print("sending Phase 3 frames. invalid frames ARE sent (flags bit0 clear) so")
    print("the STM32 can tell 'no tag' from 'link dead'.  Ctrl-C to stop.\n")

    seq, last, n_sent, n_valid = 0, 0, 0, 0
    ages, t_report = [], time.time() + 2.0

    try:
        while True:
            got = cam.take(last)
            if got is None:
                time.sleep(0.001)
                continue
            gray, t_cap, last = got

            p = solve(det.detect(gray), K, dist)

            if p is None:
                flags, tag_id, n_tags = 0, 0, 0
                rng = bearing = relyaw = quality = 0.0
            else:
                n_tags = len(p["tags"])
                tag_id = 0 if 0 in p["tags"] else p["tags"][0]
                rng, bearing, relyaw = p["range"], p["bearing"], p["relyaw"]
                # ratio 1 -> the two candidate poses are indistinguishable;
                # large -> one clearly wins. Map to 0..1 monotonically.
                quality = 1.0 - 1.0 / max(p["ratio"], 1.0)
                flags = FLAG_VALID
                if n_tags > 1:
                    flags |= FLAG_MULTITAG
                # AMBIGUOUS requires BOTH a low ratio AND a large |relyaw|.
                # Near square-on the two solutions sit close to each other AND
                # close to zero, so a low ratio there is benign -- flagging on
                # ratio alone would cry wolf during docking, which is exactly
                # when the pose is most trustworthy. See the Step 4.3 note.
                if p["ratio"] < 3.0 and abs(relyaw) > math.radians(10.0):
                    flags |= FLAG_AMBIGUOUS
                n_valid += 1

            seq = (seq + 1) & 0xFFFF
            age_us = int((time.perf_counter() - t_cap) * 1e6)
            ser.write(build(seq, flags, tag_id, rng, bearing, relyaw,
                            quality, age_us, n_tags))
            n_sent += 1
            ages.append(age_us / 1000.0)

            if not args.quiet and time.time() > t_report:
                t_report = time.time() + 2.0
                v = "  --  no tag" if p is None else (
                    f"  rng {p['range']:.3f} m  brg {math.degrees(p['bearing']):+6.2f}"
                    f"  yaw {math.degrees(p['relyaw']):+6.2f}"
                    f"  q {quality:.2f}  tags {p['tags']}")
                print(f"seq {seq:5d}  sent {n_sent:6d}  valid {n_valid:6d}"
                      f"  age {np.mean(ages[-60:]):5.1f} ms{v}")
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop = True
        time.sleep(0.15)
        cam.cap.release()
        ser.close()
        if ages:
            print(f"\nsent {n_sent} frames, {n_valid} with a valid pose")
            print(f"age_us mean {np.mean(ages):.1f} ms   max {max(ages):.1f} ms")


if __name__ == "__main__":
    main()
