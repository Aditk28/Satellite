#!/usr/bin/env python3
"""
Translation Phase 4 -- threaded capture pipeline. Runs ON THE PI.

This is the prototype of the real pose pipeline, not just another benchmark.
It fixes the ceiling the serial benchmark ran into and measures the one number
the Phase 3 protocol needs and we have never actually had: TRUE capture->pose
latency (age_us), rather than an iteration period that includes idle waiting.

THE PROBLEM IT SOLVES. The serial loop is grab -> decode -> detect -> repeat, so
capture and processing take turns. Once processing got fast enough (decimate 4:
decode 15 ms + detect 13 ms = 28 ms of work), `grab` simply blocked for 38 ms
waiting for the camera, and the period stayed pinned at ~67 ms no matter how
much faster detection got. Every millisecond saved was absorbed by waiting.

TWO THINGS THE THREAD BUYS:

 1. OVERLAP. Decode runs on the capture thread while detect runs on the main
    thread, on different cores. Period becomes max(decode, detect) instead of
    their sum -- so ~15 ms of work instead of ~28.
 2. FRESHNESS. The capture thread always keeps the NEWEST frame and drops older
    ones on the floor. The detector never works on a stale frame, and never
    blocks waiting for a new one.

WHY LATENCY IS NOT THE PERIOD, and why this script reports them separately.
`total` in the serial benchmark was the iteration period -- useful for fps,
wrong for latency, because blocked time in grab() is a frame that had not been
captured yet, not the age of one that had. Here the timestamp is taken the
instant grab() returns, so latency is measured from (approximately) capture to
(exactly) pose-available. That IS age_us.

WHAT IS STILL NOT MEASURED. Sensor exposure and USB transit happen before
grab() returns and remain invisible from userspace -- likely a further 10-30 ms.
So this is still a LOWER BOUND, and age_us built from it under-reports by that
constant. Phase 5 either measures it (point the camera at a millisecond counter
on a screen and read the lag out of the captured image) or carries it as a known
bias. Do not treat this as the whole latency.

    ~/vision/bin/python pi_vision_threaded.py
    ~/vision/bin/python pi_vision_threaded.py --decimate 2.0 --seconds 20
"""

import argparse
import sys
import threading
import time

try:
    import cv2
except ImportError:
    sys.exit("opencv missing:  sudo apt install -y python3-opencv")

import numpy as np


def make_detector(decimate, threads):
    """Reference AprilTag detector. NOTE the kwarg is `threads`, NOT `Nthreads`
    -- the module's own docstring says Nthreads and the C binding rejects it.
    It defaults to 1, so getting this wrong quietly costs ~3x on a quad-core."""
    try:
        from apriltag import apriltag as ATag
        return ATag("tag36h11", threads=threads, decimate=decimate,
                    refine_edges=True), "apriltag (reference)"
    except Exception as e:
        sys.exit(f"python3-apriltag unavailable: {type(e).__name__}: {e}\n"
                 f"  sudo apt install -y python3-apriltag")


class Camera(threading.Thread):
    """Continuously captures and decodes, keeping only the most recent frame.

    Deliberately drops frames. A queue would preserve every frame at the cost of
    letting them age, which is exactly backwards for a control-loop measurement:
    a pose from three frames ago is worse than no pose, because the estimator
    will believe it (trap T9). Newest-wins is the correct policy here.
    """

    def __init__(self, device, width, height):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            sys.exit(f"cannot open {device}")
        # fourcc BEFORE size -- setting size first can lock the driver to a
        # format that has no such size and the request silently fails.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # Hand back the raw JPEG buffer so we can decode straight to grayscale,
        # skipping colour reconstruction and a separate cvtColor. Measured
        # 50.2 ms -> 15.0 ms at 720p.
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        # BUFFERSIZE 4, not 1. With a single V4L2 buffer the driver has nowhere
        # to capture while we hold the frame, so it misses every other one --
        # measured 15.9 fps against the camera's 30. A depth of 4 cannot make
        # frames stale HERE because the capture thread drains continuously and
        # take() always returns the newest; the shallow-buffer reasoning applies
        # to a serial loop, not this one. Measured 15.9 -> 28.6 fps, latency
        # 44.5 -> 60.9 ms (processing contention, not queue age: 31 ms decode +
        # 29 ms detect ~= 60 ms), and worst case actually IMPROVED, 89 -> 76 ms.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

        self.lock    = threading.Lock()
        self.latest  = None      # (gray, t_capture)
        self.seq     = 0
        self.stop    = False
        self.n_grab  = 0
        self.t_decode = []

    def run(self):
        while not self.stop:
            if not self.cap.grab():
                time.sleep(0.005)
                continue
            # Timestamp AS EARLY AS POSSIBLE -- this is our best proxy for when
            # the frame actually existed. Everything after this is our latency.
            t_cap = time.perf_counter()
            ok, buf = self.cap.retrieve()
            if not ok:
                continue
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            self.t_decode.append((time.perf_counter() - t_cap) * 1e3)
            with self.lock:
                self.latest = (gray, t_cap)
                self.seq += 1
                self.n_grab += 1

    def take(self, last_seq):
        """Return (gray, t_capture, seq) for a frame newer than last_seq, or
        None. Never blocks -- the caller decides what to do with nothing."""
        with self.lock:
            if self.latest is None or self.seq == last_seq:
                return None
            gray, t_cap = self.latest
            return gray, t_cap, self.seq

    def release(self):
        self.stop = True
        time.sleep(0.1)
        self.cap.release()


def pct(xs, p):
    return sorted(xs)[min(len(xs) - 1, int(len(xs) * p / 100.0))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device",   default="/dev/video0")
    ap.add_argument("--width",    type=int,   default=1280)
    ap.add_argument("--height",   type=int,   default=720)
    ap.add_argument("--decimate", type=float, default=4.0)
    ap.add_argument("--threads",  type=int,   default=3,
                    help="detector threads; one core is left for capture")
    ap.add_argument("--seconds",  type=float, default=15.0)
    args = ap.parse_args()

    det, name = make_detector(args.decimate, args.threads)
    cam = Camera(args.device, args.width, args.height)
    cam.start()
    time.sleep(1.0)                      # let exposure/gain settle

    print(f"{name}  decimate={args.decimate}  threads={args.threads}  "
          f"{args.width}x{args.height} MJPG grey\n")

    lat, t_det, tags, misses = [], [], [], 0
    last_seq = 0
    n = 0
    t_end = time.time() + args.seconds
    t0 = time.perf_counter()

    while time.time() < t_end:
        got = cam.take(last_seq)
        if got is None:
            misses += 1                  # no new frame yet -- we outran the camera
            time.sleep(0.001)
            continue
        gray, t_cap, last_seq = got

        a = time.perf_counter()
        d = det.detect(gray)
        b = time.perf_counter()

        t_det.append((b - a) * 1e3)
        lat.append((b - t_cap) * 1e3)    # capture -> pose available == age_us
        tags.append(len(d))
        n += 1

    wall = time.perf_counter() - t0
    cam.release()

    if not lat:
        sys.exit("no frames processed")

    dec = cam.t_decode[5:] or [0]
    print(f"  processed {n} frames in {wall:.1f} s  ->  {n/wall:5.1f} fps")
    print(f"  camera delivered {cam.n_grab} frames  ->  {cam.n_grab/wall:5.1f} fps")
    print(f"  polls with no new frame: {misses}"
          f"   ({'CPU is ahead of the camera' if misses > n else 'camera is ahead of the CPU'})")
    print()
    print(f"  decode  (capture thread) mean {np.mean(dec):6.1f} ms")
    print(f"  detect  (main thread)    mean {np.mean(t_det):6.1f} ms  "
          f"p95 {pct(t_det, 95):6.1f}")
    print()
    print(f"  LATENCY capture->pose    mean {np.mean(lat):6.1f} ms  "
          f"p95 {pct(lat, 95):6.1f}  max {max(lat):6.1f}")
    print(f"     ^ this is age_us, minus an unmeasured exposure+USB constant")
    print()
    print(f"  tags per frame: mean {np.mean(tags):.2f}  min {min(tags)}  "
          f"max {max(tags)}")
    if min(tags) < 3:
        print("  *** tags dropped below 3 -- at this decimate/range the 4 cm")
        print("      flanking tags are not reliably detected, and those are")
        print("      what resolve the yaw ambiguity (T30).")


if __name__ == "__main__":
    main()
