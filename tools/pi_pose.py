#!/usr/bin/env python3
"""
Translation Phase 4, Step 4.3 -- pose extraction. Runs ON THE PI.

Turns tag detections into the three quantities the Phase 3 payload carries:
range, bearing, rel_yaw -- plus a quality figure the estimator uses to decide
how much to believe the yaw.

THIS PRINTS. It does not send yet. Verify the signs by hand first (see SIGN
CHECK below) -- trap T11 is a frame-transform sign error producing plausible
motion in the wrong direction, and open-loop verification is the cheap way to
catch it. Wiring to USART6 comes after the signs are confirmed.

-----------------------------------------------------------------------------
BUNDLE, NOT PER-TAG. One solvePnP over every visible tag's corners, treated as
a single rigid body with known geometry (measured 2026-08-20):

      tag 1 (4 cm)        tag 0 (12 cm)        tag 2 (4 cm)
         |<----- 14.15 cm ---|--- 14.15 cm ---->|
                     all coplanar, all centres at 9 cm height

Why a bundle and not three independent solves: relative yaw from a SINGLE planar
tag is ill-conditioned at range -- two orientations project almost identically
and the solver flips between them frame to frame (T30). Estimating orientation
from widely-separated points instead of one square's perspective distortion is
far better conditioned. It also degrades gracefully: any visible subset still
gives a fix, and the geometry tells you which tags they were.

NOTE the tags are still COPLANAR, so the ambiguity is improved, not eliminated.
Mounting the flanking tags on ~2 cm standoffs would kill it structurally, at the
cost of another measured geometry parameter. Held in reserve -- see T30.

-----------------------------------------------------------------------------
FRAMES AND SIGNS -- read before trusting any number.

DOCK frame (origin at tag 0's centre, as you LOOK AT the wall):
    +X right      +Y up      +Z out of the wall, toward the camera

CAMERA frame (OpenCV convention, and it is NOT the same as the dock frame):
    +X right      +Y DOWN    +Z forward, out of the lens

solvePnP returns the pose of the DOCK in CAMERA coordinates. From `t`:

    range   = hypot(t.x, t.z)      horizontal distance, camera -> dock origin.
                                   Vertical is ignored on purpose: this is a
                                   planar problem and t.y is just the 2.65 cm
                                   height offset between lens and tag centres.
    bearing = atan2(t.x, t.z)      POSITIVE = dock is to the camera's RIGHT
    rel_yaw = atan2(R[0,2], R[2,2])  the dock's facing versus the camera's.
                                   ZERO = square-on. Sign convention verified
                                   by the SIGN CHECK below, not by derivation.

SIGN CHECK -- do this once, on hardware, before believing anything:
    slide the platform LEFT  -> the dock moves RIGHT in view -> bearing POSITIVE
    slide the platform RIGHT -> bearing NEGATIVE
    view the dock from its left side -> rel_yaw one sign, consistently
If a sign is inverted, fix it HERE and note it, do not compensate downstream.

-----------------------------------------------------------------------------
QUALITY. For a planar target, solvePnPGeneric with IPPE returns up to two
solutions and their reprojection errors. The RATIO of those errors is a direct
ambiguity measure: near 1.0 means the solver cannot tell them apart and the yaw
is untrustworthy; a large ratio means one solution clearly wins. That number
becomes the payload's `quality` field and drives PI_FLAG_AMBIGUOUS, which is how
the estimator widens R on yaw instead of believing a flipped solution (T30).

    ~/vision/bin/python pi_pose.py
    ~/vision/bin/python pi_pose.py --hz 5        # slower print, easier to read
"""

import argparse
import math
import sys
import time

import cv2
import numpy as np

# --- measured geometry, 2026-08-20 -------------------------------------------
# x = horizontal offset of the tag CENTRE from tag 0's centre, metres, +ve right
# as you look at the wall. All three are at the same height and coplanar, so y
# and z are 0 for every tag. size = black square edge; MEASURE IT, printers
# scale, and this multiplies range directly.
TAGS = {
    0: {"size": 0.120, "x":  0.0},
    1: {"size": 0.040, "x": -0.1415},     # left  as you look at the wall
    2: {"size": 0.040, "x": +0.1415},     # right
}

# Measured focal length, NOT the spec sheet (which claims 120 deg DFOV -> 424).
# Two known distances gave 958 and 937; see decision B24/B25.
F_PX_AT_1280 = 947.0


def tag_object_points(size, cx):
    """Corners in DOCK coordinates, in the order the detector reports them:
    lb, rb, rt, lt (left-bottom, right-bottom, right-top, left-top).
    Dock frame is +X right, +Y up, so 'bottom' is -Y."""
    h = size / 2.0
    return np.array([
        [cx - h, -h, 0.0],
        [cx + h, -h, 0.0],
        [cx + h, +h, 0.0],
        [cx - h, +h, 0.0],
    ], dtype=np.float64)


def make_detector(decimate, threads):
    try:
        from apriltag import apriltag as ATag
        # kwarg is `threads`, NOT `Nthreads` -- the docstring is wrong and it
        # defaults to 1, which costs ~3x on a quad-core (T32).
        return ATag("tag36h11", threads=threads, decimate=decimate,
                    refine_edges=True)
    except Exception as e:
        sys.exit(f"python3-apriltag unavailable: {type(e).__name__}: {e}")


def solve(dets, K, dist):
    """Bundle solve over every recognised tag. Returns dict or None."""
    obj, img, used = [], [], []
    for d in dets:
        tid = int(d["id"])
        if tid not in TAGS:
            continue                      # not part of the dock -- ignore
        obj.append(tag_object_points(TAGS[tid]["size"], TAGS[tid]["x"]))
        img.append(np.array(d["lb-rb-rt-lt"], dtype=np.float64))
        used.append(tid)
    if not obj:
        return None

    obj = np.concatenate(obj).reshape(-1, 1, 3)
    img = np.concatenate(img).reshape(-1, 1, 2)

    # IPPE is the planar-target solver and returns BOTH candidate poses with
    # their errors, which is what gives us an honest ambiguity measure. It needs
    # >= 4 points; we always have a multiple of 4.
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if n < 1:
        return None

    rvec, tvec = rvecs[0], tvecs[0]
    e = [float(x) for x in np.array(errs).ravel()[:n]]
    # ratio ~1.0 -> solver cannot separate the two poses -> yaw untrustworthy.
    #
    # The 0.1 px floor on the denominator is load-bearing, not cosmetic. Dividing
    # by a raw e[0] makes "both solutions fit PERFECTLY" -- the maximally
    # AMBIGUOUS case -- come out as a huge ratio, i.e. maximally confident,
    # which is exactly backwards. Flooring at a realistic corner-noise level
    # means two equally-good fits produce a SMALL ratio and get flagged. Found
    # via a synthetic square-on case, which is also the docking configuration
    # and the worst one for planar ambiguity (T30).
    ratio = (e[1] / max(e[0], 0.1)) if n > 1 else 99.0

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()

    # relyaw: note the NEGATED arguments. The dock frame's +Z points out of the
    # wall toward the camera while the camera's +Z points forward into it, so
    # square-on is genuinely a 180 deg relative rotation -- correct geometry,
    # useless as a control signal. Negating both arguments of atan2 rotates the
    # reported angle by exactly pi and stays correctly wrapped to (-pi, pi].
    # Measured on hardware 2026-08-20: square-on read -180 before this.
    return {
        "range":   float(math.hypot(t[0], t[2])),
        "bearing": float(math.atan2(t[0], t[2])),
        "relyaw":  float(math.atan2(-R[0, 2], -R[2, 2])),
        "ratio":   ratio,
        "reproj":  e[0],
        "tags":    sorted(used),
        "t":       t,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width",  type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--decimate", type=float, default=2.0)
    ap.add_argument("--threads",  type=int, default=3)
    ap.add_argument("--hz", type=float, default=10.0, help="print rate")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    f = F_PX_AT_1280 * (W / 1280.0)
    # Principal point ASSUMED at the image centre and distortion ASSUMED zero --
    # we measured a focal length, not a calibration (B24). Bearing degrades
    # toward the frame edges. pi_calibrate.py fixes this when it matters.
    K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
    dist = np.zeros(5)

    det = make_detector(args.decimate, args.threads)
    period = 1.0 / args.hz
    t_next = 0.0

    print(f"{W}x{H}  f={f:.0f}px (MEASURED, uncalibrated -- no distortion model)")
    print("bearing +ve = dock is to the camera's RIGHT.  VERIFY BY SLIDING THE")
    print("PLATFORM before trusting it (T11).\n")
    print(f"{'tags':<10}{'range m':>9}{'bearing':>9}{'relyaw':>9}"
          f"{'ratio':>8}{'reproj':>8}")

    while True:
        if not cap.grab():
            continue
        ok, buf = cap.retrieve()
        if not ok:
            continue
        if time.perf_counter() < t_next:
            continue                      # decimate the PRINT, not the capture
        t_next = time.perf_counter() + period

        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        p = solve(det.detect(gray), K, dist)
        if p is None:
            print(f"{'--':<10}{'no tags':>9}")
            continue

        flag = ""
        if p["ratio"] < 2.0:
            flag = "  AMBIGUOUS(yaw)"      # two poses fit nearly as well
        if len(p["tags"]) < 3:
            flag += "  PARTIAL"
        print(f"{str(p['tags']):<10}{p['range']:>9.3f}"
              f"{math.degrees(p['bearing']):>9.2f}"
              f"{math.degrees(p['relyaw']):>9.2f}"
              f"{p['ratio']:>8.1f}{p['reproj']:>8.2f}{flag}")


if __name__ == "__main__":
    main()
