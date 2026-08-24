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

from pi_camcfg import lock_exposure, load_intrinsics

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

# --- platform geometry, metres ------------------------------------------------
# Both offsets are along the platform's FACING direction, measured from the
# platform's centre of rotation (the reaction wheel axis, which is what psi
# rotates about).
L_CAM = 0.1346      # lens, ahead of centre
L_MAG = 0.0946      # docking magnet, ahead of centre (13.46 - 4 cm behind lens)
D_DOCK = 0.35       # ground magnet, out from the wall. Update if the wall moves.


def platform_pose(rng, bearing, relyaw):
    """Vision measures where the DOCK is relative to the CAMERA. The controller
    needs where the PLATFORM is relative to the DOCK. Two chained transforms:

      1. invert the measurement -> camera position in the dock frame
      2. walk the lever arm back from the lens to the platform centre, then
         forward again to the magnet

    Step 2 matters because the arm ROTATES with heading: a 10 deg heading error
    swings the lens 2.3 cm from where a naive 'camera ~= platform' model puts
    it, which is twice the docking tolerance from heading alone.

    DOCK FRAME (2D, horizontal plane; the vertical axis is irrelevant here):
        origin  tag 0 projected to the table
          +X    right, as you FACE the wall
          +Y    out from the wall, toward the approach side
        psi     platform heading; 0 = facing the wall square-on

    Sanity checks built into the algebra, both confirmed on hardware:
      psi=0, bearing=0        -> (0, rng)   dead in front at the measured range
      bearing POSITIVE        -> x NEGATIVE  platform is LEFT of the dock,
                                 which is what sliding left produced (T11 check)

    Returns (x, y, psi, mag_x, mag_y), metres and radians.
    """
    psi = relyaw
    # Direction from camera to dock, rotated out of the camera axis by bearing;
    # the camera sits at minus that, scaled by range.
    cam_x = rng * math.sin(psi - bearing)
    cam_y = rng * math.cos(psi - bearing)
    # Lens is AHEAD of centre, so the centre is behind it along the facing dir.
    plat_x = cam_x + L_CAM * math.sin(psi)
    plat_y = cam_y + L_CAM * math.cos(psi)
    # Magnet is ahead of centre by a shorter arm.
    mag_x = plat_x - L_MAG * math.sin(psi)
    mag_y = plat_y - L_MAG * math.cos(psi)
    return plat_x, plat_y, psi, mag_x, mag_y


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

    # Must match the exposure the calibration was taken at (T35).
    lock_exposure(args.device)

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
    period = 1.0 / args.hz
    t_next = 0.0

    print(f"{W}x{H}  [{src}]")
    print("bearing +ve = dock is to the camera's RIGHT.  VERIFY BY SLIDING THE")
    print("PLATFORM before trusting it (T11).\n")
    print("platform pose is in the DOCK frame: +X right as you face the wall,")
    print(f"+Y out from it. dock target for the magnet is (0.000, {D_DOCK:.3f}).\n")
    print(f"{'tags':<10}{'range m':>9}{'bearing':>9}{'relyaw':>9}"
          f"{'ratio':>7} | {'plat x':>8}{'plat y':>8}{'psi':>7}"
          f" | {'mag x':>8}{'mag y':>8}{'x|psi=0':>9}")
    print("  x|psi=0 ignores relyaw entirely: -range*sin(bearing). If THAT")
    print("  tracks the tape measure while 'plat x' does not, the position")
    print("  error is coming from noisy yaw, not from bearing or range.")
    print("")

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
        if p["ratio"] < 3.0 and abs(p["relyaw"]) > math.radians(10.0):
            flag = "  AMBIGUOUS(yaw)"      # low ratio only matters off-square
        if len(p["tags"]) < 3:
            flag += "  PARTIAL"

        px, py, psi, mx, my = platform_pose(p["range"], p["bearing"],
                                            p["relyaw"])
        dock_err = math.hypot(mx - 0.0, my - D_DOCK)

        print(f"{str(p['tags']):<10}{p['range']:>9.3f}"
              f"{math.degrees(p['bearing']):>9.2f}"
              f"{math.degrees(p['relyaw']):>9.2f}"
              f"{p['ratio']:>7.1f} | {px:>8.3f}{py:>8.3f}"
              f"{math.degrees(psi):>7.1f}"
              f" | {mx:>8.3f}{my:>8.3f}"
              f"{-p['range']*math.sin(p['bearing']):>9.3f}{flag}")


if __name__ == "__main__":
    main()
