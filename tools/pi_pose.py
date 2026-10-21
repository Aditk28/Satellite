#!/usr/bin/env python3
"""
Dock pose from the AprilTags, printed to the console. Runs on the Pi.

Prints range, bearing and rel_yaw (the values pi_pose_send.py sends to the
STM32), the IPPE ambiguity ratio, and the derived platform and magnet position
in the dock frame. Use it to check signs and geometry by hand. Also provides
solve() and make_detector() for pi_pose_send.py.

All visible dock tags are solved together as one rigid, coplanar target:

      tag 1 (4 cm)        tag 0 (12 cm)        tag 2 (4 cm)
         |<----- 14.15 cm ---|--- 14.15 cm ---->|

Widely spaced points give a much better conditioned yaw than a single tag,
which flips between two near-identical solutions at range.

Frames:
    dock:   origin at tag 0 centre, +X right as you face the wall, +Y up,
            +Z out of the wall toward the camera
    camera: OpenCV, +X right, +Y down, +Z out of the lens

    range   = hypot(t.x, t.z)   horizontal distance, camera to dock origin
    bearing = atan2(t.x, t.z)   positive = dock is to the camera's right
    rel_yaw = 0 when square-on

Sign check on hardware: slide the platform left and bearing should go
positive. If a sign is wrong, fix it here rather than downstream.

    ~/vision/bin/python pi_pose.py
    ~/vision/bin/python pi_pose.py --hz 5        # slower print
"""

import argparse
import math
import sys
import time

import cv2
import numpy as np

from pi_camcfg import lock_exposure, load_intrinsics

# x = offset of each tag centre from tag 0, metres, +ve right as you face the
# wall. All tags are coplanar at the same height. size = black square edge.
TAGS = {
    0: {"size": 0.120, "x":  0.0},
    1: {"size": 0.040, "x": -0.1415},     # left as you face the wall
    2: {"size": 0.040, "x": +0.1415},     # right
}

# Measured focal length at 1280 wide, used when there is no calibration file.
F_PX_AT_1280 = 947.0

# Platform geometry, metres, along the facing direction from the centre of
# rotation (the reaction wheel axis).
L_CAM = 0.1346      # lens ahead of centre
L_MAG = 0.0946      # docking magnet ahead of centre
D_DOCK = 0.35       # ground magnet distance out from the wall


def platform_pose(rng, bearing, relyaw):
    """Convert the camera's view of the dock into the platform pose in the
    dock frame, including the lever arm from lens to platform centre and on to
    the magnet (the arm rotates with heading, so it matters).

    Dock frame (2D, horizontal): origin at tag 0 on the table, +X right as you
    face the wall, +Y out from the wall, psi = heading, 0 = facing the wall.
    With psi=0 and bearing=0 the result is (0, rng); positive bearing gives
    negative x (platform left of the dock).

    Returns (x, y, psi, mag_x, mag_y), metres and radians.
    """
    psi = relyaw
    cam_x = rng * math.sin(psi - bearing)
    cam_y = rng * math.cos(psi - bearing)
    plat_x = cam_x + L_CAM * math.sin(psi)
    plat_y = cam_y + L_CAM * math.cos(psi)
    mag_x = plat_x - L_MAG * math.sin(psi)
    mag_y = plat_y - L_MAG * math.cos(psi)
    return plat_x, plat_y, psi, mag_x, mag_y


def tag_object_points(size, cx):
    """Tag corners in dock coordinates, in detector order lb, rb, rt, lt."""
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
        # The kwarg is `threads`, not `Nthreads`; it defaults to 1.
        return ATag("tag36h11", threads=threads, decimate=decimate,
                    refine_edges=True)
    except Exception as e:
        sys.exit(f"python3-apriltag unavailable: {type(e).__name__}: {e}")


def solve(dets, K, dist):
    """Solve PnP over all recognised dock tags. Returns a dict or None."""
    obj, img, used = [], [], []
    for d in dets:
        tid = int(d["id"])
        if tid not in TAGS:
            continue
        obj.append(tag_object_points(TAGS[tid]["size"], TAGS[tid]["x"]))
        img.append(np.array(d["lb-rb-rt-lt"], dtype=np.float64))
        used.append(tid)
    if not obj:
        return None

    obj = np.concatenate(obj).reshape(-1, 1, 3)
    img = np.concatenate(img).reshape(-1, 1, 2)

    # IPPE (planar target) returns both candidate poses with their errors.
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if n < 1:
        return None

    rvec, tvec = rvecs[0], tvecs[0]
    e = [float(x) for x in np.array(errs).ravel()[:n]]
    # Error ratio of the two solutions: near 1 means yaw is ambiguous. The
    # 0.1 px floor stops two near-perfect fits (the most ambiguous case) from
    # producing a huge ratio.
    ratio = (e[1] / max(e[0], 0.1)) if n > 1 else 99.0

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()

    # The dock and camera +Z axes face each other, so square-on is a 180 deg
    # rotation. Negating both atan2 arguments shifts that to 0.
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

    lock_exposure(args.device)   # must match the calibration exposure

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
    # Without a calibration, uncorrected distortion makes rel-yaw wander as the
    # target moves off-centre.
    K, dist, src = load_intrinsics(W, H)
    if K is None:
        f = F_PX_AT_1280 * (W / 1280.0)
        K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
        dist = np.zeros(5)

    det = make_detector(args.decimate, args.threads)
    period = 1.0 / args.hz
    t_next = 0.0

    print(f"{W}x{H}  [{src}]")
    print("bearing +ve = dock is to the camera's right. Check by sliding the")
    print("platform.\n")
    print("platform pose is in the dock frame: +X right as you face the wall,")
    print(f"+Y out from it. dock target for the magnet is (0.000, {D_DOCK:.3f}).\n")
    print(f"{'tags':<10}{'range m':>9}{'bearing':>9}{'relyaw':>9}"
          f"{'ratio':>7} | {'plat x':>8}{'plat y':>8}{'psi':>7}"
          f" | {'mag x':>8}{'mag y':>8}{'x|psi=0':>9}")
    print("  x|psi=0 = -range*sin(bearing), ignoring relyaw. If it tracks the")
    print("  tape measure and 'plat x' does not, the error is from noisy yaw.")
    print("")

    while True:
        if not cap.grab():
            continue
        ok, buf = cap.retrieve()
        if not ok:
            continue
        if time.perf_counter() < t_next:
            continue                      # throttle printing, not capture
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

        print(f"{str(p['tags']):<10}{p['range']:>9.3f}"
              f"{math.degrees(p['bearing']):>9.2f}"
              f"{math.degrees(p['relyaw']):>9.2f}"
              f"{p['ratio']:>7.1f} | {px:>8.3f}{py:>8.3f}"
              f"{math.degrees(psi):>7.1f}"
              f" | {mx:>8.3f}{my:>8.3f}"
              f"{-p['range']*math.sin(p['bearing']):>9.3f}{flag}")


if __name__ == "__main__":
    main()
