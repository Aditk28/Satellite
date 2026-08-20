#!/usr/bin/env python3
"""
Translation Phase 4, Step 4.1 -- camera intrinsic calibration. Runs ON THE PI.

ONE TIME, EVER -- not per session. Produces camera_intrinsics.json (focal length,
principal point, distortion), which every later pose computation reads. You only
redo it if the lens, the focus, or the CAPTURE RESOLUTION changes.

WHY IT IS NOT OPTIONAL HERE. PnP turns pixel positions into angles and distances
using the intrinsics. Uncalibrated, you would be using the spec-sheet focal
length (424 px from the 120 deg DFOV), which is a guess -- a 10-20% error there
is a 10-20% systematic RANGE error at every distance. And a 120 deg lens has
severe barrel distortion whose worst effects are at the FRAME EDGES, which is
exactly where the flanking tags sit during close approach.

TWO THINGS THAT MUST BE TRUE BEFORE YOU RUN THIS (T14, T35):

  1. SAME RESOLUTION as production -- 1280x720. Intrinsics do not scale between
     this camera's modes because the 4:3 modes are a different aspect ratio, not
     a downscale, so a calibration at one does not transfer to another.
  2. EXPOSURE AND FOCUS LOCKED, at the values you will actually run:
         v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure_time_absolute=100
     Auto-exposure changes effective focal length. Calibrating with it enabled
     produces intrinsics that are wrong as soon as the lighting changes.

AUTO-CAPTURE, and why. Driving a live preview over SSH is painful, so this
watches for the board and saves a frame by itself when it sees one in a part of
the image it still needs. It divides the frame into a 3x3 grid and requires
several views per cell, which is what forces coverage out to the corners -- and
for a lens this wide, the corners are where the distortion information lives. A
calibration built only from centred views fits the middle of the frame well and
extrapolates badly exactly where you need it.

    ~/vision/bin/python pi_calibrate.py
    ~/vision/bin/python pi_calibrate.py --cols 9 --rows 6 --square-mm 25
    ~/vision/bin/python pi_calibrate.py --rational     # if error is high
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np


def grid_cell(pt, w, h, n=3):
    return (min(n - 1, int(pt[0] / (w / n))), min(n - 1, int(pt[1] / (h / n))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width",  type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cols",   type=int, default=9, help="INNER corners across")
    ap.add_argument("--rows",   type=int, default=6, help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--per-cell",  type=int, default=3)
    ap.add_argument("--outdir", default="calib_images")
    ap.add_argument("--rational", action="store_true",
                    help="rational distortion model (k4..k6) -- for wide lenses "
                         "where the plain radial model leaves error > ~1 px")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    pattern = (args.cols, args.rows)
    # Object points in the board's own frame: z=0, spacing = one square.
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm

    need = {(cx, cy): args.per_cell for cx in range(3) for cy in range(3)}
    objpoints, imgpoints = [], []
    n_saved = 0
    last_save = 0.0

    print(f"{W}x{H}   board {args.cols}x{args.rows} inner corners, "
          f"{args.square_mm} mm squares")
    print(f"need {args.per_cell} views per 3x3 cell = {9*args.per_cell} total\n")
    print("Move the board slowly around the frame. TILT it -- views at an angle")
    print("carry the information a flat-on view cannot. Work it into the CORNERS;")
    print("that is where a 120 deg lens hides its distortion.  Ctrl-C to stop early.\n")

    try:
        while sum(need.values()) > 0:
            if not cap.grab():
                continue
            ok, buf = cap.retrieve()
            if not ok:
                continue
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue

            # FAST_CHECK bails early on frames with no board at all, which is
            # most of them -- full detection at 720p is far too slow to run on
            # every frame.
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE +
                cv2.CALIB_CB_FAST_CHECK)
            if not found:
                continue

            centre = corners.reshape(-1, 2).mean(axis=0)
            cell = grid_cell(centre, W, H)
            if need.get(cell, 0) <= 0:
                continue
            # Space captures in time so consecutive saves are genuinely
            # different views rather than the same one twice.
            if time.time() - last_save < 1.0:
                continue

            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            need[cell] -= 1
            n_saved += 1
            last_save = time.time()
            cv2.imwrite(os.path.join(args.outdir, f"calib_{n_saved:02d}.png"), gray)

            remaining = [f"{k}:{v}" for k, v in sorted(need.items()) if v > 0]
            print(f"  [{n_saved:2d}] cell {cell}   still needed -> "
                  f"{' '.join(remaining) if remaining else 'NONE, done'}")
    except KeyboardInterrupt:
        print("\ninterrupted")

    cap.release()

    if len(objpoints) < 8:
        sys.exit(f"\nonly {len(objpoints)} views -- need at least ~8, "
                 f"ideally 20+. Re-run.")

    print(f"\nsolving with {len(objpoints)} views ...")
    flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (W, H), None, None, flags=flags)

    # Per-view reprojection error: a single bad view (motion blur, a board that
    # flexed) can drag the whole fit, and the mean hides it. Print the worst.
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        errs.append(cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj))

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    hfov = 2 * np.degrees(np.arctan(W / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(H / (2 * fy)))

    print(f"\n  RMS reprojection error : {rms:.3f} px")
    print(f"  worst single view      : {max(errs):.3f} px  (view "
          f"{int(np.argmax(errs))+1})")
    print(f"  fx, fy                 : {fx:.1f}, {fy:.1f}")
    print(f"  cx, cy                 : {cx:.1f}, {cy:.1f}   (centre is "
          f"{W/2:.0f}, {H/2:.0f})")
    print(f"  implied FOV            : {hfov:.1f} deg H, {vfov:.1f} deg V")
    print(f"  distortion             : {np.round(dist.ravel(), 4).tolist()}")

    out = {
        "width": W, "height": H,
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "rms_reproj_px": float(rms),
        "n_views": len(objpoints),
        "rational_model": bool(args.rational),
        "board": {"cols": args.cols, "rows": args.rows,
                  "square_mm": args.square_mm},
        "note": "Valid ONLY at this resolution with focus and exposure locked "
                "as they were during capture (T14, T35).",
    }
    with open("camera_intrinsics.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\n  written: camera_intrinsics.json")

    if rms > 1.0:
        print("\n  *** RMS > 1 px. Likely causes, in order: board not FLAT "
              "(mount it rigid),")
        print("      too few tilted views, motion blur, or the plain radial "
              "model not fitting")
        print("      a 120 deg lens -- try  --rational")
    elif rms < 0.5:
        print("\n  RMS < 0.5 px -- good fit.")

    print(f"\n  Sanity-check it: the implied FOV ({hfov:.0f} deg H) should be "
          f"close to the")
    print(f"  spec sheet's 120 deg DIAGONAL. Wildly different means the board "
          f"geometry")
    print(f"  (--cols/--rows/--square-mm) does not match what you printed.")


if __name__ == "__main__":
    main()
