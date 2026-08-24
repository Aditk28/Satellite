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

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from pi_camcfg import lock_exposure


HAVE_SB = hasattr(cv2, "findChessboardCornersSB")


def find_board(gray, pattern, force_classic=False):
    """Locate the board. Returns (found, corners).

    findChessboardCornersSB ("sector based", OpenCV 4.x) is both faster and
    markedly more accurate than the classic detector, and it does its own
    subpixel refinement -- so no cornerSubPix afterwards. Corner precision feeds
    straight into reprojection error, which is the number we are trying to get
    under 0.5 px, so this is not a micro-optimisation."""
    if HAVE_SB and not force_classic:
        ok, c = cv2.findChessboardCornersSB(
            gray, pattern,
            cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, c
        # fall through -- SB missing the board does NOT mean it is not there
    ok, c = cv2.findChessboardCorners(
        gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE +
        cv2.CALIB_CB_FAST_CHECK)
    if ok:
        c = cv2.cornerSubPix(
            gray, c, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return ok, (c if ok else None)


def grid_cell(pt, w, h, n=3):
    return (min(n - 1, int(pt[0] / (w / n))), min(n - 1, int(pt[1] / (h / n))))


TILT_MIN = 1.15        # >=15% foreshortening counts as a genuinely tilted view


def tilt_ratio(corners, cols, rows):
    """Perspective foreshortening as a cheap stand-in for board tilt.

    WHY THIS MATTERS AS MUCH AS FRAME COVERAGE. The 3x3 grid samples DISTORTION,
    which varies across the image. Tilt does something different and equally
    necessary: it separates FOCAL LENGTH from DISTANCE. A perfectly fronto-
    parallel board is mathematically ambiguous between 'small and close' and
    'large and far' -- only perspective breaks that. So a set of 27 flat-on views
    fills every cell and still leaves the solve poorly conditioned.

    Measured as the worst ratio between opposite edges of the detected grid:
    1.0 = dead flat-on, larger = more tilted."""
    c = corners.reshape(-1, 2)
    tl, tr, bl, br = c[0], c[cols - 1], c[-cols], c[-1]
    top, bot = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    lft, rgt = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    r1 = max(top, bot) / max(min(top, bot), 1e-6)
    r2 = max(lft, rgt) / max(min(lft, rgt), 1e-6)
    return max(r1, r2)


# --- optional live preview ---------------------------------------------------
# The calibrator and the standalone viewer cannot both hold /dev/video0, so the
# preview lives HERE. It is not a luxury: the whole job is getting board views
# into specific parts of the frame, and doing that blind -- over SSH, from a
# scrolling list of cell coordinates -- is needlessly hard. The overlay shows
# which cells are still outstanding and whether the board is being detected at
# all, which are the only two questions you have while waving it around.
_frame = {"jpg": None}
_flock = threading.Lock()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                       # keep the console readable for the capture log

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b'<body style="background:#111;margin:0;'
                             b'text-align:center"><img src="/stream" '
                             b'style="max-width:100%"></body>')
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                with _flock:
                    jpg = _frame["jpg"]
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpg)).encode() +
                                 b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1 / 10)
        except (BrokenPipeError, ConnectionResetError):
            pass


def _publish(gray, corners, found, need, W, H, n_saved, total, tilt=None,
             reason="", pattern=None, scale=1.0):
    """Annotate and hand to the server. Cheap enough at ~10 Hz."""
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for i in (1, 2):               # the 3x3 coverage grid
        cv2.line(vis, (W * i // 3, 0), (W * i // 3, H), (60, 60, 60), 1)
        cv2.line(vis, (0, H * i // 3), (W, H * i // 3), (60, 60, 60), 1)
    for (cx, cy), n in need.items():
        # green cell = satisfied, red = still wants views
        col = (0, 200, 0) if n <= 0 else (0, 0, 220)
        cv2.putText(vis, str(max(0, n)),
                    (cx * W // 3 + W // 6 - 8, cy * H // 3 + H // 6 + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2, cv2.LINE_AA)
    if found and corners is not None and pattern is not None:
        cv2.drawChessboardCorners(vis, pattern, corners, True)
    msg = f"{n_saved}/{total}"
    if tilt is not None:
        msg += f"   tilt {tilt:4.2f}"
    msg += f"   {reason}"
    col = (0, 220, 0) if reason == "READY" else (
        (0, 200, 220) if found else (0, 0, 220))
    cv2.putText(vis, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2,
                cv2.LINE_AA)
    if scale != 1.0:
        vis = cv2.resize(vis, (int(W * scale), int(H * scale)))
    ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    if ok:
        with _flock:
            _frame["jpg"] = buf.tobytes()


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
    ap.add_argument("--view", action="store_true",
                    help="serve a live preview with the coverage grid overlaid")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--preview-scale", type=float, default=1.0)
    ap.add_argument("--classic", action="store_true",
                    help="force the classic detector (skip SB)")
    args = ap.parse_args()

    if args.view:
        threading.Thread(
            target=lambda: ThreadingHTTPServer(("0.0.0.0", args.port),
                                               _Handler).serve_forever(),
            daemon=True).start()
        print(f"preview: http://raspberrypi.local:{args.port}\n")

    os.makedirs(args.outdir, exist_ok=True)

    # BEFORE opening the camera. The calibration is only valid at the exposure
    # it was captured with, and v4l2 controls do not persist across re-opens
    # (T35). Refuse to proceed rather than silently produce bad intrinsics.
    print("locking exposure:")
    if not lock_exposure(args.device):
        sys.exit("\nrefusing to calibrate with automatic exposure -- it changes "
                 "effective focal length, so the result would be wrong as soon "
                 "as the lighting did.")

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
    objpoints, imgpoints, tilts = [], [], []
    prev_corners = None
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
            found, corners = find_board(gray, pattern, args.classic)

            # --- decide whether this frame is worth keeping, and SAY WHY NOT.
            # Previously it grabbed whatever appeared, which meant it fired
            # before you could angle the board and filled cells with flat,
            # sometimes-blurred views. Now it waits for a view that is actually
            # useful, and the preview shows what it is waiting for.
            tr, reason = None, "no board"
            cell = None
            if found:
                tr = tilt_ratio(corners, args.cols, args.rows)
                centre = corners.reshape(-1, 2).mean(axis=0)
                cell = grid_cell(centre, W, H)

                # STABILITY. Corner motion between consecutive frames is a
                # direct proxy for motion blur, which displaces corners and is
                # a prime suspect for high reprojection error. It also gives
                # you time to pose the board instead of racing the shutter.
                moved = 99.0
                if prev_corners is not None and prev_corners.shape == corners.shape:
                    moved = float(np.mean(np.linalg.norm(
                        corners.reshape(-1, 2) - prev_corners.reshape(-1, 2),
                        axis=1)))
                prev_corners = corners.copy()

                if need.get(cell, 0) <= 0:      reason = "cell done"
                elif tr < TILT_MIN:             reason = "TILT MORE"
                elif moved > 1.5:               reason = "HOLD STILL"
                elif time.time() - last_save < 0.6: reason = "wait"
                else:                           reason = "READY"

            if args.view:
                _publish(gray, corners, found, need, W, H, n_saved,
                         9 * args.per_cell, tr, reason, pattern,
                         args.preview_scale)

            if reason != "READY":
                continue
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            tilts.append(tr)
            need[cell] -= 1
            n_saved += 1
            last_save = time.time()
            cv2.imwrite(os.path.join(args.outdir, f"calib_{n_saved:02d}.png"), gray)

            remaining = [f"{k}:{v}" for k, v in sorted(need.items()) if v > 0]
            kind = "TILTED" if tr >= TILT_MIN else "flat  "
            print(f"  [{n_saved:2d}] cell {cell} {kind} {tr:4.2f}   "
                  f"still needed -> "
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
        # RMS per point = L2norm / sqrt(N). Dividing by N (the original
        # bug) understates by sqrt(N) ~ 7x and made 'worst view' read
        # LOWER than the overall RMS, which is impossible.
        errs.append(cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
                    / np.sqrt(len(proj)))

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
