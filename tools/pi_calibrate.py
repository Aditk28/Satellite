#!/usr/bin/env python3
"""
Camera intrinsic calibration. Runs on the Pi.

Writes camera_intrinsics.json (focal length, principal point, distortion),
which the pose scripts load. Redo it only if the lens, focus or capture
resolution changes. Calibrate at the production resolution (1280x720) with
exposure locked; this script locks exposure itself via pi_camcfg.

Frames are captured automatically. The image is split into a 3x3 grid and each
cell needs --per-cell views, which forces coverage into the corners where the
120 deg lens distortion is strongest. A view is only kept when the board is
tilted and held still. Use --view to watch progress in a browser.

    ~/vision/bin/python pi_calibrate.py
    ~/vision/bin/python pi_calibrate.py --cols 9 --rows 6 --square-mm 25 --view
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

    Prefers findChessboardCornersSB, which is more accurate and does its own
    subpixel refinement. Falls back to the classic detector plus cornerSubPix.
    """
    if HAVE_SB and not force_classic:
        ok, c = cv2.findChessboardCornersSB(
            gray, pattern,
            cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, c
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


TILT_MIN = 1.15        # >=15% foreshortening counts as a tilted view


def tilt_ratio(corners, cols, rows):
    """Worst ratio between opposite edges of the detected grid. 1.0 is flat-on.

    Tilted views are needed to separate focal length from distance; a set of
    only flat-on views leaves the solve poorly conditioned.
    """
    c = corners.reshape(-1, 2)
    tl, tr, bl, br = c[0], c[cols - 1], c[-cols], c[-1]
    top, bot = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    lft, rgt = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    r1 = max(top, bot) / max(min(top, bot), 1e-6)
    r2 = max(lft, rgt) / max(min(lft, rgt), 1e-6)
    return max(r1, r2)


# Optional MJPEG preview (--view). Lives here because only one process can hold
# /dev/video0, so pi_cam_view.py cannot run alongside.
_frame = {"jpg": None}
_flock = threading.Lock()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

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
    """Draw the coverage grid and status onto the frame for the preview."""
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for i in (1, 2):
        cv2.line(vis, (W * i // 3, 0), (W * i // 3, H), (60, 60, 60), 1)
        cv2.line(vis, (0, H * i // 3), (W, H * i // 3), (60, 60, 60), 1)
    for (cx, cy), n in need.items():
        col = (0, 200, 0) if n <= 0 else (0, 0, 220)   # green = cell done
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
    ap.add_argument("--cols",   type=int, default=9, help="inner corners across")
    ap.add_argument("--rows",   type=int, default=6, help="inner corners down")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--per-cell",  type=int, default=3)
    ap.add_argument("--outdir", default="calib_images")
    ap.add_argument("--rational", action="store_true",
                    help="rational distortion model (k4..k6), for when the "
                         "plain radial model leaves error above ~1 px")
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

    # Lock exposure before opening the camera; intrinsics are only valid at
    # the exposure they were captured with.
    print("locking exposure:")
    if not lock_exposure(args.device):
        sys.exit("\nrefusing to calibrate with automatic exposure: it changes "
                 "the effective focal length.")

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
    # Board-frame object points: z=0, spacing one square.
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm

    need = {(cx, cy): args.per_cell for cx in range(3) for cy in range(3)}
    objpoints, imgpoints = [], []
    prev_corners = None
    n_saved = 0
    last_save = 0.0

    print(f"{W}x{H}   board {args.cols}x{args.rows} inner corners, "
          f"{args.square_mm} mm squares")
    print(f"need {args.per_cell} views per 3x3 cell = {9*args.per_cell} total\n")
    print("Move the board slowly around the frame, tilted, and work it into")
    print("the corners. Ctrl-C to stop early.\n")

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

            found, corners = find_board(gray, pattern, args.classic)

            # Decide whether to keep this frame; reason is shown in the preview.
            tr, reason = None, "no board"
            cell = None
            if found:
                tr = tilt_ratio(corners, args.cols, args.rows)
                centre = corners.reshape(-1, 2).mean(axis=0)
                cell = grid_cell(centre, W, H)

                # Mean corner motion since the last frame, as a motion-blur proxy.
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
        sys.exit(f"\nonly {len(objpoints)} views, need at least ~8 "
                 f"(ideally 20+). Re-run.")

    print(f"\nsolving with {len(objpoints)} views ...")
    flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (W, H), None, None, flags=flags)

    # Per-view RMS reprojection error (L2 norm / sqrt(N)), to spot one bad view.
    errs = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
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
        "note": "Valid only at this resolution with focus and exposure locked "
                "as they were during capture.",
    }
    with open("camera_intrinsics.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\n  written: camera_intrinsics.json")

    if rms > 1.0:
        print("\n  RMS > 1 px. Likely causes: board not flat, too few tilted")
        print("  views, motion blur, or the radial model not fitting a")
        print("  120 deg lens (try --rational).")
    elif rms < 0.5:
        print("\n  RMS < 0.5 px, good fit.")

    print(f"\n  Sanity check: the implied FOV ({hfov:.0f} deg H) should be "
          f"close to the")
    print(f"  spec sheet's 120 deg diagonal. If it is far off, --cols/--rows/"
          f"--square-mm")
    print(f"  do not match the printed board.")


if __name__ == "__main__":
    main()
