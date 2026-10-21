#!/usr/bin/env python3
"""
Live camera view over HTTP with an AprilTag overlay. Runs on the Pi.

Open http://raspberrypi.local:8080 in a browser on the same network. Each
detected tag is drawn with its id, apparent width in pixels and a rough range.
The range uses a measured focal length with no calibration, so it is labelled
UNCAL and is only indicative. Useful for aiming the camera and checking that
all three tags are detected at the ranges you care about.

Only one process can hold /dev/video0, so do not run this alongside the other
camera scripts.

    ~/vision/bin/python pi_cam_view.py
    ~/vision/bin/python pi_cam_view.py --width 640 --height 480 --port 8080
    ~/vision/bin/python pi_cam_view.py --no-detect      # plain view, less CPU
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

# Black-square edge length per tag id, metres. Scales range directly.
TAG_SIZE_M = {0: 0.120, 1: 0.040, 2: 0.040, 3: 0.020}
DEFAULT_TAG_M = 0.040

latest = {"jpg": None}
lock = threading.Lock()


def make_detector():
    # Prefer Debian's python3-apriltag. Its kwarg is `threads` (not `Nthreads`
    # as its docstring says). 2 threads leaves CPU for the HTTP server.
    try:
        from apriltag import apriltag as ATag
        d = ATag("tag36h11", threads=2, decimate=1.0, refine_edges=True)
        return ("deb_apriltag", d)
    except Exception as e:
        print(f"  !! python3-apriltag unavailable: {type(e).__name__}: {e}")
    try:
        from pupil_apriltags import Detector
        d = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0)
        return ("apriltag", d)
    except Exception:
        pass
    try:
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        try:
            return ("aruco_new", cv2.aruco.ArucoDetector(dic,
                                                         cv2.aruco.DetectorParameters()))
        except AttributeError:
            return ("aruco_old", (dic, cv2.aruco.DetectorParameters_create()))
    except Exception:
        return (None, None)


def detections(mode, det, gray):
    """Normalise every backend to [(id, 4x2 corner array), ...]."""
    if mode == "deb_apriltag":
        # Corners are ordered lb, rb, rt, lt: 0-1 bottom edge, 2-3 top edge.
        return [(int(r["id"]), np.array(r["lb-rb-rt-lt"], dtype=float))
                for r in det.detect(gray)]
    if mode == "apriltag":
        return [(r.tag_id, np.array(r.corners)) for r in det.detect(gray)]
    if mode == "aruco_new":
        corners, ids, _ = det.detectMarkers(gray)
    elif mode == "aruco_old":
        corners, ids, _ = cv2.aruco.detectMarkers(gray, det[0], parameters=det[1])
    else:
        return []
    if ids is None:
        return []
    return [(int(i[0]), c[0]) for i, c in zip(ids, corners)]


def capture_loop(args):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # Focal length measured from the 12 cm tag at two known distances (947 px
    # at 1280 wide). The spec sheet's 120 deg DFOV would imply 424 px, which
    # does not match. No principal point or distortion correction.
    f_px = 947.0 * (w / 1280.0)

    mode, det = (None, None) if args.no_detect else make_detector()
    print(f"camera {w}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}  "
          f"detector={mode or 'off'}  f~{f_px:.0f}px (UNCALIBRATED)")

    tprev, fps = time.perf_counter(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        if mode:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for tid, corners in detections(mode, det, gray):
                pts = corners.astype(int)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                # Apparent width: mean of the two horizontal edges.
                px = 0.5 * (np.linalg.norm(corners[0] - corners[1]) +
                            np.linalg.norm(corners[2] - corners[3]))
                s = TAG_SIZE_M.get(tid, DEFAULT_TAG_M)
                rng = (f_px * s / px) if px > 1 else 0.0
                c = pts.mean(axis=0).astype(int)
                cv2.putText(frame, f"id{tid} {px:.0f}px {rng:.2f}m UNCAL",
                            (c[0] - 70, c[1]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1, cv2.LINE_AA)

        now = time.perf_counter()
        fps = 0.9 * fps + 0.1 / max(now - tprev, 1e-6)
        tprev = now
        cv2.putText(frame, f"{fps:4.1f} fps", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
        if ok:
            with lock:
                latest["jpg"] = buf.tobytes()


PAGE = b"""<html><head><title>pi cam</title>
<style>body{background:#111;color:#ccc;font-family:sans-serif;margin:0;
text-align:center}img{max-width:100%;height:auto}</style></head>
<body><img src="/stream"></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    jpg = latest["jpg"]
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " +
                                 str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / 15)      # served rate; capture runs separately
        except (BrokenPipeError, ConnectionResetError):
            pass                          # browser tab closed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width",  type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--port",   type=int, default=8080)
    ap.add_argument("--quality", type=int, default=70)
    ap.add_argument("--no-detect", action="store_true")
    args = ap.parse_args()

    threading.Thread(target=capture_loop, args=(args,), daemon=True).start()
    print(f"open  http://raspberrypi.local:{args.port}   (ctrl-C to stop)")
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
