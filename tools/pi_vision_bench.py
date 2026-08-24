#!/usr/bin/env python3
"""
Translation Phase 4, Step 4.2 -- detection-rate benchmark. Runs ON THE PI.

Answers the three questions that decide everything downstream, BEFORE any
calibration work is spent on a resolution we might not end up using (T14):

  1. What frame rate does each capture mode ACTUALLY deliver?
  2. Where does the time go -- USB transfer, MJPEG decode, or tag detection?
  3. What number should go in the Phase 3 frame's age_us field?

WHY MJPEG MUST BE REQUESTED EXPLICITLY. OpenCV defaults to YUYV. This camera
caps YUYV at 1280x720 to 10 fps while MJPEG runs 30 -- so the default gives a
silent 3x loss that presents as "the Pi is too slow". The mode table below
includes the YUYV row deliberately, so the difference is measured rather than
asserted. CAP_PROP_FOURCC must be set BEFORE the frame size, and the driver can
quietly ignore it, which is why the negotiated fourcc is read back and printed.

WHY THE SAMPLE FRAMES MATTER. This camera offers 1280x720 (16:9) and then only
4:3 modes -- there is no 16:9 mode below 720p. So dropping resolution also
changes the ASPECT RATIO, and the intrinsics therefore do NOT scale between
modes; each needs its own calibration (T14). Whether the 4:3 modes are a crop of
the 720p view or the full sensor with 720p cropped OUT of it changes which mode
is actually the better deal, and one look at the saved frames settles it.

WHAT THIS DOES NOT MEASURE, and it matters. Timing starts at cap.grab(), which
is the earliest moment userspace can see the frame. Sensor exposure and USB
transit happened BEFORE that and are invisible from here -- likely a further
10-30 ms. So the figure this reports is a LOWER BOUND on true capture->pose
latency, and age_us built from it will under-report by roughly that constant.
Phase 5 must either measure the offset (point the camera at a millisecond
counter on a screen and read the lag out of the captured image) or carry it as
a known bias. Do not treat this number as the whole latency.

    python3 pi_vision_bench.py
    python3 pi_vision_bench.py --frames 200 --device /dev/video0
"""

import argparse
import sys
import time

try:
    import cv2
except ImportError:
    sys.exit("opencv missing:  sudo apt install -y python3-opencv")

import numpy as np


# ---------------------------------------------------------------- detector ---
# THREE backends, in descending order of quality. Always PRINT which one ran: a
# benchmark whose backend is ambiguous is worthless for comparison later, and we
# learned that the expensive way (2026-08-19) -- a run silently taken on aruco
# was compared against one taken on the reference detector, and the two are ~4x
# apart, which briefly looked like a hardware problem.
#
#  1. apriltag        -- Debian's python3-apriltag, the REFERENCE AprilTag 3
#                        library, prebuilt (`sudo apt install python3-apriltag`).
#                        Preferred: no compilation, and it exposes `decimate`.
#  2. pupil_apriltags -- same underlying detector via pip. Needs a source build
#                        that OOM-kills on a 512 MB Pi; use (1) instead there.
#  3. cv2.aruco       -- can DECODE tag36h11 but uses the ArUco quad detector.
#                        Measurably weaker at range and oblique angles, AND it
#                        has no decimation knob at all -- see supports_decimate.
#
# WHY supports_decimate EXISTS. `decimate` runs quad detection on a downscaled
# image (payload decoding and corner refinement stay at full resolution, so pose
# accuracy is largely preserved). It is the single biggest cost knob on a CPU
# this size -- measured 3.6x. aruco ignores it entirely, so a benchmark row
# labelled "dq=2.0" on that backend is the SAME RUN as dq=1.0 wearing a
# different label. Flag it rather than print a meaningless comparison.
class Detector:
    def __init__(self, quad_decimate=1.0, nthreads=4):
        self.decimate = quad_decimate
        self.backend = None
        self.supports_decimate = True

        # NEVER swallow these exceptions silently. A quiet fallback is how a run
        # ends up on the 4x-slower backend without anyone noticing, which cost a
        # session (2026-08-19). Report why each preferred backend was skipped.
        why = []

        # 1. Debian python3-apriltag.
        #    The kwarg is `threads`, NOT `Nthreads` -- the module's own docstring
        #    says Nthreads and the C binding rejects it. It defaults to 1, so
        #    getting this wrong silently costs ~3x on a quad-core.
        try:
            from apriltag import apriltag as ATag
            self._d = ATag("tag36h11",
                           threads=nthreads,
                           decimate=quad_decimate,
                           refine_edges=True)
            self.backend = (f"apriltag (Debian python3-apriltag, REFERENCE "
                            f"detector, decimate={quad_decimate}, "
                            f"threads={nthreads})")
            self._mode = "deb_apriltag"
            return
        except Exception as e:
            why.append(f"python3-apriltag: {type(e).__name__}: {e}")

        # 2. pupil_apriltags via pip.
        try:
            from pupil_apriltags import Detector as ATDetector
            self._d = ATDetector(families="tag36h11",
                                 nthreads=nthreads,
                                 quad_decimate=quad_decimate)
            self.backend = f"pupil_apriltags (quad_decimate={quad_decimate})"
            self._mode = "apriltag"
            return
        except Exception as e:
            why.append(f"pupil_apriltags: {type(e).__name__}: {e}")

        for w in why:
            print(f"  !! preferred detector unavailable -- {w}")

        # 3. aruco -- last resort, and it cannot decimate.
        try:
            dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            try:                                    # OpenCV >= 4.7
                params = cv2.aruco.DetectorParameters()
                self._d = cv2.aruco.ArucoDetector(dic, params)
                self._mode = "aruco_new"
            except AttributeError:                  # OpenCV 4.x older
                self._d = dic
                self._params = cv2.aruco.DetectorParameters_create()
                self._mode = "aruco_old"
            self.backend = ("cv2.aruco (WEAKER quad detector, and it IGNORES "
                            "decimate -- install python3-apriltag)")
            self.supports_decimate = False
        except Exception as e:
            sys.exit(f"no tag36h11 detector available: {e}\n"
                     f"try:  sudo apt install -y python3-apriltag   (preferred)\n"
                     f"  or: pip install pupil-apriltags\n"
                     f"  or: sudo apt install -y python3-opencv")

    def detect(self, gray):
        if self._mode in ("deb_apriltag", "apriltag"):
            return len(self._d.detect(gray))
        if self._mode == "aruco_new":
            corners, ids, _ = self._d.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._d,
                                                      parameters=self._params)
        return 0 if ids is None else len(ids)


# ------------------------------------------------------------------ helpers ---
def fourcc_str(v):
    v = int(v)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def pct(xs, p):
    return sorted(xs)[min(len(xs) - 1, int(len(xs) * p / 100.0))]


def open_camera(device, width, height, fourcc, raw=False):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    if raw:
        # CONVERT_RGB=0 makes retrieve() hand back the RAW MJPEG buffer instead
        # of a decoded BGR image. We then decode it ourselves straight to
        # grayscale, which skips BOTH the colour reconstruction and the separate
        # cvtColor -- JPEG keeps luma in its own channel, so a grey decode simply
        # does less work rather than doing the same work and discarding it.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    # ORDER MATTERS: fourcc before size. Setting size first can leave the driver
    # locked to a format that has no such size, and the request silently fails.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Keep the driver queue shallow: a deep buffer trades latency for smoothness,
    # which is exactly the wrong trade for a control-loop measurement. Not all
    # V4L2 backends honour this, so it is a request, not a guarantee.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def bench(device, width, height, fourcc, det, n_frames, warmup, save_sample,
          raw_gray=False):
    cap = open_camera(device, width, height, fourcc, raw=raw_gray)
    if cap is None:
        return None

    got_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_cc = fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))

    for _ in range(warmup):          # let exposure/gain settle before timing
        cap.grab()

    t_grab, t_dec, t_gray, t_det, n_tags = [], [], [], [], []
    sample = None
    t_start = time.perf_counter()

    for i in range(n_frames):
        a = time.perf_counter()
        if not cap.grab():
            break
        b = time.perf_counter()
        ok, frame = cap.retrieve()   # MJPEG decode happens here (unless raw)
        c = time.perf_counter()
        if not ok:
            break
        if raw_gray:
            # frame is the raw JPEG byte buffer; decode it straight to grey.
            # The whole cost lands in the 'decode' column, so 'gray' reads ~0 --
            # compare the SUM of decode+gray against the normal path, not the
            # columns individually.
            gray = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                break
            c = time.perf_counter()
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        d = time.perf_counter()
        k = det.detect(gray)
        e = time.perf_counter()

        t_grab.append((b - a) * 1e3)
        t_dec .append((c - b) * 1e3)
        t_gray.append((d - c) * 1e3)
        t_det .append((e - d) * 1e3)
        n_tags.append(k)
        if sample is None:
            sample = frame.copy()

    wall = time.perf_counter() - t_start
    cap.release()

    if not t_grab:
        return None

    if save_sample and sample is not None:
        name = f"sample_{got_w}x{got_h}_{got_cc}.png"
        cv2.imwrite(name, sample)

    total = [g + d_ + gr + dt for g, d_, gr, dt in zip(t_grab, t_dec, t_gray, t_det)]
    return {
        "req": f"{width}x{height} {fourcc}",
        "got": f"{got_w}x{got_h} {got_cc}",
        "fps": len(t_grab) / wall,
        "grab": (np.mean(t_grab), pct(t_grab, 95)),
        "dec":  (np.mean(t_dec),  pct(t_dec, 95)),
        "gray": (np.mean(t_gray), pct(t_gray, 95)),
        "det":  (np.mean(t_det),  pct(t_det, 95)),
        "tot":  (np.mean(total),  pct(total, 95), max(total)),
        "tags": (np.mean(n_tags), max(n_tags)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--warmup", type=int, default=30)
    args = ap.parse_args()

    # The YUYV row is here to MEASURE the MJPEG argument rather than assert it.
    # The two quad_decimate values are the single biggest detection-cost knob on
    # a CPU this size -- decimate 2 roughly quarters the pixels the quad
    # detector walks, at the cost of range.
    # 2026-08-20: the two winning knobs turned out to be INDEPENDENT, so the
    # interesting cell is the combination -- YUYV skips MJPEG's ~50 ms decode
    # while decimate cuts detection ~3.5x. Neither touches the other's stage.
    #
    # Resolutions below 720p are kept only as reference: they run faster but
    # DETECT ONLY 2 OF 3 TAGS -- the 4 cm flanking tags fall under the detection
    # floor, and those are what resolve the planar yaw ambiguity (T30). Speed
    # bought by losing them is not a saving.
    # 2026-08-20, after the YUYV result came in BACKWARDS from the prediction:
    # YUYV at 720p is capped ~5.9 fps by USB BANDWIDTH, not by the 10 fps the
    # camera advertises. An uncompressed 720p frame is 1.84 MB and takes ~170 ms
    # to cross the wire, which is both a rate cap and pure latency -- and it is
    # INVISIBLE in the profile because it hides inside grab() as blocking. MJPEG
    # moves ~100 KB in ~10 ms and wins on rate and latency both. YUYV dropped.
    #
    # Two knobs left, and they are independent:
    #   raw=True  -> decode MJPEG straight to grey, skipping colour
    #                reconstruction AND the separate cvtColor.
    #   decimate  -> quad detection on a downscaled image; payload decode and
    #                corner refinement stay full-res, so pose accuracy holds.
    #                Costs detection RANGE, and the `tags` column is the alarm:
    #                if it falls to 2, the 4 cm flanking tags are gone and we
    #                have decimated past what the yaw solution needs (T30).
    #
    # (w, h, fourcc, decimate, raw_gray_decode)
    modes = [
        (1280, 720, "MJPG", 2.0, True),   # baseline winner + grey decode
        (1280, 720, "MJPG", 3.0, True),   # how far can decimate go before
        (1280, 720, "MJPG", 4.0, True),   #   the tags column drops to 2?
        (1280, 720, "MJPG", 2.0, False),  # control: the current best, unchanged
        (1280, 720, "MJPG", 1.0, True),   # reference
    ]

    print(f"device {args.device}   {args.frames} frames per mode "
          f"(+{args.warmup} warmup)\n")

    seen_backend = None
    rows = []
    saved = set()
    skipped_dq = False
    for (w, h, cc, dq, raw) in modes:
        det = Detector(quad_decimate=dq)
        if seen_backend != det.backend:
            print(f"detector: {det.backend}\n")
            seen_backend = det.backend
        # Do not run a decimated row on a backend that ignores decimation -- it
        # would be the same run twice under two different labels, which is worse
        # than no data because it reads as "decimation does nothing".
        if dq != 1.0 and not det.supports_decimate:
            print(f"  {w}x{h} {cc} dq={dq}: SKIPPED -- this backend ignores "
                  f"decimate, the row would duplicate dq=1.0")
            skipped_dq = True
            continue
        key = (w, h, cc)
        r = bench(args.device, w, h, cc, det, args.frames, args.warmup,
                  save_sample=key not in saved and not raw, raw_gray=raw)
        saved.add(key)
        tag = f"dq={dq}{' grey' if raw else ''}"
        if r is None:
            print(f"  {w}x{h} {cc} {tag}: FAILED to open or capture")
            continue
        r["dq"] = dq
        r["got"] = r["got"] + (" grey" if raw else "")
        rows.append(r)
        print(f"  {r['req']} {tag}  ->  {r['fps']:.1f} fps  "
              f"({r['tot'][0]:.0f} ms, {r['tags'][1]:.0f} tags)")

    print("\n" + "=" * 78)
    print(f"{'mode':<22}{'dq':>4}{'fps':>7}{'grab':>8}{'decode':>8}"
          f"{'gray':>7}{'detect':>8}{'total':>8}{'tags':>6}")
    print("-" * 78)
    for r in rows:
        print(f"{r['got']:<22}{r['dq']:>4.1f}{r['fps']:>7.1f}"
              f"{r['grab'][0]:>8.1f}{r['dec'][0]:>8.1f}"
              f"{r['gray'][0]:>7.1f}{r['det'][0]:>8.1f}"
              f"{r['tot'][0]:>8.1f}{r['tags'][1]:>6.0f}")
    print("=" * 78)
    print("all times ms, MEAN. tags = max seen in any frame.")
    print("'grey' rows decode MJPEG straight to grayscale, so their whole cost")
    print("lands in 'decode' and 'gray' reads ~0 -- compare decode+gray as a SUM.")
    print("\n*** WATCH THE tags COLUMN. It must stay at 3. A row showing 2 has")
    print("    decimated past the point where the 4 cm flanking tags detect, and")
    print("    those are what resolve the yaw ambiguity (T30). Speed bought by")
    print("    losing a tag is not speed, it is a broken terminal approach.")
    print("\np95 / worst-case total (this is what age_us has to survive):")
    for r in rows:
        print(f"  {r['got']:<22} dq={r['dq']:.1f}  "
              f"p95 {r['tot'][1]:6.1f} ms   max {r['tot'][2]:6.1f} ms")

    print("\nSample frames written to sample_<W>x<H>_<FOURCC>.png -- compare the")
    print("720p and 640x480 images: if 640x480 shows MORE vertical scene, the")
    print("16:9 mode is a vertical crop; if it shows the SAME scene squashed, it")
    print("is a resize. That decides whether the 4:3 modes are usable at all,")
    print("and either way each mode needs its OWN calibration (T14).")
    print("\nReminder: these times start at cap.grab(). Sensor exposure and USB")
    print("transit are NOT included -- true capture->pose latency is higher by a")
    print("constant we have not measured yet.")
    if skipped_dq:
        print("\n*** Decimation rows were SKIPPED: this backend ignores them.")
        print("    decimate is the single biggest cost knob (measured 3.6x).")
        print("    Install the reference detector and re-run:")
        print("        sudo apt install -y python3-apriltag")
    if seen_backend and "aruco" in seen_backend:
        print("\n*** Ran on cv2.aruco. These numbers are NOT comparable with any")
        print("    run taken on the reference detector -- they are ~4x apart.")


if __name__ == "__main__":
    main()
