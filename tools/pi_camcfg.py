#!/usr/bin/env python3
"""
Shared camera configuration. Imported by every script that opens the camera.

WHY THIS IS ITS OWN MODULE AND NOT A LINE IN EACH SCRIPT. The calibration is
only valid at the exposure it was captured with, because exposure changes the
effective focal length (T35). So the calibrator and the production pose pipeline
MUST use the same value -- and a constant that has to match in two places is
precisely the thing that quietly drifts apart. One definition, imported.

WHY NOT SET IT ONCE BY HAND. v4l2 controls do not persist. They reset when the
camera is re-opened or the Pi reboots, and the failure is silent: auto-exposure
comes back, the frame rate halves, motion blur returns, and the intrinsics no
longer describe the lens. Observed 2026-08-20 -- a manual lock set earlier in the
session had reverted to `auto_exposure: 3 (Aperture Priority)` by the time
calibration was about to run. Apply it from code, every time, and verify.

WHY v4l2-ctl RATHER THAN OpenCV's CAP_PROP_EXPOSURE. OpenCV's mapping onto UVC
controls is driver-dependent and inconsistent about units and auto/manual
semantics. v4l2-ctl talks to the driver directly and reports what actually stuck.
"""

import json
import os
import subprocess

import numpy as np

INTRINSICS_FILE = "camera_intrinsics.json"


def load_intrinsics(width, height, path=None, verbose=True):
    """Load the calibration produced by pi_calibrate.py.

    Returns (K, dist, source) where source names what was actually used, so a
    caller can print it -- silently falling back to guessed intrinsics is how a
    session gets spent debugging a pose pipeline that was never calibrated.

    THE RESOLUTION CHECK IS NOT PEDANTRY (T14). This camera's 4:3 modes are a
    different aspect ratio, not a downscale of 720p, so intrinsics do not
    transfer between them -- and a mismatched principal point produces a
    bearing bias that looks exactly like a mechanical misalignment.
    """
    p = path or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             INTRINSICS_FILE)
    if not os.path.exists(p):
        p = INTRINSICS_FILE                     # also try the cwd
    try:
        with open(p) as f:
            d = json.load(f)
    except Exception:
        if verbose:
            print(f"  !! no {INTRINSICS_FILE} -- falling back to a MEASURED "
                  f"focal length with principal point assumed centred and NO "
                  f"distortion model. Bearing will be biased and rel-yaw will "
                  f"wander as the target moves across the frame.")
        return None, None, "uncalibrated"

    if int(d["width"]) != int(width) or int(d["height"]) != int(height):
        if verbose:
            print(f"  !! {INTRINSICS_FILE} is for {d['width']}x{d['height']} "
                  f"but the camera is {width}x{height}. Intrinsics do NOT "
                  f"scale between this camera's modes (T14). Not using it.")
        return None, None, "uncalibrated (resolution mismatch)"

    K = np.array(d["camera_matrix"], dtype=np.float64)
    dist = np.array(d["dist_coeffs"], dtype=np.float64).reshape(1, -1)
    if verbose:
        print(f"  intrinsics: {p}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
              f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}  "
              f"RMS {d.get('rms_reproj_px', float('nan')):.3f} px  "
              f"({d.get('n_views','?')} views)")
    return K, dist, "calibrated"

# UVC: auto_exposure 1 = MANUAL, 3 = auto. Counter-intuitive, and worth stating
# because reading `1` as "auto on" is an easy way to get this exactly backwards.
AUTO_EXPOSURE_MANUAL = 1

# exposure_time_absolute is in 100 us units. 100 -> 10 ms.
#   - short enough for the camera to deliver 30 fps
#   - ~6x less motion blur than the ~66 ms auto-exposure was choosing
#   - verified to still detect all 3 tags at working range WITH added light
# Lower this only if you add more light and re-verify tag detection at MAXIMUM
# range, and re-run calibration afterwards.
EXPOSURE_100US = 100


def lock_exposure(device="/dev/video0", exposure=EXPOSURE_100US, verbose=True):
    """Force manual exposure. Returns True if the driver confirms it stuck."""
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", device,
             "-c", f"auto_exposure={AUTO_EXPOSURE_MANUAL}",
             "-c", f"exposure_time_absolute={exposure}"],
            check=True, capture_output=True)
    except FileNotFoundError:
        if verbose:
            print("  !! v4l2-ctl not found -- exposure NOT locked")
        return False
    except subprocess.CalledProcessError as e:
        if verbose:
            print(f"  !! could not set exposure: {e.stderr.decode().strip()}")
        return False

    # Read it back. Setting a control can succeed and still not take effect --
    # some drivers clamp or ignore, and a silent auto-exposure is the failure
    # this whole module exists to prevent.
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", device,
             "--get-ctrl=auto_exposure,exposure_time_absolute"],
            check=True, capture_output=True).stdout.decode()
    except Exception:
        return False

    ok = f"auto_exposure: {AUTO_EXPOSURE_MANUAL}" in out
    if verbose:
        for line in out.strip().splitlines():
            print(f"  {line.strip()}")
        if not ok:
            print("  !! EXPOSURE IS STILL AUTOMATIC -- calibration taken now "
                  "would be invalid, and range will drift with the lighting.")
    return ok
