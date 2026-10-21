#!/usr/bin/env python3
"""
Shared camera setup: exposure lock and intrinsics loading. Imported by the Pi
scripts that open the camera.

The calibration is only valid at the exposure it was captured with, so the
calibrator and the pose pipeline share one exposure constant from here. v4l2
controls reset when the camera is reopened or the Pi reboots, so exposure is
applied from code on every run and read back. v4l2-ctl is used instead of
OpenCV's CAP_PROP_EXPOSURE because OpenCV's mapping to UVC controls is
driver-dependent.
"""

import json
import os
import subprocess

import numpy as np

INTRINSICS_FILE = "camera_intrinsics.json"


def load_intrinsics(width, height, path=None, verbose=True):
    """Load the calibration written by pi_calibrate.py.

    Returns (K, dist, source). K and dist are None if there is no file or it
    was made at a different resolution. This camera's 4:3 modes are not a
    downscale of 720p, so intrinsics do not transfer between modes.
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
            print(f"  !! no {INTRINSICS_FILE}, using the measured focal length "
                  f"with a centred principal point and no distortion. Bearing "
                  f"will be biased and rel-yaw will wander off-centre.")
        return None, None, "uncalibrated"

    if int(d["width"]) != int(width) or int(d["height"]) != int(height):
        if verbose:
            print(f"  !! {INTRINSICS_FILE} is for {d['width']}x{d['height']} "
                  f"but the camera is {width}x{height}. Intrinsics do not "
                  f"scale between this camera's modes. Not using it.")
        return None, None, "uncalibrated (resolution mismatch)"

    K = np.array(d["camera_matrix"], dtype=np.float64)
    dist = np.array(d["dist_coeffs"], dtype=np.float64).reshape(1, -1)
    if verbose:
        print(f"  intrinsics: {p}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
              f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}  "
              f"RMS {d.get('rms_reproj_px', float('nan')):.3f} px  "
              f"({d.get('n_views','?')} views)")
    return K, dist, "calibrated"

# UVC auto_exposure: 1 = manual, 3 = auto.
AUTO_EXPOSURE_MANUAL = 1

# exposure_time_absolute is in 100 us units, so 100 = 10 ms. Short enough for
# 30 fps with far less motion blur than auto-exposure, and still detects all
# 3 tags at working range with added light. Re-run calibration if you change it.
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
            print("  !! v4l2-ctl not found, exposure not locked")
        return False
    except subprocess.CalledProcessError as e:
        if verbose:
            print(f"  !! could not set exposure: {e.stderr.decode().strip()}")
        return False

    # Read back: some drivers accept the set and then clamp or ignore it.
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
            print("  !! exposure is still automatic. A calibration taken now "
                  "would be invalid and range will drift with lighting.")
    return ok
