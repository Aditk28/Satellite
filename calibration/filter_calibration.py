#!/usr/bin/env python3
"""
filter_calibration.py

Turns a raw calibration_run_* folder (written by capture_calibration.py) into
a filtered/ copy with the sensor corrections applied and useful derived
columns added. Raw files are never modified -- reruns are safe and
repeatable, which is the whole point of the calibration sweep being
automated in the first place.

WHAT IT CORRECTS, AND WHY:

  1. GYRO BIAS. The MPU6050 reads a small nonzero rate even at rest --
     measured at roughly +0.42 dps in this rig, consistent to within
     0.01 dps across every at-rest test. That's normal MEMS behavior, not a
     fault, but left uncorrected it integrates straight into fake platform
     rotation: ~0.4 dps over a 5 s test is ~2 degrees of drift that never
     happened.

     The bias is measured automatically from the PHASE A WINDOW OF TESTS
     THAT START FROM REST (from=0.00V in the file's metadata header). Those
     are the only windows where the platform is provably undisturbed --
     tests that spin up first (from!=0V) have real leftover platform motion
     in their phase A, and in at least one case (the 3.0V hold) a genuine
     decaying oscillation that crosses zero. Averaging those in would
     corrupt the bias estimate, so they're excluded by construction.

  2. WHEEL/PLATFORM SIGN CONVENTION. sensor.getVelocity() (vel_raw) and
     motor.shaft_velocity (vel_filtered) are near-perfect mirrors of each
     other -- SimpleFOC applies the direction correction found during
     initFOC() alignment to shaft_velocity, so vel_filtered is the
     canonical, physically-consistent wheel velocity. This script uses
     vel_filtered and ignores vel_raw.

     Separately: wheel velocity and platform gyro rate come out POSITIVELY
     correlated in the raw data (+0.33 to +0.84 in every test). Newton's
     third law says a wheel accelerating one way pushes the platform the
     other way, so a positive correlation just means the encoder's positive
     direction and the IMU's positive gyro-Z direction are defined
     oppositely relative to each other -- a fixed mounting/axis convention,
     not a physics violation. --flip-gyro (ON by default) negates the gyro
     channel so both agree on which way is positive, which is what you want
     before any momentum bookkeeping or before telling a controller which
     way to spin the wheel to correct a given platform error.

WHAT IT ADDS (derived columns):
     t_s              seconds since the start of that test
     phase            'A' (pre-step hold) or 'B' (post-step transient)
     wheel_vel        canonical wheel velocity (from vel_filtered)
     wheel_angle_rad  integrated wheel velocity
     gyro_dps         bias-corrected (and optionally sign-flipped) rate
     platform_deg     integrated gyro_dps -- platform heading, degrees

USAGE:
    python filter_calibration.py calibration_run_20260729_112929
    python filter_calibration.py <run_folder> --no-flip-gyro
    python filter_calibration.py <run_folder> --outdir somewhere_else

Output lands in <run_folder>/filtered/ unless --outdir says otherwise.
"""

import argparse
import glob
import os
import re
import sys

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("This script needs numpy and pandas:  pip install numpy pandas")
    sys.exit(1)

RE_META = re.compile(r"^#\s*from=([-\d.]+)V\s+to=([-\d.]+)V\s+hold=(\d+)ms\s+stop_reason=(\w+)")
RE_TITLE = re.compile(r"^#\s*test\s+(\d+)/(\d+):\s*(.*)$")


def read_test(path):
    """Read one calibration CSV, returning (dataframe, metadata dict)."""
    meta = {}
    with open(path, "r") as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = RE_TITLE.match(line)
            if m:
                meta["test_num"], meta["test_total"], meta["label"] = int(m.group(1)), int(m.group(2)), m.group(3)
            m = RE_META.match(line)
            if m:
                meta["from_v"] = float(m.group(1))
                meta["to_v"] = float(m.group(2))
                meta["hold_ms"] = int(m.group(3))
                meta["stop_reason"] = m.group(4)
    df = pd.read_csv(path, comment="#")
    return df, meta


def transition_index(df):
    """Index of the first sample after targetV changes (phase A -> B)."""
    tv = df["targetV"].to_numpy()
    for i in range(1, len(tv)):
        if tv[i] != tv[i - 1]:
            return i
    return len(tv)  # no transition found -- treat everything as phase A


def measure_bias(files):
    """Average gyro bias from the phase-A windows of at-rest tests only.

    At-rest means from=0.00V in the metadata: the platform is provably
    undisturbed for that whole window. Tests that spin up first are
    excluded -- their phase A contains real motion.
    """
    samples = []
    used = []
    for path in files:
        df, meta = read_test(path)
        if abs(meta.get("from_v", 0.0)) > 1e-9:
            continue  # not an at-rest test, skip for bias purposes
        ti = transition_index(df)
        if ti < 10:
            continue
        seg = df["gyroZ_dps"].to_numpy()[:ti]
        samples.append(seg)
        used.append((os.path.basename(path), seg.mean(), seg.std()))
    if not samples:
        return None, []
    allsamples = np.concatenate(samples)
    return float(allsamples.mean()), used


def integrate(y, t):
    """Cumulative trapezoidal integral of y over t, starting at 0."""
    out = np.zeros(len(y))
    if len(y) < 2:
        return out
    dt = np.diff(t)
    incr = (y[1:] + y[:-1]) / 2.0 * dt
    out[1:] = np.cumsum(incr)
    return out


def main():
    ap = argparse.ArgumentParser(description="Apply sensor corrections to a raw calibration run.")
    ap.add_argument("run_folder", help="Folder holding the raw test*.csv files")
    ap.add_argument("--outdir", default=None, help="Where to write filtered copies (default: <run_folder>/filtered)")
    ap.add_argument("--flip-gyro", dest="flip", action="store_true", default=True,
                    help="Negate gyro so wheel and platform share a sign convention (default)")
    ap.add_argument("--no-flip-gyro", dest="flip", action="store_false",
                    help="Leave the gyro sign exactly as recorded")
    ap.add_argument("--bias", type=float, default=None,
                    help="Override the measured gyro bias, in dps")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.run_folder, "test*.csv")))
    if not files:
        print(f"No test*.csv files found in {args.run_folder}")
        sys.exit(1)

    outdir = args.outdir or os.path.join(args.run_folder, "filtered")
    os.makedirs(outdir, exist_ok=True)

    if args.bias is not None:
        bias, used = args.bias, []
        print(f"Using bias override: {bias:+.4f} dps")
    else:
        bias, used = measure_bias(files)
        if bias is None:
            print("Could not find any at-rest (from=0.00V) tests to measure bias from.")
            print("Pass --bias <dps> explicitly, or include at least one from-rest test in the sweep.")
            sys.exit(1)
        print(f"Gyro bias measured from {len(used)} at-rest test(s):")
        for name, m, s in used:
            print(f"   {name:62s} mean={m:+.4f} std={s:.4f}")
        print(f"  -> using bias = {bias:+.4f} dps")

    print(f"Gyro sign flip: {'ON (wheel and platform share a convention)' if args.flip else 'OFF (as recorded)'}")
    print()

    summary_rows = []

    for path in files:
        df, meta = read_test(path)
        ti = transition_index(df)

        t_us = df["t_us"].to_numpy().astype(np.float64)
        t_s = (t_us - t_us[0]) / 1e6

        wheel_vel = df["vel_filtered"].to_numpy().astype(np.float64)
        gyro_raw = df["gyroZ_dps"].to_numpy().astype(np.float64)

        gyro = gyro_raw - bias
        if args.flip:
            gyro = -gyro

        wheel_angle = integrate(wheel_vel, t_s)
        platform_deg = integrate(gyro, t_s)

        phase = np.where(np.arange(len(df)) < ti, "A", "B")

        out = pd.DataFrame({
            "t_s": np.round(t_s, 6),
            "phase": phase,
            "targetV": df["targetV"].to_numpy(),
            "wheel_vel": np.round(wheel_vel, 4),
            "wheel_angle_rad": np.round(wheel_angle, 4),
            "gyro_dps_raw": np.round(gyro_raw, 4),
            "gyro_dps": np.round(gyro, 4),
            "platform_deg": np.round(platform_deg, 4),
        })

        name = os.path.basename(path)
        outpath = os.path.join(outdir, name)
        with open(outpath, "w", newline="") as f:
            f.write(f"# test {meta.get('test_num','?')}/{meta.get('test_total','?')}: {meta.get('label','')}\n")
            f.write(f"# from={meta.get('from_v',float('nan')):.2f}V to={meta.get('to_v',float('nan')):.2f}V "
                    f"hold={meta.get('hold_ms','?')}ms stop_reason={meta.get('stop_reason','?')}\n")
            f.write(f"# FILTERED: gyro_bias_removed={bias:+.4f}dps  gyro_sign_flipped={args.flip}\n")
            f.write(f"# phase A samples={ti}  phase B samples={len(df)-ti}\n")
            out.to_csv(f, index=False)

        b = out[out["phase"] == "B"]
        summary_rows.append({
            "test": meta.get("test_num", 0),
            "label": meta.get("label", name),
            "from_v": meta.get("from_v", np.nan),
            "to_v": meta.get("to_v", np.nan),
            "n": len(out),
            "duration_s": round(float(t_s[-1]), 3),
            "wheel_vel_final": round(float(wheel_vel[-20:].mean()), 3),
            "wheel_vel_peak": round(float(wheel_vel[np.argmax(np.abs(wheel_vel))]), 3),
            "platform_net_deg": round(float(platform_deg[-1]), 2),
            "platform_peak_dps": round(float(gyro[np.argmax(np.abs(gyro))]), 2),
            "stop_reason": meta.get("stop_reason", "?"),
        })
        print(f"  wrote {outpath}  ({len(out)} rows, phase A={ti} / B={len(df)-ti})")

    summary = pd.DataFrame(summary_rows).sort_values("test")
    spath = os.path.join(outdir, "summary.csv")
    summary.to_csv(spath, index=False)
    print(f"\n  wrote {spath}")
    print()
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()