#!/usr/bin/env python3
"""
filter_calibration.py

Turns a raw calibration_run_* folder (written by capture_calibration.py) into
a filtered/ copy with sensor corrections applied and derived columns added.
Raw files are never modified, so reruns are safe.

Handles the current firmware format (repeats, measured wheel angle, INA219
current, staircase mode) and still reads the older pre-repeats format.

WHAT IT CORRECTS:

  GYRO BIAS. The MPU6050 reads a small nonzero rate at rest (~0.42 dps in
  this rig). Left in, it integrates into rotation that never happened. The
  bias is measured from the phase-A windows of tests that start from rest
  (from=0.00V) -- the only windows where the platform is provably
  undisturbed. Tests that spin up first are excluded; their phase A holds
  real motion. The firmware now also reports its own startup measurement as
  gyro_bias_dps in each file's metadata, and this script cross-checks the
  two and warns if they disagree meaningfully (which would suggest thermal
  drift over the sweep, or that the rig wasn't actually still at startup).

  WHEEL/PLATFORM SIGN CONVENTION. Wheel velocity and gyro rate come out
  positively correlated in raw data, which would mean both bodies turning
  the same way. They aren't -- the encoder's positive direction and the
  IMU's positive gyro-Z are defined oppositely by how they're mounted.
  --flip-gyro (default on) negates the gyro so both share a convention.

  WHEEL ANGLE ZEROING. The encoder reports absolute accumulated angle that
  does not reset between tests, so each test starts from whatever the last
  one left behind. Each test's angle is re-zeroed to its own first sample.

WHAT IT ADDS:
  t_s              seconds since the start of that test
  phase            'A' (pre-step hold) or 'B' (post-step transient)
  wheel_angle_rad  measured encoder angle, zeroed per test
  wheel_accel      d(wheel_vel)/dt -- sets reaction torque on the platform
  gyro_dps         bias-corrected (and optionally sign-flipped) rate
  platform_deg     integrated gyro_dps -- platform heading, degrees
  power_mW         busV * current_mA
  iq_est_A         estimated q-axis current, see below

IQ ESTIMATE -- READ BEFORE TRUSTING IT. The INA219 is on the DC bus, not
the phase wires, so Iq is not measured directly. In voltage mode SimpleFOC
drives Ud ~= 0, so essentially all electrical power is Uq*Iq and must come
from the bus:  V_bus*I_bus ~= Uq*Iq  ->  Iq ~= V_bus*I_bus/Uq.
This neglects switching and conduction losses (a few percent) and blows up
as Uq approaches zero. Samples where |targetV| < --iq-min-v (default 0.15 V)
are left as NaN rather than reporting a huge meaningless number -- that
includes every release-to-zero test's phase B, by design.

STAIRCASE FILES are detected by mode=staircase in the metadata and passed
through with bias correction and derived columns, but no phase split (there
is no single step). breakaway_summary.csv reports, per staircase file, the
first tread where platform motion leaves the noise floor going up and where
it re-sticks coming down.

USAGE:
    python filter_calibration.py calibration_run_20260730_101500
    python filter_calibration.py <run_folder> --no-flip-gyro
    python filter_calibration.py <run_folder> --bias 0.42 --iq-min-v 0.2
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
    print("This script needs numpy and pandas:  py -m pip install numpy pandas")
    sys.exit(1)

RE_TITLE = re.compile(r"^#\s*test\s+(\d+)/(\d+):\s*(.*)$")


def parse_meta_line(line):
    """Pull key=value pairs out of the metadata comment line.

    Deliberately generic: the firmware's metadata fields change as the sweep
    evolves, and an exact-match regex on that line is what silently broke
    capture once already. Anything of the form key=value is picked up;
    unknown keys are simply ignored.
    """
    out = {}
    for m in re.finditer(r"(\w+)=([^\s]+)", line):
        out[m.group(1)] = m.group(2)
    return out


def read_test(path):
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = RE_TITLE.match(line)
            if m:
                meta["test_num"] = int(m.group(1))
                meta["test_total"] = int(m.group(2))
                meta["label"] = m.group(3)
            meta.update(parse_meta_line(line[1:]))
    df = pd.read_csv(path, comment="#")

    # Normalise column names across firmware revisions. The older sweep
    # logged vel_filtered (plus an unused vel_raw); the current one logs
    # wheel_vel. Everything downstream uses wheel_vel.
    if "wheel_vel" not in df.columns and "vel_filtered" in df.columns:
        df = df.rename(columns={"vel_filtered": "wheel_vel"})
    return df, meta


def fnum(meta, key, default=np.nan):
    try:
        return float(str(meta.get(key, default)).rstrip("V"))
    except (TypeError, ValueError):
        return default


def is_staircase(meta):
    return str(meta.get("mode", "")).lower() == "staircase"


def is_breakaway(meta):
    """Breakaway trials are structurally from-rest step tests (they have the
    same phase A/B split), so they flow through the normal path -- this is
    only used to build the extra breakaway summary at the end."""
    return str(meta.get("mode", "")).lower() == "breakaway"


def transition_index(df, meta):
    """First phase-B sample. Prefer the firmware's own index; fall back to
    detecting where targetV changes (needed for older files)."""
    if "phaseB_start_sample" in meta:
        try:
            return int(meta["phaseB_start_sample"])
        except ValueError:
            pass
    tv = df["targetV"].to_numpy()
    for i in range(1, len(tv)):
        if tv[i] != tv[i - 1]:
            return i
    return len(tv)


def measure_bias(files):
    """Average gyro bias from the phase-A windows of at-rest step tests."""
    samples, used = [], []
    for path in files:
        df, meta = read_test(path)
        if is_staircase(meta):
            continue
        if abs(fnum(meta, "from", 1.0)) > 1e-9:
            continue
        ti = transition_index(df, meta)
        if ti < 10:
            continue
        seg = df["gyroZ_dps"].to_numpy()[:ti]
        samples.append(seg)
        used.append((os.path.basename(path), seg.mean(), seg.std()))
    if not samples:
        return None, []
    return float(np.concatenate(samples).mean()), used


def integrate(y, t):
    out = np.zeros(len(y))
    if len(y) < 2:
        return out
    out[1:] = np.cumsum((y[1:] + y[:-1]) / 2.0 * np.diff(t))
    return out


def find_breakaway(df, bias, settle_dps):
    """First tread where platform motion leaves the noise floor (climbing)
    and where it settles back (descending)."""
    v = df["targetV"].to_numpy()
    g = np.abs(df["gyro_dps"].to_numpy())
    peak_v = v.max()
    up = np.arange(len(v)) <= int(np.argmax(v))
    moving = g > settle_dps
    bo = v[up & moving]
    dn = v[~up & moving]
    return (float(bo.min()) if len(bo) else np.nan,
            float(dn.min()) if len(dn) else np.nan,
            float(peak_v))


def main():
    ap = argparse.ArgumentParser(description="Apply sensor corrections to a raw calibration run.")
    ap.add_argument("run_folder")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--flip-gyro", dest="flip", action="store_true", default=True,
                    help="Negate gyro so wheel and platform share a sign convention (default)")
    ap.add_argument("--no-flip-gyro", dest="flip", action="store_false")
    ap.add_argument("--bias", type=float, default=None, help="Override measured gyro bias, dps")
    ap.add_argument("--iq-min-v", type=float, default=0.15,
                    help="Below this |targetV|, iq_est_A is left NaN (default 0.15)")
    ap.add_argument("--settle-dps", type=float, default=3.0,
                    help="Motion threshold for staircase breakaway detection (default 3.0)")
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
            print("No at-rest (from=0.00V) step tests found to measure bias from.")
            print("Pass --bias <dps>, or include at least one from-rest test in the sweep.")
            sys.exit(1)
        print(f"Gyro bias measured from {len(used)} at-rest test(s): {bias:+.4f} dps")
        if used:
            spread = max(m for _, m, _ in used) - min(m for _, m, _ in used)
            print(f"  spread across those tests: {spread:.4f} dps")

    # Cross-check against the firmware's own startup measurement.
    fw = []
    for path in files:
        _, meta = read_test(path)
        if "gyro_bias_dps" in meta:
            try:
                fw.append(float(meta["gyro_bias_dps"]))
            except ValueError:
                pass
    if fw:
        fwb = float(np.mean(fw))
        print(f"Firmware-reported bias: {fwb:+.4f} dps  (difference {abs(fwb-bias):.4f})")
        if abs(fwb - bias) > 0.15:
            print("  ! These disagree more than expected. Possible causes: the rig wasn't")
            print("    still during the firmware's startup measurement, or the gyro drifted")
            print("    over the sweep. The value measured here (from at-rest windows) is used.")

    print(f"Gyro sign flip: {'ON' if args.flip else 'OFF'}")
    print()

    rows, stair_rows = [], []
    brk_rows = []
    n_angle_flips = [0]   # list so the per-test loop can increment it

    for path in files:
        df, meta = read_test(path)
        stair = is_staircase(meta)
        name = os.path.basename(path)

        t_us = df["t_us"].to_numpy().astype(np.float64)
        t_s = (t_us - t_us[0]) / 1e6
        wheel_vel = df["wheel_vel"].to_numpy().astype(np.float64)
        gyro_raw = df["gyroZ_dps"].to_numpy().astype(np.float64)

        gyro = gyro_raw - bias
        if args.flip:
            gyro = -gyro

        # Encoder angle is absolute and does not reset between tests -- zero
        # it per test. Older files without the column fall back to
        # integrating velocity.
        #
        # SIGN: sensor.getAngle() returns the RAW encoder direction, while
        # motor.shaft_velocity has the direction correction found during
        # initFOC() alignment applied to it. On this rig they come out
        # opposite (d(angle)/dt tracks -wheel_vel at slope ~-0.98 in every
        # test), exactly like the old vel_raw/vel_filtered mirror. Left
        # uncorrected, angle and velocity would disagree about which way the
        # wheel is turning, which would silently corrupt anything using both.
        # Detected per test rather than assumed, so a firmware change that
        # switches to motor.shaft_angle won't break this.
        angle_flipped = False
        if "wheel_angle_rad" in df.columns:
            wa = df["wheel_angle_rad"].to_numpy().astype(np.float64)
            wheel_angle = wa - wa[0]
            moving = np.abs(wheel_vel) > 1.0
            if moving.sum() >= 10:
                dadt = np.gradient(wheel_angle, t_s)
                slope = np.polyfit(wheel_vel[moving], dadt[moving], 1)[0]
                if slope < 0:
                    wheel_angle = -wheel_angle
                    angle_flipped = True
                    n_angle_flips[0] += 1
        else:
            wheel_angle = integrate(wheel_vel, t_s)

        wheel_accel = np.gradient(wheel_vel, t_s)
        platform_deg = integrate(gyro, t_s)

        out = pd.DataFrame({
            "t_s": np.round(t_s, 6),
            "targetV": df["targetV"].to_numpy(),
            "wheel_vel": np.round(wheel_vel, 4),
            "wheel_angle_rad": np.round(wheel_angle, 4),
            "wheel_accel": np.round(wheel_accel, 3),
            "gyro_dps_raw": np.round(gyro_raw, 4),
            "gyro_dps": np.round(gyro, 4),
            "platform_deg": np.round(platform_deg, 4),
        })

        if not stair:
            ti = transition_index(df, meta)
            out.insert(1, "phase", np.where(np.arange(len(df)) < ti, "A", "B"))

        if "current_mA" in df.columns and "busV" in df.columns:
            cur = df["current_mA"].to_numpy().astype(np.float64)
            bus = df["busV"].to_numpy().astype(np.float64)
            tv = df["targetV"].to_numpy().astype(np.float64)
            out["current_mA"] = np.round(cur, 3)
            out["busV"] = np.round(bus, 4)
            out["power_mW"] = np.round(bus * cur, 3)
            # Iq from power balance; masked where Uq is too small to divide by.
            with np.errstate(divide="ignore", invalid="ignore"):
                iq = np.where(np.abs(tv) >= args.iq_min_v, bus * (cur / 1000.0) / tv, np.nan)
            out["iq_est_A"] = np.round(iq, 5)

        outpath = os.path.join(outdir, name)
        with open(outpath, "w", newline="") as f:
            f.write(f"# test {meta.get('test_num','?')}/{meta.get('test_total','?')}: "
                    f"{meta.get('label','')}\n")
            keep = [k for k in ("from", "to", "rep", "mode", "step_v", "max_v",
                                "dwell_ms", "phaseA_clean", "stop_reason") if k in meta]
            f.write("# " + " ".join(f"{k}={meta[k]}" for k in keep) + "\n")
            f.write(f"# FILTERED: gyro_bias_removed={bias:+.4f}dps gyro_sign_flipped={args.flip} "
                    f"wheel_angle_sign_flipped={angle_flipped} iq_min_v={args.iq_min_v}\n")
            out.to_csv(f, index=False)

        if stair:
            bo, rs, pk = find_breakaway(out, bias, args.settle_dps)
            stair_rows.append({"file": name, "breakaway_V": bo, "restick_V": rs,
                               "max_V": pk, "n": len(out)})
            print(f"  wrote {outpath}  [staircase] breakaway~{bo:.2f}V restick~{rs:.2f}V")
        else:
            iq_ss = np.nan
            if "iq_est_A" in out.columns:
                tail = out["iq_est_A"].to_numpy()[-20:]
                # All-NaN is expected and correct for release-to-zero tests:
                # targetV is 0 there, so Iq can't be recovered from power
                # balance. nanmean would warn on an all-NaN slice, so check.
                if np.any(np.isfinite(tail)):
                    iq_ss = round(float(np.nanmean(tail)), 5)
            b = out[out["phase"] == "B"]
            gB = b["gyro_dps"].to_numpy()
            peak_gyro = float(gB[np.argmax(np.abs(gB))]) if len(gB) else np.nan
            if is_breakaway(meta):
                brk_rows.append({
                    "step_V": fnum(meta, "to"),
                    "rep": int(meta.get("rep", 1)) if str(meta.get("rep", "1")).isdigit() else 1,
                    "peak_gyro_dps": round(abs(peak_gyro), 3),
                    "peak_wheel_rads": round(float(np.abs(b["wheel_vel"].to_numpy()).max()), 3),
                    "phaseA_clean": meta.get("phaseA_clean", "?"),
                })
            rows.append({
                "test": meta.get("test_num", 0),
                "label": meta.get("label", name),
                "rep": int(meta.get("rep", 1)) if str(meta.get("rep", "1")).isdigit() else 1,
                "from_v": fnum(meta, "from"),
                "to_v": fnum(meta, "to"),
                "phaseA_clean": meta.get("phaseA_clean", "?"),
                "n": len(out),
                "duration_s": round(float(t_s[-1]), 3),
                "wheel_vel_final": round(float(wheel_vel[-20:].mean()), 3),
                "wheel_vel_peak": round(float(wheel_vel[np.argmax(np.abs(wheel_vel))]), 3),
                "platform_net_deg": round(float(platform_deg[-1]), 2),
                "platform_peak_dps": round(float(gyro[np.argmax(np.abs(gyro))]), 2),
                "iq_ss_A": iq_ss,
                "stop_reason": meta.get("stop_reason", "?"),
            })
            print(f"  wrote {outpath}  ({len(out)} rows)")

    if rows:
        summary = pd.DataFrame(rows).sort_values("test")
        summary.to_csv(os.path.join(outdir, "summary.csv"), index=False)
        print(f"\n  wrote {os.path.join(outdir,'summary.csv')}")

        if n_angle_flips[0]:
            print(f"\n  note: wheel_angle_rad sign was inverted relative to wheel_vel in "
                  f"{n_angle_flips[0]} test(s) and has been corrected.")
            print("    sensor.getAngle() reports raw encoder direction; motor.shaft_velocity")
            print("    has the initFOC() alignment correction applied. Logging")
            print("    motor.shaft_angle instead would fix this at the source.")

        bad = summary[summary["phaseA_clean"] == "no"]
        if len(bad):
            print(f"\n  ! {len(bad)} test(s) reported phaseA_clean=no -- the platform was still")
            print("    moving when the step fired, so phase B started from a contaminated")
            print("    initial condition. Consider excluding these from fits:")
            for _, r in bad.iterrows():
                print(f"      test {int(r['test']):3d} rep {int(r['rep'])}  {r['label']}")

        # Repeatability across reps of the same condition.
        if summary["rep"].nunique() > 1:
            grp = summary.groupby(["from_v", "to_v"]).agg(
                n_reps=("rep", "count"),
                wheel_peak_mean=("wheel_vel_peak", "mean"),
                wheel_peak_std=("wheel_vel_peak", "std"),
                platform_net_mean=("platform_net_deg", "mean"),
                platform_net_std=("platform_net_deg", "std"),
            ).reset_index().round(3)
            grp.to_csv(os.path.join(outdir, "repeatability.csv"), index=False)
            print(f"  wrote {os.path.join(outdir,'repeatability.csv')}")
            print()
            print(grp.to_string(index=False))

    if brk_rows:
        B = pd.DataFrame(brk_rows)
        agg = B.groupby("step_V").agg(
            n=("rep", "count"),
            peak_gyro_mean=("peak_gyro_dps", "mean"),
            peak_gyro_std=("peak_gyro_dps", "std"),
            peak_wheel_mean=("peak_wheel_rads", "mean"),
        ).reset_index().round(3)
        agg["platform_moved"] = agg["peak_gyro_mean"] > args.settle_dps
        agg["wheel_moved"] = agg["peak_wheel_mean"] > 0.2
        agg.to_csv(os.path.join(outdir, "breakaway_summary.csv"), index=False)
        print(f"\n  wrote {os.path.join(outdir,'breakaway_summary.csv')}")
        print()
        print(agg.to_string(index=False))

        moved = agg[agg["platform_moved"]]
        wmoved = agg[agg["wheel_moved"]]
        print()
        if len(wmoved):
            print(f"  MOTOR deadband:    wheel first turns at step {wmoved.step_V.iloc[0]:.2f} V")
        if len(moved):
            print(f"  PLATFORM breakaway: first moves at step {moved.step_V.iloc[0]:.2f} V "
                  f"(threshold {args.settle_dps} dps)")
        else:
            print(f"  PLATFORM never exceeded {args.settle_dps} dps -- raise BREAK_MAX_V and rerun.")

    if stair_rows:
        sdf = pd.DataFrame(stair_rows)
        sdf.to_csv(os.path.join(outdir, "breakaway_summary.csv"), index=False)
        print(f"\n  wrote {os.path.join(outdir,'breakaway_summary.csv')}")
        print(sdf.to_string(index=False))


if __name__ == "__main__":
    main()