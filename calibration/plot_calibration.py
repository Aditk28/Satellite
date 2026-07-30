#!/usr/bin/env python3
"""
plot_calibration.py

Static plots from a filtered calibration run. Run filter_calibration.py
first -- this reads the filtered/ folder, not the raw one, so every plot is
already bias-corrected and sign-consistent.

Produces, into <filtered_folder>/plots/:

  test<NN>_*.png     One figure per test, three stacked panels sharing a
                     time axis: commanded voltage, wheel velocity, and
                     platform rate + heading. The phase A/B boundary is
                     marked on all three, since almost every question about
                     this data ("did it settle before the step?", "how long
                     was the transient?") is really a question about where
                     that line falls.

  overview_steps.png  All from-rest step tests overlaid, time-shifted so
                      t=0 is the step itself rather than the start of the
                      recording -- makes the voltage-to-response scaling
                      directly comparable across tests.

  overview_scaling.png  Steady-state wheel velocity and peak platform rate
                        vs commanded voltage. This is the plot that tells
                        you whether a linear model is defensible before you
                        go fit one: points near a straight line through the
                        origin means yes.

USAGE:
    python plot_calibration.py calibration_run_20260729_112929/filtered
    python plot_calibration.py <filtered_folder> --outdir figures
"""

import argparse
import glob
import os
import re
import sys

try:
    import numpy as np
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("This script needs numpy, pandas and matplotlib:")
    print("   pip install numpy pandas matplotlib")
    sys.exit(1)

RE_TITLE = re.compile(r"^#\s*test\s+(\d+)/(\d+):\s*(.*)$")
RE_META = re.compile(r"^#\s*from=([-\d.]+)V\s+to=([-\d.]+)V")

# Palette: the wheel is the warm channel, the platform the cool one, kept
# consistent across every figure so the two are never confused at a glance.
C_WHEEL = "#D98A2B"
C_PLATFORM = "#3C8DBC"
C_ANGLE = "#7B5EA7"
C_CMD = "#4A4A4A"
C_MARK = "#C0392B"


def read_filtered(path):
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = RE_TITLE.match(line)
            if m:
                meta["test_num"], meta["label"] = int(m.group(1)), m.group(3)
            m = RE_META.match(line)
            if m:
                meta["from_v"], meta["to_v"] = float(m.group(1)), float(m.group(2))
    df = pd.read_csv(path, comment="#")
    return df, meta


def step_time(df):
    """Time of the phase A->B transition, in seconds."""
    b = df[df["phase"] == "B"]
    return float(b["t_s"].iloc[0]) if len(b) else None


def plot_one(df, meta, outpath):
    t = df["t_s"].to_numpy()
    ts = step_time(df)

    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True,
                             gridspec_kw={"height_ratios": [0.6, 1, 1], "hspace": 0.12})

    fig.suptitle(f"Test {meta.get('test_num','?')} — {meta.get('label','')}",
                 fontsize=13, fontweight="bold", y=0.965)

    ax = axes[0]
    ax.plot(t, df["targetV"], color=C_CMD, lw=1.6, drawstyle="steps-post")
    ax.set_ylabel("command\n(V)", fontsize=9)
    ax.grid(alpha=0.25)
    ax.axhline(0, color="#999", lw=0.7, ls=":")

    ax = axes[1]
    ax.plot(t, df["wheel_vel"], color=C_WHEEL, lw=1.5)
    ax.set_ylabel("wheel velocity\n(rad/s)", fontsize=9)
    ax.grid(alpha=0.25)
    ax.axhline(0, color="#999", lw=0.7, ls=":")

    ax = axes[2]
    ax.plot(t, df["gyro_dps"], color=C_PLATFORM, lw=1.3, label="platform rate (dps)")
    ax.set_ylabel("platform rate\n(dps)", fontsize=9, color=C_PLATFORM)
    ax.tick_params(axis="y", labelcolor=C_PLATFORM)
    ax.grid(alpha=0.25)
    ax.axhline(0, color="#999", lw=0.7, ls=":")
    ax.set_xlabel("time (s)", fontsize=10)

    ax2 = ax.twinx()
    ax2.plot(t, df["platform_deg"], color=C_ANGLE, lw=1.4, ls="--", label="heading (deg)")
    ax2.set_ylabel("platform heading\n(deg)", fontsize=9, color=C_ANGLE)
    ax2.tick_params(axis="y", labelcolor=C_ANGLE)

    lines = ax.get_lines()[:1] + ax2.get_lines()[:1]
    ax.legend(lines, [l.get_label() for l in lines], loc="best", fontsize=8, framealpha=0.9)

    if ts is not None:
        for a in axes:
            a.axvline(ts, color=C_MARK, lw=1.2, ls="-", alpha=0.75)
        axes[0].annotate(f"step to {meta.get('to_v','?')}V",
                         xy=(ts, axes[0].get_ylim()[1]), xytext=(4, -12),
                         textcoords="offset points", color=C_MARK,
                         fontsize=8.5, fontweight="bold", va="top")

    fig.savefig(outpath, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_overview_steps(tests, outpath):
    """From-rest step tests, aligned so t=0 is the step."""
    sel = [(df, m) for df, m in tests if abs(m.get("from_v", 1)) < 1e-9]
    if not sel:
        return False
    sel.sort(key=lambda x: x[1].get("to_v", 0))

    fig, axes = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True,
                             gridspec_kw={"hspace": 0.13})
    cmap = plt.get_cmap("viridis")

    for i, (df, m) in enumerate(sel):
        ts = step_time(df) or 0.0
        t = df["t_s"].to_numpy() - ts
        col = cmap(i / max(1, len(sel) - 1) * 0.85)
        lbl = f"{m.get('to_v','?')} V"
        axes[0].plot(t, df["wheel_vel"], color=col, lw=1.5, label=lbl)
        axes[1].plot(t, df["gyro_dps"], color=col, lw=1.3, label=lbl)

    axes[0].set_ylabel("wheel velocity (rad/s)", fontsize=10)
    axes[1].set_ylabel("platform rate (dps)", fontsize=10)
    axes[1].set_xlabel("time relative to step (s)", fontsize=10)
    for a in axes:
        a.grid(alpha=0.25)
        a.axhline(0, color="#999", lw=0.7, ls=":")
        a.axvline(0, color=C_MARK, lw=1.1, alpha=0.7)
        a.legend(fontsize=8.5, ncol=2, framealpha=0.9)
    fig.suptitle("From-rest step tests, aligned at the step", fontsize=13, fontweight="bold", y=0.955)
    fig.savefig(outpath, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return True


def plot_overview_scaling(tests, outpath):
    """Steady-state wheel velocity and peak platform rate vs command."""
    rows = []
    for df, m in tests:
        if abs(m.get("from_v", 1)) > 1e-9:
            continue
        v = m.get("to_v")
        wheel_ss = float(df["wheel_vel"].to_numpy()[-20:].mean())
        g = df["gyro_dps"].to_numpy()
        peak = float(g[np.argmax(np.abs(g))])
        rows.append((v, wheel_ss, peak))
    if len(rows) < 2:
        return False
    rows.sort()
    v = np.array([r[0] for r in rows])
    w = np.array([r[1] for r in rows])
    p = np.array([r[2] for r in rows])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))

    ax = axes[0]
    ax.plot(v, w, "o-", color=C_WHEEL, lw=1.6, ms=7)
    # Reference line through the origin fitted on magnitude -- if the points
    # track it, a linear model is defensible.
    k = np.sum(v * w) / np.sum(v * v) if np.sum(v * v) else 0.0
    xs = np.linspace(min(0, v.min()) * 1.05, max(0, v.max()) * 1.05, 50)
    ax.plot(xs, k * xs, ls="--", color="#999", lw=1.1,
            label=f"through origin, slope {k:.2f}")
    ax.set_xlabel("commanded voltage (V)", fontsize=10)
    ax.set_ylabel("steady-state wheel velocity (rad/s)", fontsize=10)
    ax.set_title("Wheel response scaling", fontsize=11, fontweight="bold")
    ax.grid(alpha=0.25)
    ax.axhline(0, color="#999", lw=0.7, ls=":")
    ax.axvline(0, color="#999", lw=0.7, ls=":")
    ax.legend(fontsize=8.5)

    ax = axes[1]
    ax.plot(v, p, "s-", color=C_PLATFORM, lw=1.6, ms=6)
    ax.set_xlabel("commanded voltage (V)", fontsize=10)
    ax.set_ylabel("peak platform rate (dps)", fontsize=10)
    ax.set_title("Platform reaction scaling", fontsize=11, fontweight="bold")
    ax.grid(alpha=0.25)
    ax.axhline(0, color="#999", lw=0.7, ls=":")
    ax.axvline(0, color="#999", lw=0.7, ls=":")

    fig.tight_layout()
    fig.savefig(outpath, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description="Plot a filtered calibration run.")
    ap.add_argument("filtered_folder", help="The filtered/ folder from filter_calibration.py")
    ap.add_argument("--outdir", default=None, help="Where to write PNGs (default: <filtered_folder>/plots)")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.filtered_folder, "test*.csv")))
    if not files:
        print(f"No test*.csv found in {args.filtered_folder}")
        print("Run filter_calibration.py first, then point this at the filtered/ folder.")
        sys.exit(1)

    outdir = args.outdir or os.path.join(args.filtered_folder, "plots")
    os.makedirs(outdir, exist_ok=True)

    tests = []
    for path in files:
        df, meta = read_filtered(path)
        tests.append((df, meta))
        name = os.path.splitext(os.path.basename(path))[0]
        out = os.path.join(outdir, name + ".png")
        plot_one(df, meta, out)
        print(f"  wrote {out}")

    p = os.path.join(outdir, "overview_steps.png")
    if plot_overview_steps(tests, p):
        print(f"  wrote {p}")

    p = os.path.join(outdir, "overview_scaling.png")
    if plot_overview_scaling(tests, p):
        print(f"  wrote {p}")


if __name__ == "__main__":
    main()