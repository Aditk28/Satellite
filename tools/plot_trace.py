#!/usr/bin/env python
"""plot_trace.py - Gantt + inter-arrival histogram from a scheduler-trace dump.

Usage:
    py tools/plot_trace.py trace.txt        # a saved serial capture
    (or pipe a capture in on stdin)

Reads any file that CONTAINS the CSV block emitted by trace_dump():
    t_us,id,evt        (evt: 1=switched IN, 0=switched OUT, >=10=user marker)
Non-CSV lines (banner, '#' comments) are ignored, so you can just save the whole
serial dump to a text file and point this at it.
"""
import sys
import re
import matplotlib
matplotlib.use("Agg")          # file-producing script: never open a blocking window
import matplotlib.pyplot as plt

ID_NAMES = {0: "IDLE", 1: "CTRL", 2: "FOC", 3: "SAFETY",
            4: "COMMS", 5: "TELEM", 6: "TEST", 7: "TEST2"}
CSV_RE = re.compile(r'^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*$')


def load(path):
    text = open(path).read() if path else sys.stdin.read()
    rows = []
    for ln in text.splitlines():
        m = CSV_RE.match(ln)
        if m:
            rows.append(tuple(int(g) for g in m.groups()))   # (t_us, id, evt)
    return rows


def unwrap(rows):
    """TIM5 is 32-bit us (wraps ~71 min). Undo any wrap so time is monotonic."""
    out, offset, prev = [], 0, rows[0][0]
    for t, i, e in rows:
        if t < prev:                    # counter went backwards -> wrapped
            offset += (1 << 32)
        prev = t
        out.append((t + offset, i, e))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    rows = load(path)
    if not rows:
        print("no 't_us,id,evt' rows found in input")
        return
    rows = unwrap(rows)
    base = rows[0][0]

    intervals, open_in, markers = {}, {}, {}
    for t, i, e in rows:
        tt = t - base
        if e == 1:                      # switched IN
            open_in[i] = tt
        elif e == 0:                    # switched OUT -> close the interval
            if i in open_in:
                intervals.setdefault(i, []).append((open_in[i], tt - open_in[i]))
                del open_in[i]
        else:                           # user marker
            markers.setdefault(i, []).append((tt, e))

    ids = sorted(intervals.keys())
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))

    # --- Gantt: one row per task ---
    for row, i in enumerate(ids):
        ax1.broken_barh(intervals[i], (row * 10, 8))
    ax1.set_yticks([row * 10 + 4 for row in range(len(ids))])
    ax1.set_yticklabels([ID_NAMES.get(i, "id%d" % i) for i in ids])
    ax1.set_xlabel("t (us)")
    ax1.set_title("scheduler timeline  (%d events)" % len(rows))
    ax1.grid(axis="x", alpha=0.3)

    # --- per-task inter-arrival (time between consecutive switch-INs) ---
    for i in ids:
        starts = [s for (s, d) in intervals[i]]
        if len(starts) > 2:
            deltas = [starts[k + 1] - starts[k] for k in range(len(starts) - 1)]
            ax2.hist(deltas, bins=50, alpha=0.5, label=ID_NAMES.get(i, "id%d" % i))
    ax2.set_xlabel("inter-arrival between switch-INs (us)")
    ax2.set_ylabel("count")
    ax2.set_title("per-task inter-arrival")
    ax2.legend()

    plt.tight_layout()
    out = (path or "trace") + ".png"
    plt.savefig(out, dpi=110)
    print("wrote", out)
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
