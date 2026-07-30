#!/usr/bin/env python3
"""
capture_calibration.py

Companion script for calibration.cpp. The Nucleo-F446RE in this project has
no SD card / filesystem, so it can't write files itself -- this script runs
on your PC, connects to the board over the SAME serial link you'd normally
watch in PuTTY, triggers the sweep's start signal itself, and writes each
test's CSV dump straight into its own file. Everything else the board
prints (bring-up messages, per-test status lines) is echoed to the console
live, same as you'd see in PuTTY. This is meant to fully replace PuTTY for
actual data-collection runs, not run alongside it -- only one program can
hold a COM port open at a time, which is also why closing PuTTY was
necessary before this script could open the port at all.

TWO-WAY, like a terminal: anything you type in this console + Enter gets
sent straight to the board, same as typing in PuTTY -- e.g. to manually
trigger an abort (any byte) if something looks wrong mid-sweep. A background
thread handles this so it doesn't block the reading/logging loop.

USAGE:
    pip install pyserial
    python capture_calibration.py --port COM5
    python capture_calibration.py --port /dev/ttyACM0 --baud 115200

    (omit --port to see a list of available ports and pick one)

WORKFLOW:
    1. Press the Nucleo's physical RESET button (or power-cycle it) so the
       sketch is at the very start of setup() -- same as the "reset the
       board to run again" instruction already printed by the firmware.
    2. Run this script. It waits for the exact prompt
       "Hardware bring-up done. Send any character..." that
       waitForStartSignal() prints, then sends a single byte to trigger
       the sweep -- you don't need to type anything to start it.
    3. Watch the console for live progress, and type + Enter at any time to
       send something manually (e.g. to abort). When the sweep finishes, a
       timestamped folder will contain one .csv file per test, named from
       that test's number and label.

NOTE: this works over either link (USB Serial or the HC-05 Bluetooth COM
port) -- they carry identical data, so just point --port at whichever one
you're connected to. Don't run this against both at once; it only needs one.
"""

import argparse
import os
import re
import sys
import threading
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("This script needs pyserial. Install it with:  pip install pyserial")
    sys.exit(1)

START_PROMPT = "Hardware bring-up done. Send any character on Serial or HC-05 to start the sweep..."

# Matches: --- capture start (test 3/9: spin to 1.0V, then reverse to -1.0V) ---
RE_START = re.compile(r"^--- capture start \(test (\d+)/(\d+): (.*)\) ---$")
# Matches: from=1.00V to=-1.00V hold=2000ms stop_reason=platform_settled
RE_META = re.compile(r"^from=([-\d.]+)V to=([-\d.]+)V hold=(\d+)ms stop_reason=(\w+)$")
RE_END = re.compile(r"^--- capture end ---$")

# The CSV header is whatever line starts with the time column. Detecting the
# header rather than pattern-matching the metadata line is deliberate: the
# firmware's metadata fields change as the sweep evolves (repeats, phase
# flags, bias, staircase parameters), and an exact-match regex on that line
# silently stalls the whole capture whenever it changes -- which is exactly
# what happened when repeats and the phase-A flags were added. Everything
# between the start marker and this header is treated as opaque metadata and
# copied through verbatim, so new fields never break capture again.
RE_HEADER = re.compile(r"^t_us\s*,")


def sanitize(label):
    """Turn a test label into a safe filename fragment."""
    s = re.sub(r"[^A-Za-z0-9]+", "_", label)
    return s.strip("_")


def stdin_forwarder(ser):
    """Runs in a background thread: whatever you type + Enter in the console
    gets written straight to the serial port, so this script works as a
    normal two-way terminal (like PuTTY) in addition to auto-capturing CSV
    data. Daemon thread -- dies with the main thread on exit/Ctrl+C, no
    special shutdown handling needed."""
    while True:
        try:
            line = input()
        except (EOFError, OSError):
            break
        try:
            ser.write((line + "\n").encode("utf-8"))
        except Exception:
            break


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in the board (or check the HC-05's paired COM port) and try again.")
        sys.exit(1)
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")
    choice = input("Pick a port number: ").strip()
    try:
        return ports[int(choice)].device
    except (ValueError, IndexError):
        print("Not a valid choice.")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Capture the calibration sweep's CSV output into per-test files.")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200, matches the firmware)")
    parser.add_argument("--outdir", default=None, help="Output folder (default: timestamped calibration_run_* folder)")
    args = parser.parse_args()

    port = args.port or pick_port()
    outdir = args.outdir or f"calibration_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(outdir, exist_ok=True)

    print(f"Opening {port} at {args.baud} baud...")
    ser = serial.Serial(port, args.baud, timeout=1)
    # Nucleo's ST-LINK VCP does not auto-reset the target on connect, so this
    # just starts reading whatever the board is currently printing -- press
    # the physical reset button before running this script if you want the
    # full bring-up sequence from the top.

    forwarder = threading.Thread(target=stdin_forwarder, args=(ser,), daemon=True)
    forwarder.start()

    sent_start_signal = False
    state = "IDLE"  # IDLE -> AWAIT_HEADER -> IN_DATA
    cur_test_num = cur_total = cur_label = None
    cur_meta_lines = []
    cur_header = None
    cur_rows = []
    files_written = 0
    stall_guard = 0

    print(f"Writing CSV files to: {outdir}/")
    print("Type + Enter at any time to send it to the board (e.g. to abort manually).")
    print("Waiting for the board... (Ctrl+C to stop)")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue  # timeout with nothing received, keep waiting
            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue

            # Trigger the sweep ourselves once we see the exact prompt --
            # removes the need for a second terminal to type into.
            if not sent_start_signal and START_PROMPT in line:
                print(line)
                print(">> Sending start signal...")
                ser.write(b"\n")
                sent_start_signal = True
                continue

            if state == "IDLE":
                m = RE_START.match(line)
                if m:
                    cur_test_num, cur_total, cur_label = m.group(1), m.group(2), m.group(3)
                    cur_meta_lines = []
                    cur_rows = []
                    cur_header = None
                    stall_guard = 0
                    state = "AWAIT_HEADER"
                    print(f">> Capturing test {cur_test_num}/{cur_total}: {cur_label}")
                else:
                    print(line)  # ordinary status line -- just echo it

            elif state == "AWAIT_HEADER":
                if RE_HEADER.match(line):
                    cur_header = line
                    state = "IN_DATA"
                else:
                    # Opaque metadata -- copied through without being parsed.
                    cur_meta_lines.append(line)
                    stall_guard += 1
                    # If the header never arrives, don't silently swallow the
                    # rest of the sweep the way the old parser did. Bail back
                    # to IDLE and say so, loudly.
                    if stall_guard > 12:
                        print("!! No CSV header found after the capture-start marker.")
                        print("!! Expected a line beginning with 't_us,'. Skipping this block.")
                        for ml in cur_meta_lines:
                            print("   " + ml)
                        state = "IDLE"

            elif state == "IN_DATA":
                if RE_END.match(line):
                    fname = f"test{int(cur_test_num):02d}_{sanitize(cur_label)}.csv"
                    path = os.path.join(outdir, fname)
                    with open(path, "w", newline="") as f:
                        f.write(f"# test {cur_test_num}/{cur_total}: {cur_label}\n")
                        for ml in cur_meta_lines:
                            f.write(f"# {ml}\n")
                        f.write(cur_header + "\n")
                        f.write("\n".join(cur_rows) + "\n")
                    files_written += 1
                    print(f">> Saved {path}  ({len(cur_rows)} rows)")
                    state = "IDLE"
                    cur_rows = []
                else:
                    cur_rows.append(line)

    except KeyboardInterrupt:
        print(f"\nStopped. {files_written} test file(s) written to {outdir}/")
    finally:
        ser.close()
        if files_written:
            print(f"{files_written} test file(s) in {outdir}/")
        else:
            print(f"No files written. If the board was streaming data, the capture-start "
                  f"marker or CSV header didn't match what this script expects.")


if __name__ == "__main__":
    main()