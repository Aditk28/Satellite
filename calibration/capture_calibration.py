#!/usr/bin/env python3
"""
Capture the firmware's CSV dumps from the serial port into per-test files.

The board has no filesystem, so this runs on the PC in place of a serial
terminal. Board output is echoed to the console, and each block between
"--- capture start (test N/M: label) ---" and "--- capture end ---" is saved
as testNN_<label>.csv. Lines typed into the console are sent to the board.
If the firmware prints the bring-up prompt, the start signal is sent
automatically.

Usage:
    pip install pyserial
    python capture_calibration.py                 # auto-pick a port
    python capture_calibration.py --port COM5 --baud 115200

Works over USB or the HC-05 Bluetooth port. Only one program can hold the
port, so close any other terminal first.
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

# e.g. "--- capture start (test 3/9: spin to 1.0V) ---"
RE_START = re.compile(r"^--- capture start \(test (\d+)/(\d+): (.*)\) ---$")
RE_END = re.compile(r"^--- capture end ---$")

# Everything between the start marker and the CSV header is copied through as
# metadata without parsing, so new firmware fields don't break capture.
RE_HEADER = re.compile(r"^t_us\s*,")


def sanitize(label):
    """Turn a test label into a safe filename fragment."""
    s = re.sub(r"[^A-Za-z0-9]+", "_", label)
    return s.strip("_")


def stdin_forwarder(ser):
    """Background thread: send each console line to the board."""
    while True:
        try:
            line = input()
        except (EOFError, OSError):
            break
        try:
            ser.write((line + "\n").encode("utf-8"))
        except Exception:
            break


# Windows reassigns the HC-05's COM number, so ports are ranked and tried in
# order: Bluetooth first (the untethered link), then ST-LINK, then other USB.
def rank_port(p):
    text = f"{p.description} {p.manufacturer or ''}".lower()
    if "bluetooth" in text:
        return 0
    if "stlink" in text or "st-link" in text or "stmicro" in text:
        return 1
    if "usb" in text or "serial" in text:
        return 2
    return 3


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in the board (or check the HC-05's paired COM port) and try again.")
        sys.exit(1)

    print("Available ports:")
    for p in ports:
        print(f"  {p.device}  ({p.description})")

    # A paired HC-05 shows up as two Bluetooth ports and only the outgoing one
    # opens, so take the first port that opens.
    for p in sorted(ports, key=rank_port):
        try:
            serial.Serial(p.device, timeout=0.1).close()
        except Exception:
            continue
        print(f"Using {p.device}  ({p.description})")
        return p.device

    print("None of those ports would open. Close any other terminal holding one,")
    print("or pass it explicitly with --port.")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Capture the calibration sweep's CSV output into per-test files.")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--outdir", default=None, help="Output folder (default: calibration/runs/run_<timestamp>/)")
    args = parser.parse_args()

    port = args.port or pick_port()
    # Resolved from __file__ so the default is the same from any working directory.
    if args.outdir:
        outdir = args.outdir
    else:
        runs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")
        outdir = os.path.join(
            runs_dir, f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(outdir, exist_ok=True)

    print(f"Opening {port} at {args.baud} baud...")
    ser = serial.Serial(port, args.baud, timeout=1)
    # Opening the port does not reset the Nucleo. Press RESET to see bring-up.

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
                continue
            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue

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
                    print(line)

            elif state == "AWAIT_HEADER":
                if RE_HEADER.match(line):
                    cur_header = line
                    state = "IN_DATA"
                else:
                    cur_meta_lines.append(line)
                    stall_guard += 1
                    # No header after a dozen lines: give up on this block.
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
            print("No files written. If the board was streaming data, the capture-start "
                  "marker or CSV header didn't match what this script expects.")


if __name__ == "__main__":
    main()