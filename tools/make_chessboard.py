#!/usr/bin/env python3
"""
Generate a calibration chessboard as SVG, sized in real millimetres.

Run on the LAPTOP (not the Pi), open the SVG in a browser, print at 100%.

WHY SVG. It carries physical units, so a browser prints it at true size without
depending on DPI metadata surviving the trip through a print dialog. A PNG at
"300 dpi" is a promise the printer may quietly break, and a scale error here is
a scale error in every range measurement forever.

WHY 9x6 AND NOT SOMETHING SQUARE. The counts must DIFFER, or the pattern has a
180 deg rotational symmetry and the detector cannot tell which way up it is --
corner ordering then flips between frames and the calibration is garbage.

NOTE "inner corners" means the interior crossings where four squares meet, NOT
the number of squares. 9x6 inner corners is a 10x7 grid of squares.

    python make_chessboard.py                 # 9x6 inner corners, 25 mm, A4
    python make_chessboard.py --square-mm 30
"""

import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="INNER corners across")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--out", default="chessboard.svg")
    args = ap.parse_args()

    # inner corners + 1 = squares along each axis
    nx, ny = args.cols + 1, args.rows + 1
    s = args.square_mm
    w, h = nx * s, ny * s
    margin = 10.0                      # white quiet border, helps the detector

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{w+2*margin}mm" height="{h+2*margin}mm" '
        f'viewBox="0 0 {w+2*margin} {h+2*margin}">',
        f'<rect width="{w+2*margin}" height="{h+2*margin}" fill="white"/>',
    ]
    for iy in range(ny):
        for ix in range(nx):
            if (ix + iy) % 2 == 0:
                parts.append(
                    f'<rect x="{margin+ix*s}" y="{margin+iy*s}" '
                    f'width="{s}" height="{s}" fill="black"/>')
    # Printed reference so a scaling error is visible without a caliper.
    parts.append(
        f'<text x="{margin}" y="{margin+h+7}" font-family="sans-serif" '
        f'font-size="4">{args.cols}x{args.rows} inner corners, {s:g} mm '
        f'squares - MEASURE ONE SQUARE AFTER PRINTING</text>')
    parts.append('</svg>')

    with open(args.out, "w") as f:
        f.write("\n".join(parts))

    print(f"wrote {args.out}")
    print(f"  {nx}x{ny} squares = {args.cols}x{args.rows} INNER corners")
    print(f"  pattern {w:.0f} x {h:.0f} mm, page {w+2*margin:.0f} x "
          f"{h+2*margin:.0f} mm")
    print(f"\n  Print at 100% / 'Actual size' -- NOT 'fit to page'.")
    print(f"  Then MEASURE one square with calipers and pass the real number")
    print(f"  to pi_calibrate.py --square-mm. Printers scale.")


if __name__ == "__main__":
    main()
