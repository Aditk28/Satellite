#!/usr/bin/env python3
"""
Generate a calibration chessboard as an SVG sized in real millimetres.

Runs on the laptop. Open the SVG in a browser and print at 100%. SVG carries
physical units, so it prints at true size without relying on DPI metadata.
The corner counts must differ (9x6, not square), otherwise the board is
rotationally symmetric and the detector cannot tell which way up it is.
"Inner corners" are the crossings where four squares meet: 9x6 inner corners
is a 10x7 grid of squares.

    python make_chessboard.py                 # 9x6 inner corners, 25 mm
    python make_chessboard.py --square-mm 30
"""

import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--out", default="chessboard.svg")
    args = ap.parse_args()

    nx, ny = args.cols + 1, args.rows + 1
    s = args.square_mm
    w, h = nx * s, ny * s
    margin = 10.0                      # white border helps the detector

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
    parts.append(
        f'<text x="{margin}" y="{margin+h+7}" font-family="sans-serif" '
        f'font-size="4">{args.cols}x{args.rows} inner corners, {s:g} mm '
        f'squares - measure one square after printing</text>')
    parts.append('</svg>')

    with open(args.out, "w") as f:
        f.write("\n".join(parts))

    print(f"wrote {args.out}")
    print(f"  {nx}x{ny} squares = {args.cols}x{args.rows} inner corners")
    print(f"  pattern {w:.0f} x {h:.0f} mm, page {w+2*margin:.0f} x "
          f"{h+2*margin:.0f} mm")
    print("\n  Print at 100% / 'Actual size', not 'fit to page'.")
    print("  Measure one square and pass the real size to")
    print("  pi_calibrate.py --square-mm.")


if __name__ == "__main__":
    main()
