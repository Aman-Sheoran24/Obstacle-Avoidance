#!/usr/bin/env python3
"""Smoke-test the LD19 LiDAR on its own.

Reads N revolutions from the sensor, prints summary stats, and (optionally)
plots the most recent scan on a polar matplotlib axis. Run before wiring
the LiDAR to anything else.
"""

from __future__ import annotations

import argparse
import sys

from obstacle_avoidance.lidar.ld19 import LD19


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Dump a few LD19 scans for diagnostics.")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=230400)
    p.add_argument("-n", "--num-scans", type=int, default=5)
    p.add_argument("--plot", action="store_true", help="show a polar plot of the last scan.")
    args = p.parse_args(argv)

    lidar = LD19(port=args.port, baud=args.baud)
    print(f"connected to {args.port} @ {args.baud}; reading {args.num_scans} scans...")
    last = None
    for i in range(args.num_scans):
        scan = lidar.read_scan()
        last = scan
        n = len(scan.ranges_m)
        r_min = min(scan.ranges_m) if n else float("nan")
        r_max = max(scan.ranges_m) if n else float("nan")
        r_mean = (sum(scan.ranges_m) / n) if n else float("nan")
        print(
            f"scan {i + 1}: {n:>3d} points, r_min={r_min:.2f} m, r_max={r_max:.2f} m, r_mean={r_mean:.2f} m"
        )
    lidar.close()

    if args.plot and last is not None:
        import matplotlib.pyplot as plt
        import numpy as np

        ax = plt.subplot(111, projection="polar")
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.scatter(np.deg2rad(last.angles_deg), last.ranges_m, s=4)
        ax.set_title(f"LD19 scan ({len(last.ranges_m)} points)")
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
