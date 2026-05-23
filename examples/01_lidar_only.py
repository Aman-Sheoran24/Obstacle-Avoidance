#!/usr/bin/env python3
"""Example 01 — read a few real LD19 scans and print summary stats.

Usage:
    python examples/01_lidar_only.py --port /dev/ttyUSB0 --plot
"""

from __future__ import annotations

import argparse

from obstacle_avoidance.lidar.ld19 import LD19


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--num-scans", type=int, default=10)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    lidar = LD19(port=args.port, baud=args.baud)
    last_scan = None
    for i in range(args.num_scans):
        scan = lidar.read_scan()
        last_scan = scan
        print(
            f"scan {i + 1:>2d}: {len(scan.ranges_m):>3d} pts, "
            f"min={min(scan.ranges_m):.2f} m, max={max(scan.ranges_m):.2f} m"
        )
    lidar.close()

    if args.plot and last_scan is not None:
        import matplotlib.pyplot as plt
        import numpy as np

        ax = plt.subplot(projection="polar")
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.scatter(np.deg2rad(last_scan.angles_deg), last_scan.ranges_m, s=4)
        ax.set_title("LD19 — last scan")
        plt.show()


if __name__ == "__main__":
    main()
