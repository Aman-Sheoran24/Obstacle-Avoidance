#!/usr/bin/env python3
"""Example 02 — run VFH+ on a synthetic scan and plot the histogram.

Useful for sanity-checking the planner without any hardware or simulator.
"""

from __future__ import annotations

import argparse

from obstacle_avoidance.lidar.synthetic import SyntheticLidar
from obstacle_avoidance.vfh.vfh_plus import VFHPlus


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--scene",
        choices=["cylinder", "doorway", "dead-end", "default"],
        default="default",
    )
    ap.add_argument("--target-deg", type=float, default=0.0)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    if args.scene == "cylinder":
        lidar = SyntheticLidar().add_cylinder(3.0, 0.0, 0.5)
    elif args.scene == "doorway":
        lidar = SyntheticLidar().add_box(5.0, 1.2, 1.0, 0.5).add_box(5.0, -1.2, 1.0, 0.5)
    elif args.scene == "dead-end":
        lidar = (
            SyntheticLidar()
            .add_box(2.0, 0.0, 0.4, 4.0)
            .add_box(0.0, 2.0, 4.0, 0.4)
            .add_box(0.0, -2.0, 4.0, 0.4)
            .add_box(-2.0, 0.0, 0.4, 4.0)
        )
    else:
        lidar = SyntheticLidar.default_scene()

    scan = lidar.read_scan()
    vfh = VFHPlus()
    theta = vfh.step(scan.angles_deg, scan.ranges_m, args.target_deg, 0.0)
    print(
        f"scene={args.scene} target={args.target_deg:.0f} deg "
        f"-> chosen heading={'BRAKE' if theta is None else f'{theta:.0f} deg'}"
    )

    if not args.no_plot:
        from obstacle_avoidance.vfh.plotting import plot_polar

        plot_polar(vfh, theta)


if __name__ == "__main__":
    main()
