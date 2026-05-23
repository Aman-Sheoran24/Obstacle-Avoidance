#!/usr/bin/env python3
"""Example 03 — full SITL loop: arm, take off, run VFH+ against a fake scene.

Prereq: ArduPilot SITL running on UDP 14550/14551.
    sim_vehicle.py -v ArduCopter --console --map -w

Then in this terminal:
    python examples/03_sitl_full_loop.py
"""

from __future__ import annotations

import argparse
import signal

from obstacle_avoidance.config import Config
from obstacle_avoidance.lidar.synthetic import SyntheticLidar
from obstacle_avoidance.mavlink.controller import MavController
from obstacle_avoidance.pipeline import run_pipeline


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--connection", default="udpin:127.0.0.1:14551")
    ap.add_argument("--takeoff-alt", type=float, default=5.0)
    ap.add_argument("--goal-bearing", type=float, default=0.0)
    args = ap.parse_args()

    cfg = Config()
    cfg.mavlink.connection = args.connection
    cfg.mavlink.takeoff_alt_m = args.takeoff_alt

    lidar = SyntheticLidar.default_scene()
    mav = MavController(cfg.mavlink)
    mav.connect()
    mav.set_mode(cfg.mavlink.flight_mode)
    mav.arm()
    mav.takeoff(cfg.mavlink.takeoff_alt_m)

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    try:
        run_pipeline(
            cfg,
            lidar,
            mav,
            goal_bearing_deg=lambda: args.goal_bearing,
            stop_flag=lambda: stop["flag"],
        )
    finally:
        mav.rtl()
        mav.close()
        lidar.close()


if __name__ == "__main__":
    main()
