"""Command-line entry point: ``python -m obstacle_avoidance.run --sim``."""

from __future__ import annotations

import argparse
import logging
import signal
import sys

from .config import Config
from .mavlink.controller import MavController


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="obstacle-avoidance",
        description="UAV obstacle avoidance with LD19 LiDAR + VFH+ + Pixhawk.",
    )
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="connect to ArduPilot SITL")
    mode.add_argument("--hw", action="store_true", help="connect to a real Pixhawk + LD19")

    p.add_argument(
        "--connection",
        default=None,
        help="override the MAVLink connection string (e.g. udpin:127.0.0.1:14551).",
    )
    p.add_argument(
        "--lidar-port",
        default=None,
        help="override the LD19 serial port (default: /dev/ttyUSB0).",
    )
    p.add_argument(
        "--goal-bearing",
        type=float,
        default=0.0,
        help="constant goal bearing in degrees (0 = forward).",
    )
    p.add_argument(
        "--takeoff-alt",
        type=float,
        default=None,
        help="override the takeoff altitude in metres.",
    )
    p.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    cfg = Config()
    if args.takeoff_alt is not None:
        cfg.mavlink.takeoff_alt_m = args.takeoff_alt

    if args.sim:
        cfg.mavlink.connection = args.connection or "udpin:127.0.0.1:14551"
        from .lidar.synthetic import SyntheticLidar

        lidar = SyntheticLidar.default_scene()
    else:
        if args.connection:
            cfg.mavlink.connection = args.connection
        if args.lidar_port:
            cfg.lidar.port = args.lidar_port
        from .lidar.ld19 import LD19

        lidar = LD19(port=cfg.lidar.port, baud=cfg.lidar.baud, timeout=cfg.lidar.timeout_s)

    # Lazy import so `--help` does not require pymavlink.
    from .pipeline import run_pipeline

    mav = MavController(cfg.mavlink)
    mav.connect()
    mav.set_mode(cfg.mavlink.flight_mode)
    mav.arm()
    mav.takeoff(cfg.mavlink.takeoff_alt_m)

    stop = {"flag": False}

    def _stop(*_: object) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        run_pipeline(
            cfg,
            lidar,
            mav,
            goal_bearing_deg=lambda: args.goal_bearing,
            stop_flag=lambda: stop["flag"],
        )
    finally:
        try:
            mav.rtl()
        except Exception as exc:  # — diagnostic shutdown path
            logging.error("RTL failed during shutdown: %s", exc)
        lidar.close()
        mav.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
