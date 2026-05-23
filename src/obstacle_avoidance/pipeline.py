"""The 10 Hz control loop wiring LiDAR -> VFH+ -> velocity setpoint."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from typing import Protocol

from .config import Config
from .lidar.filters import apply_filters
from .lidar.ld19 import Scan
from .mavlink.controller import MavController
from .mavlink.velocity import send_body_velocity
from .vfh.vfh_plus import VFHPlus

log = logging.getLogger(__name__)


class ScanSource(Protocol):
    """Anything that can return a 360-degree Scan."""

    def read_scan(self) -> Scan: ...

    def close(self) -> None: ...


def vel_from_theta(theta_deg: float, v_mps: float) -> tuple[float, float]:
    """Project a VFH+ steering angle onto body-frame (vx, vy)."""
    th = math.radians(theta_deg)
    return v_mps * math.cos(th), v_mps * math.sin(th)


def run_pipeline(
    cfg: Config,
    lidar: ScanSource,
    mav: MavController,
    goal_bearing_deg: Callable[[], float],
    stop_flag: Callable[[], bool],
) -> None:
    """Run the control loop until ``stop_flag()`` returns True.

    Parameters
    ----------
    cfg
        Full configuration object — used for VFH+ tuning and safety limits.
    lidar
        Either ``LD19`` or ``SyntheticLidar``; both implement ``read_scan``.
    mav
        Connected ``MavController`` — assumed armed and in GUIDED mode.
    goal_bearing_deg
        Callable returning the current desired heading (degrees, body frame).
    stop_flag
        Callable polled each iteration; returning True breaks the loop and
        sends a final zero-velocity command.
    """
    vfh = VFHPlus.from_config(cfg.vfh)
    period = 1.0 / cfg.mavlink.rate_hz
    next_tick = time.monotonic()

    log.info(
        "pipeline start: rate=%.1f Hz v_max=%.1f m/s",
        cfg.mavlink.rate_hz,
        cfg.safety.v_max_horizontal_mps,
    )

    while not stop_flag():
        scan = lidar.read_scan()
        scan = apply_filters(scan, cfg.lidar)

        current_yaw = mav.current_yaw_deg() or 0.0
        target = goal_bearing_deg()
        theta = vfh.step(scan.angles_deg, scan.ranges_m, target, current_yaw)

        if theta is None:
            send_body_velocity(mav.conn, 0.0, 0.0, 0.0)
            log.info("dead-end / no candidate — braking")
        else:
            # Slow down when the chosen sector is cluttered (peak H^p).
            clutter = vfh.last_density_peak() or 1e-3
            slowdown = 1.0 - (vfh.last_density_at(theta) / clutter)
            v_cmd = max(
                0.0,
                min(cfg.safety.v_max_horizontal_mps, cfg.safety.v_base_mps * slowdown),
            )
            vx, vy = vel_from_theta(theta, v_cmd)
            send_body_velocity(mav.conn, vx, vy, 0.0)

        # Monotonic scheduling — keep jitter under control.
        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            # We're behind schedule — reset so the next sleep isn't negative.
            next_tick = time.monotonic()

    send_body_velocity(mav.conn, 0.0, 0.0, 0.0)
    log.info("pipeline stopped")
