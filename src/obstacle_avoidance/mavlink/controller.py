"""High-level MAVLink session: connect, mode, arm, takeoff, RTL.

Designed for ArduPilot Copter 4.5.x via pymavlink 2.4.41. The same code
also flies PX4 SITL in OFFBOARD mode if you swap the frame in
``velocity.py``; see ``docs/architecture.md`` for the differences.
"""

from __future__ import annotations

import contextlib
import logging
import time

from ..config import MAVLinkConfig
from .telemetry import TelemetryListener

log = logging.getLogger(__name__)


class MavController:
    def __init__(self, cfg: MAVLinkConfig):
        self.cfg = cfg
        self.conn = None  # populated by ``connect()``
        self.telem: TelemetryListener | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, timeout_s: float = 30.0) -> None:
        from pymavlink import mavutil

        url = self.cfg.connection
        log.info("connecting to %s", url)
        if "://" in url or url.startswith(("udpin:", "udpout:", "tcp:")):
            self.conn = mavutil.mavlink_connection(url, source_system=self.cfg.source_system)
        else:
            self.conn = mavutil.mavlink_connection(
                url, baud=self.cfg.baud, source_system=self.cfg.source_system
            )
        hb = self.conn.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise TimeoutError(f"no heartbeat from {url} in {timeout_s} s")
        log.info(
            "heartbeat from sys=%d comp=%d (type=%d autopilot=%d)",
            self.conn.target_system,
            self.conn.target_component,
            hb.type,
            hb.autopilot,
        )
        self.telem = TelemetryListener(self.conn)
        self.telem.start()

    def close(self) -> None:
        if self.telem is not None:
            self.telem.stop()
        if self.conn is not None:
            with contextlib.suppress(Exception):
                self.conn.close()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def set_mode(self, mode_name: str, timeout_s: float = 10.0) -> None:
        from pymavlink import mavutil

        mapping = self.conn.mode_mapping()
        if mode_name not in mapping:
            raise ValueError(f"mode {mode_name} not in {sorted(mapping)}")
        mode_id = mapping[mode_name]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is not None and msg.custom_mode == mode_id:
                log.info("mode set to %s", mode_name)
                return
        raise TimeoutError(f"mode change to {mode_name} not confirmed in {timeout_s} s")

    def arm(self, timeout_s: float = 10.0) -> None:
        log.info("arming")
        self.conn.arducopter_arm()
        self.conn.motors_armed_wait()

    def takeoff(self, altitude_m: float, timeout_s: float = 30.0) -> None:
        from pymavlink import mavutil

        log.info("takeoff to %.1f m", altitude_m)
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(altitude_m),
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            alt = self.current_alt_m()
            if alt is not None and alt >= altitude_m * 0.95:
                log.info("reached takeoff altitude %.2f m", alt)
                return
            time.sleep(0.5)
        log.warning(
            "takeoff did not confirm in %.1f s (last alt=%s)", timeout_s, self.current_alt_m()
        )

    def rtl(self) -> None:
        log.info("return-to-launch")
        try:
            self.set_mode("RTL", timeout_s=5.0)
        except Exception as exc:
            log.error("RTL set_mode failed: %s", exc)

    # ------------------------------------------------------------------
    # Telemetry shortcuts
    # ------------------------------------------------------------------

    def current_yaw_deg(self) -> float | None:
        return None if self.telem is None else self.telem.state.yaw_deg

    def current_alt_m(self) -> float | None:
        return None if self.telem is None else self.telem.state.relative_alt_m

    def heartbeat_age_s(self) -> float | None:
        return None if self.telem is None else self.telem.heartbeat_age_s()
