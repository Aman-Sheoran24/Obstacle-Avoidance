"""Background telemetry reader so the control loop always sees fresh state."""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class VehicleState:
    yaw_deg: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    relative_alt_m: float | None = None
    lat_deg: float | None = None
    lon_deg: float | None = None
    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None
    ekf_flags: int | None = None
    last_heartbeat_t: float = field(default_factory=lambda: 0.0)


class TelemetryListener:
    """Spawns a background thread that drains MAVLink messages into ``state``."""

    def __init__(self, conn):
        self.conn = conn
        self.state = VehicleState()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, name="mav-telem", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def heartbeat_age_s(self) -> float:
        return time.monotonic() - self.state.last_heartbeat_t

    # ------------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:  # — broken pipe / closed serial
                log.debug("recv_match raised: %s", exc)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                self.state.last_heartbeat_t = time.monotonic()
            elif t == "ATTITUDE":
                self.state.roll_deg = math.degrees(msg.roll)
                self.state.pitch_deg = math.degrees(msg.pitch)
                self.state.yaw_deg = math.degrees(msg.yaw) % 360.0
            elif t == "GLOBAL_POSITION_INT":
                self.state.lat_deg = msg.lat / 1e7
                self.state.lon_deg = msg.lon / 1e7
                self.state.relative_alt_m = msg.relative_alt / 1000.0
            elif t == "SYS_STATUS":
                self.state.battery_voltage_v = msg.voltage_battery / 1000.0
                self.state.battery_remaining_pct = msg.battery_remaining
            elif t == "EKF_STATUS_REPORT":
                self.state.ekf_flags = msg.flags
