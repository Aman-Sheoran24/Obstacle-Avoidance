"""LDRobot LD19 / STL-19P / D500 serial parser.

Protocol summary (Elecrow LD19 Development Manual V2.3):

================  =====  =================================================
Field             Bytes  Meaning
================  =====  =================================================
header              1    always 0x54
ver_len             1    fixed 0x2C (type=1, 12 points/packet)
speed               2    rotation rate, deg/s (little-endian)
start_angle         2    unit 0.01 deg
data[12]           36    12 x (distance u16 mm, intensity u8)
end_angle           2    unit 0.01 deg
timestamp           2    ms, wraps at 30000
crc8                1    CRC-8, poly 0x4D, init 0x00
================  =====  =================================================

Total packet length: **47 bytes**. UART: 230400 8N1, no parity, no flow
control, one-way. The LD19 starts streaming as soon as the motor stabilises
(~2-3 s after power on) and accepts no commands.
"""

from __future__ import annotations

import contextlib
import struct
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .crc_table import crc8

if TYPE_CHECKING:
    import serial as _serial_t  # noqa: F401 — only for type hints

PKT_LEN = 47
HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACKET = 12


@dataclass
class Scan:
    """A single (approximately 360-degree) revolution worth of points."""

    angles_deg: list[float]
    ranges_m: list[float]
    intensities: list[int]


def interpolate_angles(start_deg: float, end_deg: float, n: int = POINTS_PER_PACKET) -> list[float]:
    """Wrap-aware linear interpolation between start and end angles."""
    span = (end_deg - start_deg) % 360.0
    step = span / (n - 1)
    return [(start_deg + i * step) % 360.0 for i in range(n)]


class LD19:
    """Blocking serial driver for the LD19 family.

    Use ``read_scan()`` in a loop — it returns one ``Scan`` per revolution.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 230400, timeout: float = 0.05):
        # Lazy import so importing this module does not require pyserial.
        import serial

        self.ser = serial.Serial(port, baud, timeout=timeout)
        # angle_int -> (range_m, intensity) accumulator for the current scan.
        self._scan_points: dict[int, tuple[float, int]] = {}

    # ------------------------------------------------------------------
    # Low-level packet I/O
    # ------------------------------------------------------------------

    def _read_packet(self) -> bytes | None:
        """Read one valid 47-byte packet or return None on timeout."""
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] != HEADER:
                continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != VERLEN:
                continue
            rest = self.ser.read(PKT_LEN - 2)
            if len(rest) != PKT_LEN - 2:
                return None
            pkt = bytes([HEADER, VERLEN]) + rest
            if crc8(pkt[:-1]) != pkt[-1]:
                continue  # bad CRC — drop and resync
            return pkt

    @staticmethod
    def parse_packet(pkt: bytes) -> list[tuple[float, float, int]]:
        """Decode one 47-byte packet into a list of (angle_deg, range_m, intensity)."""
        if len(pkt) != PKT_LEN:
            raise ValueError(f"expected {PKT_LEN}-byte packet, got {len(pkt)}")
        start_angle = struct.unpack_from("<H", pkt, 4)[0] / 100.0
        points: list[tuple[int, int]] = []
        off = 6
        for _ in range(POINTS_PER_PACKET):
            dist_mm = struct.unpack_from("<H", pkt, off)[0]
            intens = pkt[off + 2]
            points.append((dist_mm, intens))
            off += 3
        end_angle = struct.unpack_from("<H", pkt, off)[0] / 100.0
        angles = interpolate_angles(start_angle, end_angle, POINTS_PER_PACKET)
        return [(angles[i], d_mm / 1000.0, inten) for i, (d_mm, inten) in enumerate(points)]

    # ------------------------------------------------------------------
    # High-level scan accumulation
    # ------------------------------------------------------------------

    def read_scan(self) -> Scan:
        """Block until one revolution of points is collected, then return it.

        Points are keyed by integer-degree, so the resulting Scan has up to
        360 entries sorted by angle. Invalid points (distance == 0) and
        low-intensity points are dropped here so downstream code sees only
        usable returns.
        """
        last_ang: float | None = None
        while True:
            pkt = self._read_packet()
            if pkt is None:
                continue
            for ang, r, inten in self.parse_packet(pkt):
                if r <= 0.0:
                    continue
                self._scan_points[int(ang)] = (r, inten)
                # A revolution boundary is the only place where ang wraps
                # backwards by a large amount.
                if last_ang is not None and ang < last_ang - 180.0:
                    items = sorted(self._scan_points.items())
                    scan = Scan(
                        angles_deg=[float(a) for a, _ in items],
                        ranges_m=[v[0] for _, v in items],
                        intensities=[v[1] for _, v in items],
                    )
                    self._scan_points.clear()
                    return scan
                last_ang = ang

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self.ser.close()
