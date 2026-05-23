"""CRC-8 lookup table for the LD19 / STL-19P serial protocol.

Polynomial : 0x4D
Initial    : 0x00
Reflection : none
Final XOR  : 0x00

Reference: ``ldrobotSensorTeam/ldlidar_stl_sdk`` (``include/lipkg.h``).
Computing the table at import time gives the same 256-byte literal the
vendor SDK hard-codes, but is trivial to audit.
"""

from __future__ import annotations


def _build_table(poly: int = 0x4D) -> tuple[int, ...]:
    table = [0] * 256
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        table[i] = c
    return tuple(table)


CRC_TABLE: tuple[int, ...] = _build_table()


def crc8(data: bytes | bytearray) -> int:
    """Compute the LD19 CRC-8 over ``data``."""
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c
