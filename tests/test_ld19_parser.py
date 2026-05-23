"""Unit tests for the LD19 protocol parser."""

from __future__ import annotations

import struct

import pytest
from obstacle_avoidance.lidar.crc_table import crc8
from obstacle_avoidance.lidar.ld19 import (
    HEADER,
    LD19,
    PKT_LEN,
    POINTS_PER_PACKET,
    VERLEN,
    interpolate_angles,
)


def _build_packet(
    start_angle_deg: float,
    end_angle_deg: float,
    distances_mm: list[int] | None = None,
    intensities: list[int] | None = None,
    speed_deg_s: int = 3600,
    timestamp_ms: int = 0,
) -> bytes:
    distances_mm = distances_mm or [1000] * POINTS_PER_PACKET
    intensities = intensities or [200] * POINTS_PER_PACKET
    assert len(distances_mm) == POINTS_PER_PACKET
    assert len(intensities) == POINTS_PER_PACKET

    buf = bytearray()
    buf.append(HEADER)
    buf.append(VERLEN)
    buf += struct.pack("<H", speed_deg_s)
    buf += struct.pack("<H", int(round(start_angle_deg * 100)) % 36000)
    for d, i in zip(distances_mm, intensities, strict=True):
        buf += struct.pack("<H", d)
        buf.append(i)
    buf += struct.pack("<H", int(round(end_angle_deg * 100)) % 36000)
    buf += struct.pack("<H", timestamp_ms)
    buf.append(crc8(bytes(buf)))
    assert len(buf) == PKT_LEN
    return bytes(buf)


# ----------------------------------------------------------------------
# CRC + packet structure
# ----------------------------------------------------------------------


def test_crc_zero_buffer_is_zero():
    assert crc8(b"") == 0
    assert crc8(b"\x00" * 10) == 0


def test_crc_round_trip():
    pkt = _build_packet(0.0, 30.0)
    # CRC is the last byte; recomputing over the first 46 bytes must match.
    assert crc8(pkt[:-1]) == pkt[-1]


def test_parse_packet_returns_12_points():
    pkt = _build_packet(0.0, 33.0, distances_mm=[1500] * POINTS_PER_PACKET)
    points = LD19.parse_packet(pkt)
    assert len(points) == POINTS_PER_PACKET
    for ang, r, inten in points:
        assert 0.0 <= ang < 360.0
        assert r == pytest.approx(1.5)
        assert inten == 200


def test_parse_packet_wrong_length_raises():
    with pytest.raises(ValueError):
        LD19.parse_packet(b"\x54\x2c")


# ----------------------------------------------------------------------
# Angle interpolation
# ----------------------------------------------------------------------


def test_interpolate_angles_no_wrap():
    angs = interpolate_angles(10.0, 21.0, n=12)
    assert angs[0] == pytest.approx(10.0)
    assert angs[-1] == pytest.approx(21.0)
    # Step is constant.
    diffs = [angs[i + 1] - angs[i] for i in range(11)]
    assert all(d == pytest.approx(diffs[0]) for d in diffs)


def test_interpolate_angles_wraps_through_zero():
    angs = interpolate_angles(355.0, 6.0, n=12)
    # Span must be 11 degrees, not -349.
    span = (angs[-1] - angs[0]) % 360.0
    assert span == pytest.approx(11.0)
    assert all(0.0 <= a < 360.0 for a in angs)


# ----------------------------------------------------------------------
# Realistic packet round-trip
# ----------------------------------------------------------------------


def test_parse_packet_decodes_known_bytes():
    distances = [1000 + 100 * i for i in range(POINTS_PER_PACKET)]
    pkt = _build_packet(45.0, 56.0, distances_mm=distances)
    points = LD19.parse_packet(pkt)
    decoded_r = [p[1] for p in points]
    expected = [d / 1000.0 for d in distances]
    for got, exp in zip(decoded_r, expected, strict=True):
        assert got == pytest.approx(exp)
