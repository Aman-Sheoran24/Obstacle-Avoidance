"""Unit tests for the body-velocity setpoint and type mask."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")  # skip if pymavlink missing

from obstacle_avoidance.mavlink.velocity import (  # noqa: E402
    TYPE_MASK_VEL_ONLY,
    send_body_velocity,
)


def test_type_mask_value():
    # Position (bits 0-2) and accel (bits 6-8) and yaw (bit 10) ignored;
    # velocity (bits 3-5) and yaw_rate (bit 11) honoured.
    assert TYPE_MASK_VEL_ONLY == 0b0000_1111_1100_0111


def test_send_body_velocity_invokes_correct_message():
    conn = MagicMock()
    conn.target_system = 1
    conn.target_component = 1

    send_body_velocity(conn, 1.5, -0.5, 0.0, yaw_rate=0.1)

    conn.mav.set_position_target_local_ned_send.assert_called_once()
    args, _ = conn.mav.set_position_target_local_ned_send.call_args

    # args[0] is the boot-time milliseconds, args[1] is sys, args[2] is comp.
    assert args[1] == 1
    assert args[2] == 1
    # args[3] is the coordinate frame — ArduPilot uses BODY_OFFSET_NED.
    assert args[3] == mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
    # args[4] is the type mask.
    assert args[4] == TYPE_MASK_VEL_ONLY
    # Position bytes are zero (args[5..7]).
    assert args[5] == 0.0 and args[6] == 0.0 and args[7] == 0.0
    # Velocity bytes (args[8..10]) carry the supplied values.
    assert args[8] == pytest.approx(1.5)
    assert args[9] == pytest.approx(-0.5)
    assert args[10] == pytest.approx(0.0)
    # Yaw absolute (args[14]) is zero; yaw rate is last (args[15]).
    assert args[14] == pytest.approx(0.0)
    assert args[15] == pytest.approx(0.1)
