"""Body-frame velocity setpoints for ArduPilot GUIDED mode.

ArduPilot expects ``MAV_FRAME_BODY_OFFSET_NED`` for body-frame velocity.
PX4 OFFBOARD uses ``MAV_FRAME_BODY_NED`` instead — get this wrong and your
drone flies in the wrong direction. The bit mask 0b0000111111000111 tells
the autopilot to ignore everything except vx / vy / vz / yaw_rate.
"""

from __future__ import annotations

import time

# Type mask: ignore position, acceleration, yaw (absolute). Honour velocity
# and yaw_rate. Bit ordering matches MAV_TYPE_MASK_*_IGNORE flags.
TYPE_MASK_VEL_ONLY = 0b0000111111000111

_BOOT_T = time.monotonic()


def send_body_velocity(
    conn,
    vx: float,
    vy: float,
    vz: float = 0.0,
    yaw_rate: float = 0.0,
) -> None:
    """Stream one ``SET_POSITION_TARGET_LOCAL_NED`` setpoint.

    Parameters are in metres per second (body frame: +x forward, +y right,
    +z down per NED). ``yaw_rate`` is in radians per second.
    """
    from pymavlink import mavutil

    conn.mav.set_position_target_local_ned_send(
        int((time.monotonic() - _BOOT_T) * 1000),
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        TYPE_MASK_VEL_ONLY,
        0.0,
        0.0,
        0.0,  # x, y, z position (ignored)
        float(vx),
        float(vy),
        float(vz),
        0.0,
        0.0,
        0.0,  # ax, ay, az (ignored)
        0.0,
        float(yaw_rate),
    )
