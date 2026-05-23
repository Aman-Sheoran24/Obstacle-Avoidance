#!/usr/bin/env python3
"""Pre-flight checklist — exits non-zero if any item fails.

Encodes the 7-item list from ``docs/safety.md``. Run on the vehicle before
arming for the first hardware flight of the day.
"""

from __future__ import annotations

import argparse
import sys
import time

from obstacle_avoidance.lidar.ld19 import LD19


def check(name: str, ok: bool, detail: str = "") -> bool:
    mark = "OK  " if ok else "FAIL"
    line = f"[{mark}] {name}"
    if detail:
        line += f"  ({detail})"
    print(line)
    return ok


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Run the obstacle-avoidance pre-flight checklist.")
    p.add_argument("--connection", default="/dev/ttyACM0")
    p.add_argument("--mav-baud", type=int, default=115200)
    p.add_argument("--lidar-port", default="/dev/ttyUSB0")
    p.add_argument("--lidar-baud", type=int, default=230400)
    p.add_argument("--min-battery-v", type=float, default=14.0)
    args = p.parse_args(argv)

    failures = 0
    from pymavlink import mavutil

    url = args.connection
    if "://" in url or url.startswith(("udpin:", "udpout:", "tcp:")):
        conn = mavutil.mavlink_connection(url, source_system=255)
    else:
        conn = mavutil.mavlink_connection(url, baud=args.mav_baud, source_system=255)

    # 1) Heartbeat within 5 s.
    hb = conn.wait_heartbeat(timeout=5)
    if not check("heartbeat <5 s", hb is not None):
        failures += 1
        conn.close()
        return failures

    # 2) Drain a few seconds for the messages we need.
    deadline = time.monotonic() + 4.0
    ekf_flags = gps_fix = gps_hdop = volts = None
    while time.monotonic() < deadline:
        m = conn.recv_match(blocking=True, timeout=0.5)
        if m is None:
            continue
        t = m.get_type()
        if t == "EKF_STATUS_REPORT":
            ekf_flags = m.flags
        elif t == "GPS_RAW_INT":
            gps_fix = m.fix_type
            gps_hdop = m.eph / 100.0
        elif t == "SYS_STATUS":
            volts = m.voltage_battery / 1000.0

    # 3) EKF healthy: posHorizAbs + posHorizRel + posVertAbs + velHoriz + attitude (0x1F).
    ekf_ok = ekf_flags is not None and (ekf_flags & 0x1F) == 0x1F
    if not check("EKF flags healthy (0x1F)", ekf_ok, f"flags={ekf_flags}"):
        failures += 1

    # 4) GPS fix >= 3, HDOP < 1.5.
    gps_ok = gps_fix is not None and gps_fix >= 3 and (gps_hdop is None or gps_hdop < 1.5)
    if not check("GPS fix >= 3 and HDOP < 1.5", gps_ok, f"fix={gps_fix} hdop={gps_hdop}"):
        failures += 1

    # 5) Battery voltage above threshold.
    batt_ok = volts is not None and volts >= args.min_battery_v
    if not check(f"battery >= {args.min_battery_v} V", batt_ok, f"V={volts}"):
        failures += 1

    # 6) LD19 producing > 4000 points/s (motor spinning).
    try:
        lidar = LD19(port=args.lidar_port, baud=args.lidar_baud)
        n_points = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            scan = lidar.read_scan()
            n_points += len(scan.ranges_m)
        lidar.close()
        rate = n_points / (time.monotonic() - t0)
        lidar_ok = rate > 4000
        if not check("LD19 > 4000 pts/s", lidar_ok, f"observed {rate:.0f} pts/s"):
            failures += 1
    except Exception as exc:
        check("LD19 readable", False, str(exc))
        failures += 1

    # 7) RC8 (avoidance kill) > 1700 us.
    rc = conn.recv_match(type="RC_CHANNELS", blocking=True, timeout=2.0)
    if rc is None:
        check("RC8 PWM observed", False, "no RC_CHANNELS message")
        failures += 1
    else:
        ch8 = rc.chan8_raw
        if not check("RC8 (avoidance) > 1700 us", ch8 > 1700, f"chan8={ch8} us"):
            failures += 1

    conn.close()
    print()
    if failures == 0:
        print("ALL CHECKS PASSED — clear to arm.")
        return 0
    print(f"{failures} check(s) failed — do not arm.")
    return failures


if __name__ == "__main__":
    sys.exit(main())
