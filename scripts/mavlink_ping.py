#!/usr/bin/env python3
"""Smoke-test the MAVLink link to the autopilot.

Connects, waits for a heartbeat, requests an AUTOPILOT_VERSION, and prints
the first few HEARTBEAT / GLOBAL_POSITION_INT / SYS_STATUS messages. Use
this before running the full pipeline.
"""

from __future__ import annotations

import argparse
import sys
import time


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Smoke-test a MAVLink connection.")
    p.add_argument(
        "--connection",
        default="udpin:127.0.0.1:14551",
        help="serial port or MAVLink URL (default: SITL on udp 14551).",
    )
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=5.0, help="seconds to listen after handshake.")
    args = p.parse_args(argv)

    from pymavlink import mavutil

    url = args.connection
    if "://" in url or url.startswith(("udpin:", "udpout:", "tcp:")):
        conn = mavutil.mavlink_connection(url, source_system=255)
    else:
        conn = mavutil.mavlink_connection(url, baud=args.baud, source_system=255)

    print(f"waiting for heartbeat on {url}...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("no heartbeat — check connection / power.", file=sys.stderr)
        return 1
    print(f"heartbeat OK: sys={conn.target_system} comp={conn.target_component}")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    interesting = {"HEARTBEAT", "AUTOPILOT_VERSION", "GLOBAL_POSITION_INT", "SYS_STATUS"}
    deadline = time.monotonic() + args.duration
    seen: dict[str, int] = {}
    while time.monotonic() < deadline:
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        seen[t] = seen.get(t, 0) + 1
        if t in interesting and seen[t] <= 2:
            print(f"  {t}: {msg.to_dict()}")
    print("message counts:", {k: v for k, v in sorted(seen.items())})
    conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
