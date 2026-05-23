#!/usr/bin/env python3
"""Plot altitude and ground-speed from an ArduPilot dataflash (.BIN) or .tlog."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Quick-look ArduPilot flight log plotter.")
    p.add_argument("log", type=Path, help="path to .bin (DataFlash) or .tlog (telemetry).")
    p.add_argument("--save", type=Path, default=None, help="optional output PNG path.")
    args = p.parse_args(argv)

    from pymavlink import mavutil

    log = mavutil.mavlink_connection(str(args.log))
    times: list[float] = []
    alts: list[float] = []
    gs: list[float] = []
    while True:
        m = log.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD"], blocking=False)
        if m is None:
            break
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            times.append(m.time_boot_ms / 1000.0)
            alts.append(m.relative_alt / 1000.0)
        elif t == "VFR_HUD":
            gs.append(m.groundspeed)

    if not times:
        print("no GLOBAL_POSITION_INT messages in log.", file=sys.stderr)
        return 1

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(times, alts, label="rel. altitude (m)")
    axes[0].set_ylabel("altitude (m)")
    axes[0].legend()
    if gs:
        n = min(len(times), len(gs))
        axes[1].plot(times[:n], gs[:n], color="orange", label="ground speed (m/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("speed (m/s)")
    axes[1].legend()
    fig.suptitle(args.log.name)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved plot to {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
