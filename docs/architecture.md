# Architecture

## System block diagram

```
                          +-------------------+
                          |   LD19 LiDAR      |
                          |   (USB-UART, 10Hz)|
                          +---------+---------+
                                    |  /dev/ttyUSB0  (230400 8N1)
                                    v
+-----------------------------------+-------------------------------+
|   Jetson Orin Nano  (companion computer)                          |
|                                                                   |
|   +----------------+   +----------------+   +------------------+  |
|   |  lidar.ld19    |-->|  lidar.filters |-->|  vfh.vfh_plus    |  |
|   |  (Scan)        |   |  (clean Scan)  |   |  theta_best      |  |
|   +----------------+   +----------------+   +--------+---------+  |
|                                                      |            |
|                                              theta_deg|, v_cmd    |
|                                                      v            |
|   +--------------------------------------------------+---------+  |
|   |  pipeline.run_pipeline (10 Hz, monotonic clock)            |  |
|   |  -- vx, vy from theta + V_MAX + clutter-based slowdown     |  |
|   +-------------------+--------------------+--------------------+ |
|                       |                    |                      |
|                       v                    v                      |
|       +---------------+--------+   +-------+-------------+        |
|       | mavlink.velocity       |   | mavlink.controller  |        |
|       | SET_POSITION_TARGET    |   | mode / arm / RTL    |        |
|       +-----------+------------+   +---------+-----------+        |
+-------------------|--------------------------|--------------------+
                    |  /dev/ttyACM0 (USB MAVLink)
                    v
              +-----+------+
              |  Pixhawk   |
              |  ArduCopter|
              |  GUIDED    |
              +------------+
```

### Sim alternative

For the SITL path, the LiDAR block is replaced by `lidar.synthetic` (or by
the `/lidar_scan` Gazebo topic via `gz-transport13`), and the MAVLink link
becomes `udpin:127.0.0.1:14551` rather than a USB serial port. Everything
between (filters, VFH+, velocity, controller) is identical — the only
abstractions that change at the system boundary are the sensor and the
transport.

## Data + timing budget

| Stage                 | Rate    | Latency  | Notes                                  |
|-----------------------|---------|----------|----------------------------------------|
| LD19 packet           | ~480/s  | <2 ms    | 12 points/packet, 4500 points/scan     |
| Scan assembly         | 10 Hz   | ~100 ms  | one revolution                         |
| Filtering             | 10 Hz   | <2 ms    | intensity + median + prop mask         |
| VFH+ step             | 10 Hz   | <5 ms    | vectorised numpy on the Orin Nano      |
| MAVLink velocity send | 10 Hz   | <1 ms    | UDP or USB serial                      |
| ArduPilot timeout     | 3 s     | n/a      | stops vehicle on dropped commands      |
| PX4 timeout           | 0.5 s   | n/a      | for the alternate stack                |

The end-to-end loop runs at 10 Hz which keeps both ArduPilot and PX4
safely above their command-watchdog thresholds.

## Why ArduPilot and not PX4

ArduPilot GUIDED was chosen as the primary target because:

- No pre-arm offboard handshake (PX4 OFFBOARD requires "*at least a second*"
  of streamed setpoints before it will arm).
- More forgiving timeout (3 s vs. 500 ms).
- Body-frame velocity uses `MAV_FRAME_BODY_OFFSET_NED` which is well
  documented in the ArduPilot dev wiki.
- The failsafe surface (geofence, RC kill, RTL on link loss) is broader
  and easier to configure for a beginner project.

The architecture is portable to PX4 — swap the frame in
[`mavlink/velocity.py`](../src/obstacle_avoidance/mavlink/velocity.py) to
`MAV_FRAME_BODY_NED` and start streaming setpoints before arming.

## Module responsibilities

- `config.py` — every tunable in one dataclass; no magic numbers elsewhere.
- `lidar/` — sensor I/O, filters, and a synthetic stand-in for sim/dev.
- `vfh/` — VFH+ algorithm (stages A-D from Ulrich & Borenstein 1998).
- `mavlink/` — connection lifecycle, GUIDED-mode commands, telemetry thread.
- `pipeline.py` — the 10 Hz loop; the only place those layers talk to each
  other.
- `run.py` — argparse + signal handling + wiring everything together.
