# Safety and Failsafes

This is a **student / research project**. The hardware can move at several
metres per second and weighs more than a kilogram. Read this page before
flying for the first time, and re-read it after any time away from the
project.

> **Do not fly this system over people, vehicles, or property.**
> Operate over open grass with no bystanders inside the flight area.

---

## Pre-flight checklist

Encoded in [`scripts/preflight.py`](../scripts/preflight.py) — exits
non-zero on any failure:

1. **Heartbeat** received from the autopilot within 5 s of connect.
2. **EKF healthy**: `EKF_STATUS_REPORT.flags & 0x1F == 0x1F` (position
   horizontal absolute + relative + vertical absolute + horizontal velocity
   + attitude).
3. **GPS fix** type >= 3, HDOP < 1.5.
4. **Battery voltage** above your configured `BATT_LOW_VOLT` threshold
   (`14.0 V` default for a 4S LiPo).
5. **LD19** publishing more than 4000 points per second (the motor is
   actually spinning).
6. **RC channel 8** (avoidance kill / enable) reads `> 1700 us` — the
   pilot has engaged the avoidance system.
7. **Geofence** enabled (`FENCE_ENABLE == 1`).

Run with:

```bash
python scripts/preflight.py --connection /dev/ttyACM0 --lidar-port /dev/ttyUSB0
```

Do not arm if any check fails.

## Failsafe matrix

| Failure                          | Detection                          | Action                                      |
|----------------------------------|------------------------------------|---------------------------------------------|
| LiDAR -> dead-end (no valley)    | `VFHPlus.step()` returns `None`    | Send zero velocity (brake); pilot may RTL.  |
| Companion link drops             | ArduPilot 3 s command watchdog     | Vehicle stops automatically.                |
| GCS heartbeat lost               | `FS_GCS_ENABLE = 1`                | Configured action (default RTL).            |
| RC link lost                     | `FS_THR_ENABLE = 1`                | Configured action (default RTL).            |
| Geofence breach                  | `FENCE_ACTION = 1`                 | RTL or LAND.                                |
| Battery low                      | `BATT_FS_LOW_ACT = 2`              | RTL.                                        |
| Pilot kill                       | RC channel 5 -> STABILIZE / LAND   | Manual recovery.                            |

The RC kill switch on channel 8 (`RC8_OPTION = 40`, Proximity Avoidance
Enable) is **the operator's last line of defence**. Always have it within
finger reach during avoidance flights.

## Velocity envelope

| Parameter (config field)                  | Default      | Notes                                          |
|-------------------------------------------|--------------|------------------------------------------------|
| `safety.v_max_horizontal_mps`             | `2.0`        | First flights at 1.0 then ramp up.             |
| `safety.v_max_vertical_mps`               | `0.5`        | Altitude is held by ArduPilot, not us.         |
| `safety.v_base_mps`                       | `1.0`        | Slowed proportionally by VFH+ clutter peak.    |
| `mavlink.rate_hz`                         | `10`         | Keeps both ArduPilot (3 s) + PX4 (0.5 s) safe. |
| ArduPilot `WPNAV_SPEED` (cm/s)            | `200`        | Caps AUTO-mode horizontal speed.               |

## Flight-test progression

Skip steps at your peril.

1. **Bench**: airframe propped on a stand, props **off**. Run
   `scripts/preflight.py`; confirm scans visualise correctly.
2. **Tethered hover**: 3 m cord, 1 m altitude, 0.5 m/s velocity cap.
   Engage avoidance over a soft (foam) obstacle.
3. **Open field**: large area (>= 30 m radius), no bystanders, RC kill
   switch in hand. Plan a single straight-line traverse past one
   obstacle.
4. **Multiple obstacles**: doorway, then slalom, only after each previous
   scenario succeeds three times in a row.

## Things to refuse

- Flying indoors with people present.
- Flying without an RC kill switch wired and tested on the day.
- Flying with battery < 14.4 V (4S) — the avoidance loop assumes responsive
  motors.
- Flying within sight of bright sunlight reflecting off windows / wet
  ground (LD19 spec is 30000 lux; direct sunlight is up to ~98000 lux).
- Modifying VFH+ weights mid-flight via parameter download.

## Reporting incidents

If the drone hits anything, save the dataflash log (`*.bin`) and the
companion-computer journal (`journalctl -u obstacle-avoidance > log.txt`)
before powering down. Both contain the timing information needed to
understand what happened.
