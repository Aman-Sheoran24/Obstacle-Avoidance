# obstacle-avoidance

[![CI](https://github.com/Aman-Sheoran24/obstacle-avoidance/actions/workflows/ci.yml/badge.svg)](https://github.com/Aman-Sheoran24/obstacle-avoidance/actions/workflows/ci.yml)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

> Real-time UAV obstacle avoidance with the LDRobot **LD19 / STL-19P** 2D LiDAR,
> the **VFH+** algorithm (Ulrich & Borenstein, 1998), and a **Pixhawk** autopilot
> driven over MAVLink. Runs on a Jetson Orin Nano companion computer. No ROS.

<!--
   Replace this comment block with a demo GIF once you have one:
   ![demo](docs/images/demo.gif)
-->

---

## Contents

1. [What this project is](#what-this-project-is)
2. [System architecture](#system-architecture)
3. [Hardware BOM](#hardware-bom)
4. [Wiring](#wiring)
5. [Software install](#software-install)
6. [Simulation quickstart (5 minutes)](#simulation-quickstart-5-minutes)
7. [Hardware deployment](#hardware-deployment)
8. [How the algorithm works](#how-the-algorithm-works)
9. [Configuration reference](#configuration-reference)
10. [Troubleshooting](#troubleshooting)
11. [Safety / disclaimer](#safety--disclaimer)
12. [Project layout](#project-layout)
13. [References](#references)
14. [License](#license)

---

## What this project is

A complete, **pure-Python** companion-computer stack that:

- Parses the LD19 / STL-19P 230400-baud serial protocol (47-byte packets,
  12 points each, CRC-8 polynomial 0x4D).
- Runs **VFH+** at 10 Hz on a 72-sector polar histogram with hysteresis,
  valley enumeration, and the 3-term cost function.
- Streams `SET_POSITION_TARGET_LOCAL_NED` body-frame velocity setpoints to
  the Pixhawk in ArduPilot GUIDED mode.
- Includes a **drop-in synthetic LiDAR** so you can develop everything in
  ArduPilot SITL before touching hardware.
- Includes a Gazebo Harmonic world (Iris quad + `gpu_lidar`) for the
  full end-to-end sim.

What it is **not**:

- A 3D / vertical avoidance planner (use VFH* or 3D-VFH for that).
- A drop-in replacement for ArduPilot's built-in `BendyRuler` / proximity
  avoidance — this runs *outside* the autopilot and commands it.
- Production safety-critical code. See [Safety](#safety--disclaimer).

## System architecture

```
LD19 LiDAR -> Jetson Orin Nano -> [filter -> VFH+ -> velocity cmd] -> Pixhawk -> motors
                                                                          ^
                                            ArduPilot SITL + Gazebo  -----'  (sim path)
```

Full block diagram and timing budget: [`docs/architecture.md`](docs/architecture.md).

## Hardware BOM

| Item                                                | Notes                                              |
|-----------------------------------------------------|----------------------------------------------------|
| NVIDIA Jetson Orin Nano Developer Kit               | Ubuntu 22.04 / JetPack 6                           |
| Pixhawk autopilot (e.g. **Cube Orange** or Pixhawk 6C) | Running ArduCopter 4.5.x                        |
| **LDRobot D500 kit** (LD19 + CP2102 adapter)        | Also sold as Waveshare DTOF LD19 / DFRobot STL-19P |
| 4S LiPo battery, ESCs, motors, frame                | Whatever you currently fly                         |
| Spare RC transmitter with channel 8 toggle          | Avoidance kill switch                              |

## Wiring

Full pinouts (USB primary, TELEM2 UART alternative): [`docs/wiring.md`](docs/wiring.md).

Quick version:
- LD19 CP2102 adapter -> Jetson USB port -> shows up as `/dev/ttyUSB0`.
- Pixhawk USB -> Jetson USB port -> shows up as `/dev/ttyACM0`.

## Software install

```bash
git clone https://github.com/Aman-Sheoran24/obstacle-avoidance.git
cd obstacle-avoidance

python3.10 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install -e ".[dev]"
```

For the Gazebo simulator path, see the OS-level prereqs below in
[Simulation quickstart](#simulation-quickstart-5-minutes).

## Simulation quickstart (5 minutes)

This path lets you watch the avoidance loop work without any hardware.

**Prereq:** Ubuntu 22.04, ArduPilot dev environment, Gazebo Harmonic.

```bash
# One-time: install ArduPilot prerequisites
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile && cd ..

# One-time: Gazebo Harmonic + ardupilot_gazebo
sudo apt install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install gz-harmonic libgz-sim8-dev rapidjson-dev

git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make -j4
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Then run the three pieces (this script will open three `gnome-terminal`
tabs for you, or print the commands if it isn't installed):

```bash
bash sim/launch/sitl_gazebo.sh
```

You should see the Iris arm, take off to 5 m, then fly forward — diverting
around the cylinder placed dead ahead by the avoidance loop.

### Even lighter dev loop (no Gazebo)

If you just want to iterate on VFH+ logic, skip Gazebo entirely:

```bash
# Terminal 1
sim_vehicle.py -v ArduCopter --console --map -w

# Terminal 2 (uses the built-in SyntheticLidar)
python -m obstacle_avoidance.run --sim --connection udpin:127.0.0.1:14551
```

## Hardware deployment

1. Follow [`docs/wiring.md`](docs/wiring.md) for the physical connections.
2. Bench-test (props off):

   ```bash
   python scripts/ld19_dump.py --port /dev/ttyUSB0 --plot
   python scripts/mavlink_ping.py --connection /dev/ttyACM0
   python scripts/preflight.py    # all 7 checks must pass
   ```
3. Tethered hover first (see [Safety](#safety--disclaimer)).
4. Auto-start on boot via systemd. Drop this in `/etc/systemd/system/obstacle-avoidance.service`:

   ```ini
   [Unit]
   Description=UAV obstacle avoidance
   After=network.target

   [Service]
   ExecStart=/home/jetson/.venv/bin/python -m obstacle_avoidance.run --hw
   Restart=on-failure
   User=jetson

   [Install]
   WantedBy=multi-user.target
   ```

   Then `sudo systemctl enable --now obstacle-avoidance`. Logs land in
   `journalctl -u obstacle-avoidance`.

## How the algorithm works

The full VFH+ walk-through — polar histogram math, hysteresis, valley
selection, and cost function with paper equation references — lives at
[`docs/algorithm.md`](docs/algorithm.md).

## Configuration reference

Every tunable lives in one place:
[`src/obstacle_avoidance/config.py`](src/obstacle_avoidance/config.py).

| Group     | Field                                | Default            | Notes                                            |
|-----------|--------------------------------------|--------------------|--------------------------------------------------|
| `lidar`   | `port`                               | `/dev/ttyUSB0`     | CP2102 adapter.                                  |
| `lidar`   | `baud`                               | `230400`           | LD19 fixed rate.                                 |
| `lidar`   | `min_intensity`                      | `50`               | Drop low-confidence reflections.                 |
| `lidar`   | `range_max_m`                        | `12.0`             | Sensor spec.                                     |
| `lidar`   | `median_window`                      | `3`                | Odd or `1` to disable.                           |
| `lidar`   | `prop_mask_deg`                      | `None`             | E.g. `(170.0, 190.0)` to mask the prop arm.      |
| `vfh`     | `n_sectors` / `alpha_deg`            | `72 / 5.0`         | Must multiply to 360.                            |
| `vfh`     | `r_safe_m`                           | `0.65`             | Drone radius + safety margin.                    |
| `vfh`     | `d_max_m`                            | `5.0`              | Planning horizon (sensor reach is 12 m).         |
| `vfh`     | `a, b`                               | `197.0, 1.0`       | Cell-magnitude weights (paper eq. 2-3).          |
| `vfh`     | `tau_low, tau_high`                  | `197.0, 591.0`     | Hysteresis thresholds.                           |
| `vfh`     | `s_max`                              | `16`               | Wide / narrow valley boundary.                   |
| `vfh`     | `mu1, mu2, mu3`                      | `5.0, 2.0, 2.0`    | Cost weights. Must satisfy `mu1 > mu2 + mu3`.    |
| `mavlink` | `connection`                         | `/dev/ttyACM0`     | Or `udpin:127.0.0.1:14551` for SITL.             |
| `mavlink` | `flight_mode`                        | `GUIDED`           | PX4 users: `OFFBOARD`.                           |
| `mavlink` | `takeoff_alt_m`                      | `5.0`              |                                                  |
| `mavlink` | `rate_hz`                            | `10`               | Above both ArduPilot (3 s) and PX4 (0.5 s) timeouts. |
| `safety`  | `v_max_horizontal_mps`               | `2.0`              | First flights at 1.0 m/s.                        |
| `safety`  | `rc_kill_channel`                    | `8`                | `RC8_OPTION = 40` on the autopilot.              |

## Troubleshooting

| Symptom                                            | Likely cause / fix                                                                               |
|----------------------------------------------------|--------------------------------------------------------------------------------------------------|
| `permission denied: /dev/ttyUSB0`                  | `sudo usermod -aG dialout $USER`, log out + back in.                                             |
| LD19 returns zero scans                            | Motor not spinning. Check 5 V on the CP2102 board, listen for the click.                         |
| `no heartbeat from <url>`                          | Wrong port, wrong baud, or Pixhawk not powered. Try `mavlink_ping.py`.                           |
| `mode change to GUIDED not confirmed`              | Pre-arm check failed on the autopilot. Inspect MAVProxy console.                                 |
| Drone takes off then immediately RTLs              | Companion loop didn't start streaming setpoints. Confirm `rate_hz >= 5`.                         |
| Drone flies the wrong direction                    | Frame mismatch: ArduPilot needs `MAV_FRAME_BODY_OFFSET_NED`, PX4 needs `MAV_FRAME_BODY_NED`.     |
| VFH+ always brakes (returns `None`)                | Hysteresis stuck. Restart the script, or lower `tau_low` / `tau_high`.                           |
| EKF flags never go healthy in SITL                 | Wait 30 s for GPS lock or run `param set ARMING_CHECK 0` (sim only).                             |
| Gazebo crashes on launch                           | `libgz-sim8-dev` not installed, or `GZ_SIM_SYSTEM_PLUGIN_PATH` missing the ardupilot_gazebo build dir. |
| `pymavlink` install fails on Jetson                | `sudo apt install python3-lxml python3-future` then `pip install pymavlink==2.4.41`.             |

## Safety / disclaimer

This is a **research / student project**. Do not fly it over people or
property. Read [`docs/safety.md`](docs/safety.md) cover-to-cover before
arming on real hardware. Always have an RC kill switch wired and tested
on the day.

## Project layout

```
obstacle-avoidance/
├── README.md
├── LICENSE
├── pyproject.toml                       # PEP 621 metadata + ruff/black
├── requirements*.txt                    # pinned
├── .pre-commit-config.yaml
├── .github/workflows/ci.yml             # lint + pytest, two Python versions
├── docs/                                # architecture, wiring, algorithm, safety
├── src/obstacle_avoidance/
│   ├── config.py                        # one dataclass for every tunable
│   ├── pipeline.py                      # 10 Hz loop
│   ├── run.py                           # __main__ — argparse, --sim / --hw
│   ├── lidar/                           # LD19 driver + filters + synthetic
│   ├── vfh/                             # VFH+ + plotting
│   └── mavlink/                         # controller + velocity + telemetry
├── sim/                                 # Gazebo world, iris+lidar model, SITL params
├── scripts/                             # ld19_dump, mavlink_ping, preflight, plot_log
├── tests/                               # unit tests (CI-runnable, no hardware)
└── examples/                            # 3 progressively-larger snippets
```

## References

- Iwan Ulrich and Johann Borenstein. **VFH+: Reliable Obstacle Avoidance
  for Fast Mobile Robots.** *Proc. IEEE ICRA*, 1998, pp. 1572-1577.
  PDF: <https://cs.cmu.edu/~iwan/papers/vfh+.pdf>
- LDRobot LD19 Development Manual V2.3 (`LD19_Development_Manual_V2.3.pdf`,
  mirrored by Elecrow, Waveshare, and youyeetoo).
- ArduPilot — *Copter Commands in Guided Mode*:
  <https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html>
- pymavlink mavgen guide:
  <https://www.ardusub.com/developers/pymavlink.html>
- Vendor C SDK (CRC table reference):
  <https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk>
- Reference Python LD06/LD19 loader:
  <https://github.com/henjin0/LIDAR_LD06_python_loader>
- Original VFH-only Python implementation (extend with binary histogram +
  cost function): <https://github.com/vanderbiltrobotics/vfh-python>
- `ardupilot_gazebo` plugin (sim path):
  <https://github.com/ArduPilot/ardupilot_gazebo>

## License

[MIT](LICENSE). Contributions welcome — please open an issue first for
non-trivial changes.
