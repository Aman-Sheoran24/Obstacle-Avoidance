# UAV Obstacle Avoidance with LD19 LiDAR + VFH+ + Pixhawk — Complete GitHub Repository Blueprint

## TL;DR
- **Yes, this is buildable in pure Python on Jetson Orin Nano**: parse the LD19's 230400-baud UART stream (header `0x54`, 47-byte packets of 12 points each) → run VFH+ on a 72-sector polar histogram → stream `SET_POSITION_TARGET_LOCAL_NED` velocity setpoints at 10 Hz to the Pixhawk over USB or TELEM2.
- **Simulate first, hardware second**: use ArduPilot SITL + `ardupilot_gazebo` (Gazebo Harmonic) with the Iris quadcopter for the full end-to-end loop; for a lighter dev cycle, use SITL alone with synthetic LiDAR scans injected into the Python pipeline. Real flight only after three obstacle scenarios pass in sim.
- **Lay out the repo as a `src/`-layout Python package** with one module per concern (`lidar/`, `vfh/`, `mavlink/`, `sim/`), pinned dependencies, MIT license, and a README that contains an architecture diagram and a copy-pasteable 5-minute SITL quickstart.

---

## Key Findings

1. **LD19 / STL-19P / D500 are the same DTOF module** sold under three SKUs (LDRobot LD19, Waveshare DTOF LIDAR LD19, DFRobot STL-19P kit). All share: 230400 baud UART (3.3 V level), 0x54 packet header, 12 measurement points per packet, 47-byte fixed packet length, CRC-8 with polynomial 0x4D, internal/PWM speed control with 10 Hz default scan rate, 0.02 m – 12 m range, ±45 mm accuracy (DFRobot product page, Elecrow `LD19_Development_Manual_V2.3.pdf`).
2. **The official C SDK is `ldrobotSensorTeam/ldlidar_stl_sdk`**, but for pure-Python with no ROS, the cleanest references are `henjin0/LIDAR_LD06_python_loader` (LD06/LD19 protocol identical), `halac123b/Visualize-data-from-Lidar-LD19_Matplotlib-Python`, and the protocol breakdown in `LudovaTech/lidar-LD19-tutorial`. The development kit ships with a **CP2102 USB-to-UART adapter**, so on the Jetson the device shows up as `/dev/ttyUSB0` — preferred for this project because it bypasses Jetson UART voltage / serial-console issues.
3. **VFH+** (Ulrich & Borenstein, *Proc. IEEE ICRA 1998*, pp. 1572–1577) is the correct algorithm tier — VFH+ added binary hysteresis, masked histogram, and the 3-term cost function `g(c) = μ1·Δ(c,kt) + μ2·Δ(c,kθ) + μ3·Δ(c,kprev)`. Paper-recommended weights are **μ1=5, μ2=2, μ3=2** with the constraint **μ1 > μ2+μ3** (eq. 17). Sector width **α=5°** giving **n=72 sectors**; wide/narrow valley threshold **s_max=16**. `vanderbiltrobotics/vfh-python` is the closest open-source Python reference but only implements the original VFH (1991), so you must extend it with the binary-histogram, masked-histogram, and cost-function steps yourself.
4. **For Pixhawk control without ROS, use ArduPilot GUIDED mode**. Per the ArduPilot "Copter Commands in Guided Mode" wiki: *"If sending velocity or acceleration commands, they should be re-sent every second (the vehicle will stop after 3 seconds if no command is received)."* PX4 OFFBOARD is also viable but requires *"a continuous 2 Hz proof-of-life signal... sent for at least a second before PX4 will arm in offboard mode"* (PX4 main docs); PX4 also has a *"timeout of 500 ms between two Offboard commands"* (PX4 MAVROS docs). Stream at **10 Hz** to be safe on both stacks.
5. **For simulation**, recommend **ArduPilot SITL + Gazebo Harmonic + the official `ArduPilot/ardupilot_gazebo` plugin** on Ubuntu 22.04 (the plugin README states Harmonic is recommended). The Iris quadcopter SDF gets a `gpu_lidar` sensor added; the scan is read via the Gazebo Transport Python bridge or — simpler for this scope — by injecting synthetic scans directly into the VFH pipeline while SITL handles flight dynamics.
6. **Safety architecture**: velocity setpoints (not position) for obstacle avoidance, ArduPilot geofence parameters, a code-side velocity cap (`V_MAX=2 m/s` for first flights), RTL on heartbeat loss, mandatory RC kill switch on channel 8 with `RC8_OPTION=40` to toggle proximity avoidance (per ArduPilot Simple Object Avoidance docs).

---

## Details

### 1. The LD19 / STL-19P serial protocol (parser-ready spec)

The protocol is documented in the LD19 development manual published by Elecrow (`LD19_Development_Manual_V2.3.pdf`) and reproduced verbatim across the LudovaTech tutorial, the Waveshare wiki, and the youyeetoo D300 wiki. Concrete numbers your parser must hard-code:

| Field | Bytes | Meaning |
|---|---|---|
| `header` | 1 | always `0x54` |
| `ver_len` | 1 | upper 3 bits = packet type (1), lower 5 bits = points per packet (12) → fixed `0x2C` |
| `speed` | 2 (LE) | rotation rate, deg/s |
| `start_angle` | 2 (LE) | unit 0.01°, divide by 100 → degrees |
| `data[12]` | 36 | 12 × (distance_u16_mm, intensity_u8) |
| `end_angle` | 2 (LE) | unit 0.01° |
| `timestamp` | 2 (LE) | ms (wraps at 30000) |
| `crc8` | 1 | CRC-8, poly 0x4D, init 0x00, no reflection, final XOR 0x00 |

Total packet length: **47 bytes**. UART params: **230400 8N1, no parity, no flow control, one-way**. The LD19 starts streaming the moment its motor stabilises (~2–3 s after power-on) and accepts no commands.

The per-point angle is computed by linear interpolation between `start_angle` and `end_angle`:
```
angle_step = ((end_angle - start_angle + 360) % 360) / (12 - 1)
angle[i]   = (start_angle + i * angle_step) % 360
```

**Reference Python parsing pattern (henjin0-style, distilled):**

```python
# src/uav_avoidance/lidar/ld19.py
import serial, struct, math
from dataclasses import dataclass

PKT_LEN = 47
HEADER, VERLEN = 0x54, 0x2C
CRC_TABLE = [  # 256-byte LD19 CRC-8 table, poly 0x4D
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE, ...  # full table in repo
]

def crc8(buf):
    c = 0
    for b in buf:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c

@dataclass
class Scan:
    angles_deg:  list  # length-N, sorted 0..360
    ranges_m:    list  # length-N
    intensities: list

class LD19:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._scan_points = {}   # angle_int -> (r, i), one-revolution accumulator

    def _read_packet(self):
        while True:                                # sync on 0x54 0x2C
            b = self.ser.read(1)
            if not b: return None
            if b[0] != HEADER: continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != VERLEN: continue
            rest = self.ser.read(PKT_LEN - 2)
            if len(rest) != PKT_LEN - 2: return None
            pkt = bytes([HEADER, VERLEN]) + rest
            if crc8(pkt[:-1]) != pkt[-1]:
                continue                           # bad CRC, resync
            return pkt

    def _parse(self, pkt):
        start_angle = struct.unpack_from("<H", pkt, 4)[0] / 100.0
        points = []
        off = 6
        for _ in range(12):
            dist   = struct.unpack_from("<H", pkt, off)[0]   # mm
            intens = pkt[off + 2]
            points.append((dist, intens))
            off += 3
        end_angle = struct.unpack_from("<H", pkt, off)[0] / 100.0
        span = (end_angle - start_angle) % 360.0
        step = span / 11.0
        return [((start_angle + i*step) % 360.0, d/1000.0, inten)
                for i,(d,inten) in enumerate(points)]

    def read_scan(self, min_intensity=50):
        last_ang = None
        while True:
            pkt = self._read_packet()
            if pkt is None: continue
            for ang, r, inten in self._parse(pkt):
                if inten < min_intensity or r <= 0.0:
                    continue
                self._scan_points[int(ang)] = (r, inten)
                if last_ang is not None and ang < last_ang - 180.0:
                    items = sorted(self._scan_points.items())
                    scan = Scan([a for a,_ in items],
                                [v[0] for _,v in items],
                                [v[1] for _,v in items])
                    self._scan_points.clear()
                    return scan
                last_ang = ang
```

**Filtering recommendations** (validated by manufacturer notes and community drivers):
- Drop points with `intensity < 50` (low-confidence reflections).
- Treat `distance == 0` as "no return" → either skip or set to `range_max=12.0 m`.
- Apply a **3-tap median filter** along the angle axis to remove single-bin spikes.
- Mask out an angular cone behind/below the propellers (e.g., `[170°, 190°]`) if the LiDAR is mounted under the airframe — set those bins to `range_max` so VFH treats them as free.

**Wiring**: The kit's CP2102 adapter board powers the LiDAR from 5 V (USB VBUS) and exposes the 3.3 V UART through a virtual COM port. **Plug the supplied USB cable into a Jetson Orin Nano USB port** — udev will assign `/dev/ttyUSB0`. Add the user to `dialout` (`sudo usermod -aG dialout $USER`) so root is not required. Direct connection to Jetson `/dev/ttyTHS1` (pins 8/10 on the 40-pin header) is also valid because the LD19 is already 3.3 V, but you must first disable the serial console (`sudo systemctl disable nvgetty`). The USB-adapter route is simpler and is what the LDRobot quickstart documents.

### 2. VFH+ implementation in Python

The textbook VFH+ pipeline, sized for a 360° 2D LiDAR with 12 m range:

**Stage A — polar obstacle density H^p.** With α=5° → n=72 sectors. From the LD19 scan, for each (angle β, distance d) point with d ≤ d_max (set d_max = 5 m for this 12 m sensor → planning horizon well inside sensor range):

```
m = max(0, a - b * d**2)   # cell magnitude (Ulrich & Borenstein eq. 2)
γ = arcsin(r_safe / d)     # enlargement angle (eq. 4); r_safe = r_drone + d_s
for k in 0..n-1:
    if (k*α) ∈ [β - γ, β + γ] (modulo 360):
        H^p[k] += m
```

Per Ulrich & Borenstein (ICRA 1998, eq. 2), the cell magnitude is **m_{i,j} = c_{i,j}² · (a − b·d_{i,j}²)** with the constraint **a − b·((w_s−1)/2)² = 1** (eq. 3), where w_s is the active-window diameter; the paper does **not** commit to specific a, b numerics. Pragmatic choice mirroring the Vanderbilt implementation: **b=1, a=1+((w_s−1)/2)²**, with w_s=29 cells (≈5 m radius at 0.18 m resolution → a≈197). For each cell c_{i,j}=1 (LiDAR hit) or 0 (free).

**Stage B — binary histogram H^b with hysteresis** (eq. 7):
```
H^b[k] = 1 if H^p[k] > τ_high
       = 0 if H^p[k] < τ_low
       = H^b_prev[k] otherwise
```
The paper introduces τ_high and τ_low only symbolically and does not commit to numerical values — they are application-tuned. Start with **τ_low = 1.0·a, τ_high = 3.0·a** as a first guess, then lower until the drone reacts to obstacles at the desired distance.

**Stage C — masked histogram H^m.** Mask out sectors that, given the drone's heading and minimum turning radius r_min, would still collide. For a multirotor that can strafe, set r_min = 0 (omnidirectional) — masked = binary.

**Stage D — valley selection and cost function.** Per the paper (eqs. 12–17): find contiguous runs of `H^m[k]==0` (valleys). A valley is **wide if width > s_max=16 sectors (80°)**, else **narrow**.
- Narrow: candidate direction **c_n = (k_r + k_l)/2**
- Wide: **c_r = k_r + s_max/2**, **c_l = k_l − s_max/2**, plus **c_t = k_t** if k_t ∈ [c_r, c_l]

Cost (eqs. 15–16): **g(c) = μ1·Δ(c, k_t) + μ2·Δ(c, θ_i/α) + μ3·Δ(c, k_{n,i−1})** where Δ is the wrapped angular distance. Paper-recommended weights: **μ1 = 5, μ2 = 2, μ3 = 2**, with the necessary condition **μ1 > μ2 + μ3** (eq. 17) to guarantee goal-attraction dominates.

```python
# src/uav_avoidance/vfh/vfh_plus.py
import numpy as np

class VFHPlus:
    def __init__(self, n_sectors=72, alpha_deg=5.0,
                 r_safe=0.65, d_max=5.0,
                 a=197.0, b=1.0,
                 tau_low=197.0, tau_high=591.0,
                 s_max=16, mu1=5.0, mu2=2.0, mu3=2.0):
        assert n_sectors * alpha_deg == 360.0
        assert mu1 > mu2 + mu3                    # Ulrich-Borenstein, eq. 17
        self.n, self.alpha = n_sectors, alpha_deg
        self.r_safe, self.d_max = r_safe, d_max
        self.a, self.b = a, b
        self.tau_low, self.tau_high = tau_low, tau_high
        self.s_max = s_max
        self.mu = (mu1, mu2, mu3)
        self.h_b_prev = np.zeros(n_sectors, dtype=np.int8)
        self.k_prev = 0

    def step(self, angles_deg, ranges_m, target_heading_deg, current_heading_deg):
        # ---- Stage A ----
        Hp = np.zeros(self.n)
        for beta, d in zip(angles_deg, ranges_m):
            if d <= 0.0 or d > self.d_max: continue
            m = max(0.0, self.a - self.b * d * d)
            gamma = np.degrees(np.arcsin(min(1.0, self.r_safe / max(d, 1e-3))))
            for k in range(self.n):
                diff = (k*self.alpha - beta + 180) % 360 - 180
                if abs(diff) <= gamma:
                    Hp[k] += m
        # ---- Stage B ----
        Hb = np.where(Hp > self.tau_high, 1,
              np.where(Hp < self.tau_low, 0, self.h_b_prev)).astype(np.int8)
        self.h_b_prev = Hb
        # ---- Stage C (multirotor: masked == binary) ----
        Hm = Hb
        # ---- Stage D ----
        k_t     = int(round(target_heading_deg  / self.alpha)) % self.n
        k_theta = int(round(current_heading_deg / self.alpha)) % self.n
        candidates = self._candidates(Hm, k_t)
        if not candidates: return None             # brake / RTL
        def delta(a, b):
            d = abs(a - b);  return min(d, self.n - d)
        _, k_best = min((self.mu[0]*delta(c, k_t)
                       + self.mu[1]*delta(c, k_theta)
                       + self.mu[2]*delta(c, self.k_prev), c) for c in candidates)
        self.k_prev = k_best
        return (k_best * self.alpha) % 360.0

    def _candidates(self, Hm, k_t):
        if Hm.sum() == self.n: return []
        ext = np.concatenate([Hm, Hm])             # wrap-aware
        valleys, start = [], None
        for i, v in enumerate(ext):
            if v == 0 and start is None: start = i
            elif v == 1 and start is not None:
                valleys.append((start, i-1)); start = None
        seen, out = set(), []
        for kr, kl in valleys:
            key = (kr % self.n, kl % self.n)
            if key in seen: continue
            seen.add(key)
            width = kl - kr + 1
            if width >= self.s_max:
                out.extend([(kr + self.s_max//2) % self.n,
                            (kl - self.s_max//2) % self.n])
                if kr <= k_t <= kl: out.append(k_t)
            else:
                out.append(((kr + kl) // 2) % self.n)
        return out
```

**Converting θ_best to a velocity command (ArduPilot body frame, `MAV_FRAME_BODY_OFFSET_NED`):**
```python
v_cmd  = min(V_MAX, V_BASE * (1 - Hp[k_best] / max(Hp.max(), 1e-3)))  # slow if cluttered
th     = math.radians(theta_best_deg)
vx     =  v_cmd * math.cos(th)         # forward
vy     =  v_cmd * math.sin(th)         # right
vz     =  0.0                          # altitude held separately
```

### 3. pymavlink control of Pixhawk (no ROS)

**Install on Jetson** (Ubuntu 22.04 / JetPack 6):
```
sudo apt install python3-pip python3-lxml python3-future
pip3 install pymavlink==2.4.41 pyserial==3.5 numpy==1.26.4
```
Pin to `pymavlink==2.4.41` (May 2024) — newer releases occasionally introduce dialect changes; this version is widely tested with ArduCopter 4.5.x.

**Connection** (USB cable from Pixhawk to Jetson, recommended over UART for first integration):
```python
from pymavlink import mavutil
mav = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200, source_system=255)
mav.wait_heartbeat();  print("HB from sys", mav.target_system)
```
For UART (Pixhawk TELEM2 ↔ Jetson `/dev/ttyTHS1`): set ArduPilot params `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921`, and on Jetson disable `nvgetty`. Then `mavutil.mavlink_connection("/dev/ttyTHS1", baud=921600)`.

**Arm + GUIDED + takeoff** (pattern from the official `ardusub.com` pymavlink gitbook and ArduPilot dev docs):
```python
def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    while mav.recv_match(type="HEARTBEAT", blocking=True).custom_mode != mode_id:
        pass

set_mode(mav, "GUIDED")
mav.arducopter_arm();  mav.motors_armed_wait()
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0, 0,0, 5.0)        # 5 m
```

**Velocity loop — the heart of the control system**:
```python
import time
TYPE_MASK_VEL_ONLY = 0b0000111111000111            # ignore pos, accel, yaw

def send_body_velocity(mav, vx, vy, vz, yaw_rate=0.0):
    mav.mav.set_position_target_local_ned_send(
        int((time.time() - boot_t) * 1000),
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        TYPE_MASK_VEL_ONLY,
        0, 0, 0,                                    # x, y, z position (ignored)
        vx, vy, vz,                                 # velocity m/s, body frame
        0, 0, 0,                                    # acceleration (ignored)
        0.0, yaw_rate)

RATE_HZ = 10
while running:
    scan  = lidar.read_scan()
    theta = vfh.step(scan.angles_deg, scan.ranges_m,
                     target_heading_deg=goal_bearing(),
                     current_heading_deg=current_yaw_deg())
    if theta is None:
        send_body_velocity(mav, 0, 0, 0)            # brake
    else:
        vx, vy = vel_from_theta(theta, V_MAX)
        send_body_velocity(mav, vx, vy, 0)
    time.sleep(1.0 / RATE_HZ)
```

**ArduPilot vs PX4 differences that matter for this repo:**

| Concern | ArduPilot (GUIDED) | PX4 (OFFBOARD) |
|---|---|---|
| Pre-arm setpoint stream | Not required | *"sent for at least a second before PX4 will arm in offboard mode"* (PX4 docs) |
| Stop timeout | *"the vehicle will stop after 3 seconds if no command is received"* (ArduPilot dev docs) | *"500 ms between two Offboard commands"* (PX4 MAVROS docs) |
| Frame for body-velocity | `MAV_FRAME_BODY_OFFSET_NED` | `MAV_FRAME_BODY_NED` |
| RC kill | Mode switch to STABILIZE/RTL | Switch out of OFFBOARD via RC |

Recommend **ArduPilot** for a student/SUAS project — the failsafe surface is much friendlier and the documentation is more complete.

### 4. Simulation

**4a. Pure ArduPilot SITL (fastest dev loop, no Gazebo)**
```
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
cd ArduCopter
sim_vehicle.py -v ArduCopter --console --map -w
```
SITL exposes MAVLink on UDP 127.0.0.1:14550. Point your script at `udpin:127.0.0.1:14551` and use MAVProxy `output add 127.0.0.1:14551` to fork the stream so QGroundControl can stay on 14550. For LiDAR in this mode, **inject synthetic scans** in your test harness — SITL's built-in 360° lidar simulation (`PRX1_TYPE=16` for LD06) drives ArduPilot's *own* avoidance, not your VFH stack, so it is the wrong tool here.

**4b. ArduPilot SITL + Gazebo Harmonic (recommended end-to-end)**
On Ubuntu 22.04 — the `ardupilot_gazebo` README explicitly states Harmonic is the recommended target:
```
# Gazebo Harmonic
sudo apt install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install gz-harmonic libgz-sim8-dev rapidjson-dev

# ardupilot_gazebo plugin (official)
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make -j4
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```
Run the Iris-with-LiDAR world (you author this by forking `iris_runway.sdf`, adding boxes/cylinders, and adding a `<sensor type="gpu_lidar">` child to `iris_with_ardupilot/model.sdf`):
```
gz sim -v4 -r sim/worlds/iris_avoidance.sdf                                       # terminal 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console          # terminal 2
python -m uav_avoidance.run --sim                                                  # terminal 3
```
The `gpu_lidar` SDF tag is straightforward — typical 2D config (Gazebo Harmonic sensor tutorial):
```xml
<sensor name="lidar2d" type="gpu_lidar">
  <update_rate>10</update_rate>
  <topic>lidar_scan</topic>
  <lidar>
    <scan><horizontal>
      <samples>360</samples><resolution>1</resolution>
      <min_angle>-3.1416</min_angle><max_angle>3.1416</max_angle>
    </horizontal></scan>
    <range><min>0.05</min><max>12.0</max><resolution>0.01</resolution></range>
  </lidar>
  <visualize>true</visualize>
</sensor>
```
Read the `gz.msgs.LaserScan` topic in Python via the `gz-transport13` bindings (`pip install gz-transport13` after `apt install libgz-transport13-dev`), or pipe through a tiny `gz topic -e` subprocess that emits JSON — both are documented in the Gazebo Harmonic sensor tutorial.

**4c. PX4 + Gazebo (optional alternative)**
```
git clone https://github.com/PX4/PX4-Autopilot --recursive
cd PX4-Autopilot
make px4_sitl gz_x500
```
Drive the same Python script at `udpin:127.0.0.1:14540`. Differences in MAV frames and the 2 Hz proof-of-life rule apply.

### 5. Repository structure

```
uav-vfh-avoidance/
├── README.md
├── LICENSE                            # MIT
├── pyproject.toml                     # PEP 621 metadata + ruff/black config
├── requirements.txt                   # pinned
├── requirements-dev.txt               # ruff, black, pytest, mypy
├── .pre-commit-config.yaml
├── .github/workflows/ci.yml           # lint + unit tests on push
├── docs/
│   ├── architecture.md                # system block diagram
│   ├── wiring.md                      # Jetson ↔ LD19, Jetson ↔ Pixhawk pinouts
│   ├── algorithm.md                   # VFH+ math walk-through with figures
│   ├── safety.md                      # pre-flight + failsafe
│   └── images/                        # diagrams, demo gifs
├── src/uav_avoidance/
│   ├── __init__.py
│   ├── config.py                      # dataclass: all tunables in one place
│   ├── lidar/
│   │   ├── ld19.py                    # parser shown above
│   │   ├── filters.py                 # median, intensity, prop-mask
│   │   └── synthetic.py               # generate scans from boxes for SITL-only mode
│   ├── vfh/
│   │   ├── vfh_plus.py                # algorithm shown above
│   │   └── plotting.py                # matplotlib polar plot for debug
│   ├── mavlink/
│   │   ├── controller.py              # connect, arm, mode, takeoff, RTL
│   │   ├── velocity.py                # send_body_velocity, rate-limited
│   │   └── telemetry.py               # async heartbeat + position listener
│   ├── pipeline.py                    # the 10 Hz loop wiring it all together
│   └── run.py                         # __main__: argparse, --sim / --hw
├── sim/
│   ├── worlds/iris_avoidance.sdf
│   ├── models/iris_lidar/model.sdf    # iris_with_ardupilot + gpu_lidar
│   ├── params/copter_avoid.parm       # SITL param file (geofence, max v, etc.)
│   └── launch/sitl_gazebo.sh
├── scripts/
│   ├── ld19_dump.py                   # smoke-test the LiDAR alone
│   ├── mavlink_ping.py                # smoke-test Pixhawk link
│   └── plot_log.py                    # post-flight log analysis
├── tests/
│   ├── test_ld19_parser.py            # CRC, packet sync, hand-crafted bytes
│   ├── test_vfh_plus.py               # single obstacle, two valleys, dead-end
│   ├── test_mavlink_messages.py       # type_mask, message factory
│   └── data/                          # captured LD19 packet dumps
└── examples/
    ├── 01_lidar_only.py
    ├── 02_vfh_offline_replay.py
    └── 03_sitl_full_loop.py
```

**Pinned `requirements.txt`:**
```
pyserial==3.5
pymavlink==2.4.41
numpy==1.26.4
matplotlib==3.8.4
scipy==1.13.0
```
**`requirements-dev.txt`**: `ruff==0.4.4`, `black==24.4.2`, `pytest==8.2.1`, `mypy==1.10.0`, `pre-commit==3.7.1`.

**CI (`.github/workflows/ci.yml`)**: matrix on Python 3.10/3.11, run `ruff check .`, `black --check .`, `pytest -q`. No hardware-in-the-loop in CI — that lives in `examples/03_sitl_full_loop.py` and is gated behind a `[sitl]` install extra.

### 6. README template (the most important file)

The README is what makes the repo "GitHub-ready". Sections, in order, modeled on `AtsushiSakai/PythonRobotics` and `PX4/PX4-Avoidance`:

1. **Title + one-line description + badges** (build status, license, Python version).
2. **Demo GIF / screenshot** above the fold (record the SITL run).
3. **System architecture** — a single PNG showing LD19 → Jetson → VFH → pymavlink → Pixhawk → motors, with the simulation alternative branch.
4. **Hardware BOM** — Jetson Orin Nano Developer Kit, Pixhawk (specify revision, e.g., Cube Orange), LDRobot D500 kit (LD19 + CP2102), 4S LiPo, ESCs/motors, frame.
5. **Wiring** — table + photo. CP2102 USB → Jetson USB. Pixhawk USB → Jetson USB (primary) or Pixhawk TELEM2 → Jetson UART1 (advanced).
6. **Software install** — `git clone`, `pip install -r requirements.txt`, system deps (`libgz-sim8-dev` if using Gazebo).
7. **Simulation quickstart** — three terminals, copy-paste commands, expected output.
8. **Hardware deployment** — `systemd` service file template, autostart on boot, log location.
9. **Algorithm explanation** — link to `docs/algorithm.md`, embed the key VFH+ polar-histogram figure.
10. **Configuration reference** — table of every tunable in `config.py` with default value and recommended range.
11. **Troubleshooting** — top 10 failure modes (no heartbeat, EKF not happy, LiDAR not enumerating, GUIDED rejected, etc.).
12. **Safety / disclaimer** — explicit "this is a research / student project, do not fly over people".
13. **References** — links to: Ulrich & Borenstein 1998 (`cs.cmu.edu/~iwan/papers/vfh+.pdf`), LD19 development manual, ArduPilot GUIDED docs, pymavlink mavgen guide, `ldrobotSensorTeam/ldlidar_stl_sdk`, `vanderbiltrobotics/vfh-python`.

### 7. Safety and practical considerations

- **Velocity > position setpoints for dynamic avoidance** because velocity setpoints decay to zero on link loss (drone stops); position setpoints continue executing toward an obsolete waypoint. Also smoother on noisy LiDAR data.
- **Continuous streaming**: schedule the 10 Hz loop with a monotonic clock, not `sleep(0.1)`, to keep jitter low. *"The vehicle will stop after 3 seconds if no command is received"* (ArduPilot dev docs); PX4 *"has a timeout of 500 ms between two Offboard commands"* (PX4 MAVROS docs).
- **Geofence**: in SITL params, set `FENCE_ENABLE=1`, `FENCE_TYPE=7` (alt+circle+polygon), `FENCE_ALT_MAX=20`, `FENCE_RADIUS=50`. On breach → RTL.
- **Velocity cap**: `V_MAX=2.0 m/s` for first flights. Also set ArduPilot `WPNAV_SPEED=200` (cm/s).
- **Pre-flight checklist** (encode as `scripts/preflight.py` that exits non-zero if any item fails):
  1. Heartbeat received within 5 s.
  2. EKF flags all green (`EKF_STATUS_REPORT.flags & 0x01F == 0x01F`).
  3. GPS fix type ≥ 3 with HDOP < 1.5.
  4. Battery voltage > nominal.
  5. LD19 publishing > 4000 points/sec (i.e., motor actually spinning).
  6. RC channel 8 (avoidance kill) reads > 1700 µs (engaged).
  7. Geofence enabled.

---

## Recommendations

**Build in this order — each stage is a green-light gate:**

1. **Week 1 — LiDAR alone.** Implement `lidar/ld19.py` + `scripts/ld19_dump.py`. Verify on bench: spin the LiDAR pointing at a wall 2 m away and check the polar plot. Add `tests/test_ld19_parser.py` with hand-crafted packets and CRC checks. **Gate: 360° scan visualised in matplotlib at 10 Hz.**

2. **Week 2 — VFH+ offline.** Implement `vfh/vfh_plus.py`. Write `examples/02_vfh_offline_replay.py` that loads a recorded scan from `tests/data/` and shows the polar histogram + selected steering angle. Unit-test against synthetic worlds (single wall, narrow doorway, dead-end). **Gate: dead-end produces `theta=None` (brake), doorway produces a direction within the doorway.**

3. **Week 3 — MAVLink alone.** Implement `mavlink/controller.py`, then run SITL and execute `examples/03_sitl_full_loop.py` with a *constant* velocity command (no LiDAR). Confirm the drone arms, takes off, flies a square via body-frame velocity, lands. **Gate: SITL square pattern visible in MAVProxy map.**

4. **Week 4 — Integration in SITL with synthetic LiDAR.** Wire VFH+ output → velocity setpoints, feed synthetic obstacle fields (`lidar/synthetic.py`). **Gate: drone navigates around a single cylinder placed between start and goal in the synthetic world.**

5. **Week 5 — Gazebo full sim.** Author the iris+lidar SDF, run end-to-end with simulated LiDAR. **Gate: 3 obstacle scenarios pass: single box, corridor with two boxes, slalom.**

6. **Week 6 — Hardware bench.** Tape the drone to the bench, props off. Verify the LiDAR-on-the-real-airframe scan is clean (look for prop-arm artefacts → add masks to `lidar/filters.py`). Verify MAVLink over USB to Pixhawk, EKF healthy, GUIDED accepted.

7. **Week 7 — Tethered hover.** Tether to a 3 m cord. First flights at 1 m altitude, 0.5 m/s velocity cap. Engage avoidance only over a soft (foam) obstacle.

8. **Week 8 — Free flight.** Outdoor, large area, RC kill switch in hand.

**Decision thresholds that should change the plan:**
- If the LD19 scan has **> 10 % invalid points** indoors → swap to a higher-end DTOF before continuing. The **LDROBOT STL-27L (ToF, 10 Hz, 21.6 kHz sample rate, 0.03–25 m range, ±15 mm accuracy)** is listed at **$142** in `kaiaai/awesome-2d-lidars` (DFRobot retail $160) and is a drop-in protocol-compatible upgrade.
- If VFH+ produces oscillation between two valleys → increase μ3 (commitment) from 2 → 3 (keeping μ1 > μ2+μ3).
- If ArduPilot GUIDED feels "jerky" → switch to PX4 OFFBOARD with a `TrajectorySetpoint` that includes a velocity feedforward.
- If you need vertical avoidance (drones flying under wires) → upgrade to VFH* or 3D-VFH, but that is out of scope for this repo per the user's instructions.

---

## Caveats

- **The LD19's 12 m range is a best case** (70 % reflective target, per the DFRobot product spec). Outdoors against grass, sky, or sun glare, expect 4–6 m effective range and high noise. The Waveshare wiki specifies the LD19 *"can resist the strong light environment of 30000 lux"* — usable indoors and on overcast days, but **bright sunlight provides ~98,000 lux on a perpendicular surface at sea level** (Wikipedia, *Sunlight*), so direct-sun outdoor operation is outside spec. Plan flights for diffuse/cloudy conditions or shade.
- **Hysteresis thresholds τ_low and τ_high are not given numerical defaults in the VFH+ paper** — Ulrich & Borenstein deliberately leave them as application-tuned parameters. Treat the values in this report as starting guesses, not specifications.
- **GUIDED-mode body-frame velocity in ArduPilot uses `MAV_FRAME_BODY_OFFSET_NED`, not `MAV_FRAME_BODY_NED`**. The latter exists in the MAVLink spec but is not what ArduPilot expects. PX4 is the opposite. Get this wrong and your drone flies in the wrong direction.
- **`vanderbiltrobotics/vfh-python` is a small, lightly-maintained repo that only implements the original 1991 VFH**, not VFH+. Use it for the data-structure shape (`HistogramGrid` / `PolarHistogram` / `PathPlanner` classes) but write the binary-histogram, masked-histogram, and cost-function pieces yourself per the 1998 paper.
- **DroneKit-Python is deprecated** (3DRobotics archived it). Do not depend on it. The pymavlink-only code shown here is the modern path. MAVSDK-Python is an alternative but adds gRPC complexity that this repo does not need.
- **Jetson Orin Nano GPIO UART is 3.3 V** — same level as the LD19 — so direct UART works electrically, but the serial-console-on-boot issue (`nvgetty.service`) bites everyone. The USB-via-CP2102 path is recommended unless you have specific reasons to free up the USB port.
- **All numeric SITL parameters in this report (e.g., `WPNAV_SPEED=200`) are reasonable starting points**, not validated by the user's airframe. They will need tuning during the bench-test phase.
- **CRC-8 lookup table truncated in code blocks**: the full 256-byte LD19 CRC table (poly 0x4D, init 0x00, no reflection) appears verbatim in `ldrobotSensorTeam/ldlidar_stl_sdk` (`include/lipkg.h`) and in `henjin0/LIDAR_LD06_python_loader` (`crc_utils.py`); copy it from there rather than retyping.