"""Central configuration for the obstacle-avoidance stack.

Every tunable used anywhere in the pipeline lives here so the rest of the
code stays free of magic numbers. Override at the command line via
``run.py`` or programmatically by constructing a new ``Config``.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LidarConfig:
    """Serial + filtering parameters for the LD19 / STL-19P."""

    port: str = "/dev/ttyUSB0"
    baud: int = 230400
    timeout_s: float = 0.05

    # Low-confidence reflections — see LD19 manual §filtering recommendations.
    min_intensity: int = 50

    # Treat any point outside this annulus as "no return" (set to range_max).
    range_min_m: float = 0.02
    range_max_m: float = 12.0

    # Width of the 3-tap median filter (must be odd). Set to 1 to disable.
    median_window: int = 3

    # Angular cone to mask out as "the propeller arm" — start/end in degrees.
    # Default disabled; configure once the LiDAR is mounted on the airframe.
    prop_mask_deg: tuple[float, float] | None = None


@dataclass
class VFHPlusConfig:
    """VFH+ parameters following Ulrich & Borenstein, ICRA 1998."""

    n_sectors: int = 72  # 360 / alpha_deg
    alpha_deg: float = 5.0
    r_safe_m: float = 0.65  # drone radius + safety margin d_s
    d_max_m: float = 5.0  # planning horizon (sensor reaches 12 m)

    # Cell-magnitude weights — see eq. 2-3 of the paper.
    # b=1, a chosen so a - b * ((w_s - 1) / 2)**2 == 1 with w_s = 29 cells.
    a: float = 197.0
    b: float = 1.0

    # Hysteresis thresholds for the binary histogram (eq. 7).
    # The paper leaves these as application-tuned; start here and tune.
    tau_low: float = 197.0  # 1.0 * a
    tau_high: float = 591.0  # 3.0 * a

    # Wide vs. narrow valley boundary (sectors).
    s_max: int = 16

    # Cost weights — paper recommends mu1 > mu2 + mu3.
    mu1: float = 5.0  # goal attraction
    mu2: float = 2.0  # heading commitment
    mu3: float = 2.0  # previous-direction commitment


@dataclass
class MAVLinkConfig:
    """How and where to talk to the autopilot."""

    # Either a serial device ("/dev/ttyACM0") or a MAVLink URL
    # ("udpin:127.0.0.1:14551" for SITL, "tcp:127.0.0.1:5760" etc.).
    connection: str = "/dev/ttyACM0"
    baud: int = 115200
    source_system: int = 255

    # Flight-mode name used for offboard control on ArduPilot.
    flight_mode: str = "GUIDED"

    # Takeoff altitude in metres.
    takeoff_alt_m: float = 5.0

    # Velocity setpoint cadence — 10 Hz keeps both ArduPilot's 3 s timeout
    # and PX4's 500 ms timeout safely cleared.
    rate_hz: float = 10.0


@dataclass
class SafetyConfig:
    """Hard envelope the pipeline will not cross even if VFH+ says otherwise."""

    v_max_horizontal_mps: float = 2.0
    v_max_vertical_mps: float = 0.5
    v_base_mps: float = 1.0  # base speed before clutter-derived slowdown

    # Minimum acceptable heartbeat age in seconds. Above this -> brake + RTL.
    heartbeat_timeout_s: float = 1.0

    # RC kill switch (channel index in PWM input map).
    rc_kill_channel: int = 8
    rc_kill_threshold_us: int = 1700


@dataclass
class Config:
    lidar: LidarConfig = field(default_factory=LidarConfig)
    vfh: VFHPlusConfig = field(default_factory=VFHPlusConfig)
    mavlink: MAVLinkConfig = field(default_factory=MAVLinkConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
