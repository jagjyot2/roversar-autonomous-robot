"""Shared physical and safety configuration constants for RoverSAR."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class RoverConfig:
    """Physical chassis, kinematic, sensor, and safety thresholds."""

    # Chassis envelope: 240mm x 180mm x 95mm
    chassis_length_m: float = 0.240
    chassis_width_m: float = 0.180
    chassis_height_m: float = 0.095
    collision_radius_m: float = 0.130

    # Differential drive kinematics
    wheel_radius_m: float = 0.0325
    wheel_base_m: float = 0.1600
    encoder_ticks_per_rev: int = 1440
    max_wheel_rad_s: float = 18.85
    max_linear_mps: float = 0.60
    max_angular_rps: float = 3.50
    motor_time_constant_s: float = 0.075

    # 5-beam VL53L0X ToF array (-60, -30, 0, +30, +60 deg)
    tof_mount_offset_x_m: float = 0.105
    tof_angles_rad: tuple[float, ...] = field(
        default_factory=lambda: tuple(
            math.radians(deg) for deg in (-60.0, -30.0, 0.0, 30.0, 60.0)
        )
    )
    tof_min_range_m: float = 0.03
    tof_max_range_m: float = 2.00

    # 2D Occupancy Grid (10m x 10m arena @ 0.10 m/cell)
    grid_width: int = 100
    grid_height: int = 100
    grid_resolution_m: float = 0.10
    grid_origin_x_m: float = -5.0
    grid_origin_y_m: float = -5.0

    # Safety & Watchdog
    watchdog_timeout_ms: int = 500
    max_safe_tilt_deg: float = 35.0
    vbat_nominal_v: float = 7.40
    vbat_full_v: float = 8.40
    vbat_low_warn_v: float = 6.40
    vbat_critical_cutoff_v: float = 6.00

    # Simulation Loop
    sim_hz: int = 50
