"""Simulated VL53L0X ToF raycaster, MPU6050 IMU, encoder odometry, and battery pack."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from simulator.config import RoverConfig
from simulator.kinematics import clamp, wrap_angle
from simulator.world import RescueWorld


@dataclass
class Pose2D:
    x_m: float = -2.2
    y_m: float = 0.0
    theta_rad: float = 0.0
    left_rad_s: float = 0.0
    right_rad_s: float = 0.0
    linear_mps: float = 0.0
    angular_rps: float = 0.0


@dataclass
class ImuState:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_rad: float = 0.0
    yaw_rate_rps: float = 0.0
    healthy: bool = True


@dataclass
class BatteryState:
    voltage_v: float = 8.15
    soc_percent: float = 89.6
    low_warning: bool = False
    critical_cutoff: bool = False


class Vl53l0xArraySim:
    """5-channel VL53L0X Time-of-Flight laser raycaster."""

    def __init__(self, config: RoverConfig | None = None, noise_std_m: float = 0.008) -> None:
        self.cfg = config or RoverConfig()
        self.noise_std_m = noise_std_m
        self._rng = random.Random(101)

    def scan(self, true_x: float, true_y: float, true_theta: float, world: RescueWorld) -> list[float]:
        sensor_x = true_x + self.cfg.tof_mount_offset_x_m * math.cos(true_theta)
        sensor_y = true_y + self.cfg.tof_mount_offset_x_m * math.sin(true_theta)
        ranges: list[float] = []
        for rel_ang in self.cfg.tof_angles_rad:
            raw = world.cast_ray(sensor_x, sensor_y, true_theta + rel_ang, self.cfg.tof_max_range_m)
            if self.noise_std_m > 0.0 and raw < self.cfg.tof_max_range_m - 0.02:
                raw += self._rng.gauss(0.0, self.noise_std_m)
            ranges.append(round(clamp(raw, self.cfg.tof_min_range_m, self.cfg.tof_max_range_m), 4))
        return ranges


class EncoderOdometry:
    """RK2 differential-drive encoder odometry with optional IMU yaw rate fusion."""

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self.pose = Pose2D()

    def reset(self, x_m: float = -2.2, y_m: float = 0.0, theta_rad: float = 0.0) -> None:
        self.pose = Pose2D(x_m=x_m, y_m=y_m, theta_rad=wrap_angle(theta_rad))

    def update(
        self,
        d_ticks_l: int,
        d_ticks_r: int,
        dt_s: float,
        imu_yaw_rate_rps: float | None = None,
    ) -> Pose2D:
        if dt_s <= 1e-5:
            return self.pose

        rad_per_tick = (2.0 * math.pi) / self.cfg.encoder_ticks_per_rev
        d_phi_l = d_ticks_l * rad_per_tick
        d_phi_r = d_ticks_r * rad_per_tick

        ds_l = d_phi_l * self.cfg.wheel_radius_m
        ds_r = d_phi_r * self.cfg.wheel_radius_m
        ds = 0.5 * (ds_l + ds_r)
        d_theta_enc = (ds_r - ds_l) / self.cfg.wheel_base_m

        if imu_yaw_rate_rps is not None:
            d_theta = 0.70 * (imu_yaw_rate_rps * dt_s) + 0.30 * d_theta_enc
        else:
            d_theta = d_theta_enc

        theta_mid = self.pose.theta_rad + 0.5 * d_theta
        self.pose.x_m += ds * math.cos(theta_mid)
        self.pose.y_m += ds * math.sin(theta_mid)
        self.pose.theta_rad = wrap_angle(self.pose.theta_rad + d_theta)
        self.pose.left_rad_s = d_phi_l / dt_s
        self.pose.right_rad_s = d_phi_r / dt_s
        self.pose.linear_mps = ds / dt_s
        self.pose.angular_rps = d_theta / dt_s
        return self.pose


class BatterySim:
    """2S Li-Ion battery discharge and voltage sag model."""

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self.resting_v = 8.15
        self.state = BatteryState(voltage_v=self.resting_v, soc_percent=89.6)

    def reset(self, voltage_v: float = 8.15) -> None:
        self.resting_v = voltage_v
        self.step(0.0, 0.0, 0.02)

    def step(self, left_effort: float, right_effort: float, dt_s: float) -> BatteryState:
        current_a = 0.25 + 1.15 * (abs(left_effort) + abs(right_effort))
        # Gentle coulomb discharge over time (2200 mAh 2S pack ~ 7920 As)
        self.resting_v = max(5.6, self.resting_v - (current_a * dt_s) / 3600.0 * 0.45)
        sag_v = current_a * 0.045
        terminal_v = max(5.5, self.resting_v - sag_v)
        soc = clamp((terminal_v - 6.0) / (8.4 - 6.0) * 100.0, 0.0, 100.0)
        self.state = BatteryState(
            voltage_v=round(terminal_v, 3),
            soc_percent=round(soc, 1),
            low_warning=terminal_v <= self.cfg.vbat_low_warn_v,
            critical_cutoff=terminal_v <= self.cfg.vbat_critical_cutoff_v,
        )
        return self.state
