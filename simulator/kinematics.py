"""Differential-drive kinematics, wheel velocity PID, and motor/encoder plant."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from simulator.config import RoverConfig


def wrap_angle(rad: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


@dataclass
class WheelTargets:
    left_rad_s: float = 0.0
    right_rad_s: float = 0.0


@dataclass
class Twist2D:
    linear_mps: float = 0.0
    angular_rps: float = 0.0


class DiffDriveKinematics:
    """Forward and inverse differential-drive kinematic equations."""

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()

    def inverse(self, linear_mps: float, angular_rps: float) -> WheelTargets:
        v = clamp(linear_mps, -self.cfg.max_linear_mps, self.cfg.max_linear_mps)
        w = clamp(angular_rps, -self.cfg.max_angular_rps, self.cfg.max_angular_rps)

        r = self.cfg.wheel_radius_m
        L = self.cfg.wheel_base_m

        omega_l = (v - 0.5 * L * w) / r
        omega_r = (v + 0.5 * L * w) / r

        max_mag = max(abs(omega_l), abs(omega_r))
        if max_mag > self.cfg.max_wheel_rad_s and max_mag > 1e-6:
            scale = self.cfg.max_wheel_rad_s / max_mag
            omega_l *= scale
            omega_r *= scale

        return WheelTargets(left_rad_s=omega_l, right_rad_s=omega_r)

    def forward(self, left_rad_s: float, right_rad_s: float) -> Twist2D:
        r = self.cfg.wheel_radius_m
        L = self.cfg.wheel_base_m
        v = 0.5 * r * (right_rad_s + left_rad_s)
        w = (r / L) * (right_rad_s - left_rad_s)
        return Twist2D(linear_mps=v, angular_rps=w)

    @staticmethod
    def rad_s_to_rpm(rad_s: float) -> float:
        return rad_s * 60.0 / (2.0 * math.pi)

    @staticmethod
    def rpm_to_rad_s(rpm: float) -> float:
        return rpm * (2.0 * math.pi) / 60.0


class WheelPidController:
    """Discrete PID wheel velocity controller with feedforward and anti-windup."""

    def __init__(
        self,
        kp: float = 0.085,
        ki: float = 0.220,
        kd: float = 0.002,
        kf: float = 0.052,
        out_min: float = -1.0,
        out_max: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def update(self, setpoint_rad_s: float, measured_rad_s: float, dt_s: float) -> float:
        if dt_s <= 1e-5:
            return 0.0
        if abs(setpoint_rad_s) < 0.02 and abs(measured_rad_s) < 0.08:
            self.reset()
            return 0.0

        error = setpoint_rad_s - measured_rad_s
        p_term = self.kp * error
        tentative_i = self.integral + error * dt_s
        d_term = 0.0
        if self.has_prev:
            d_term = self.kd * ((error - self.prev_error) / dt_s)
        self.prev_error = error
        self.has_prev = True

        ff_term = self.kf * setpoint_rad_s
        raw = ff_term + p_term + self.ki * tentative_i + d_term

        saturated_hi = raw > self.out_max and error > 0.0
        saturated_lo = raw < self.out_min and error < 0.0
        if not saturated_hi and not saturated_lo:
            self.integral = tentative_i

        return clamp(ff_term + p_term + self.ki * self.integral + d_term, self.out_min, self.out_max)


class WheelActuatorPlant:
    """First-order DC geared motor + quadrature encoder tick plant."""

    def __init__(self, config: RoverConfig | None = None, encoder_noise_std: float = 0.0) -> None:
        self.cfg = config or RoverConfig()
        self.encoder_noise_std = encoder_noise_std
        self.true_left_rad_s = 0.0
        self.true_right_rad_s = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self._left_frac_ticks = 0.0
        self._right_frac_ticks = 0.0
        self._rng = random.Random(42)

    def reset(self) -> None:
        self.true_left_rad_s = 0.0
        self.true_right_rad_s = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self._left_frac_ticks = 0.0
        self._right_frac_ticks = 0.0

    def step(
        self,
        left_effort: float,
        right_effort: float,
        motors_enabled: bool,
        dt_s: float,
    ) -> tuple[int, int, float, float]:
        """Advance wheel physics and return (d_ticks_l, d_ticks_r, true_wl, true_wr)."""
        if not motors_enabled:
            target_l = 0.0
            target_r = 0.0
            tau = 0.035  # Active electromagnetic brake / friction halt
        else:
            target_l = clamp(left_effort, -1.0, 1.0) * self.cfg.max_wheel_rad_s
            target_r = clamp(right_effort, -1.0, 1.0) * self.cfg.max_wheel_rad_s
            tau = self.cfg.motor_time_constant_s

        alpha = dt_s / (tau + dt_s)
        self.true_left_rad_s += alpha * (target_l - self.true_left_rad_s)
        self.true_right_rad_s += alpha * (target_r - self.true_right_rad_s)

        if not motors_enabled:
            if abs(self.true_left_rad_s) < 0.05:
                self.true_left_rad_s = 0.0
            if abs(self.true_right_rad_s) < 0.05:
                self.true_right_rad_s = 0.0

        ticks_per_rad = self.cfg.encoder_ticks_per_rev / (2.0 * math.pi)
        noise_l = self._rng.gauss(0.0, self.encoder_noise_std) if self.encoder_noise_std > 0 else 0.0
        noise_r = self._rng.gauss(0.0, self.encoder_noise_std) if self.encoder_noise_std > 0 else 0.0

        self._left_frac_ticks += (self.true_left_rad_s * dt_s + noise_l) * ticks_per_rad
        self._right_frac_ticks += (self.true_right_rad_s * dt_s + noise_r) * ticks_per_rad

        dl = int(math.trunc(self._left_frac_ticks))
        dr = int(math.trunc(self._right_frac_ticks))
        self._left_frac_ticks -= dl
        self._right_frac_ticks -= dr

        self.left_ticks += dl
        self.right_ticks += dr
        return dl, dr, self.true_left_rad_s, self.true_right_rad_s
