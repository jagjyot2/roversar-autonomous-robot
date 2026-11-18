"""Tests for differential-drive kinematics and wheel PID speed tracking."""

from __future__ import annotations

import math

import pytest

from simulator.config import RoverConfig
from simulator.kinematics import (
    DiffDriveKinematics,
    WheelActuatorPlant,
    WheelPidController,
)


def test_forward_inverse_kinematics_roundtrip(rover_config: RoverConfig) -> None:
    kin = DiffDriveKinematics(rover_config)
    test_twists = [
        (0.40, 0.0),
        (0.0, 2.0),
        (0.32, 1.15),
        (-0.25, -0.90),
    ]
    for v_cmd, w_cmd in test_twists:
        wheels = kin.inverse(v_cmd, w_cmd)
        recovered = kin.forward(wheels.left_rad_s, wheels.right_rad_s)
        assert recovered.linear_mps == pytest.approx(v_cmd, abs=1e-4)
        assert recovered.angular_rps == pytest.approx(w_cmd, abs=1e-4)


def test_inverse_kinematics_curvature_preserving_saturation(rover_config: RoverConfig) -> None:
    kin = DiffDriveKinematics(rover_config)
    wheels = kin.inverse(0.60, 3.50)
    assert max(abs(wheels.left_rad_s), abs(wheels.right_rad_s)) <= rover_config.max_wheel_rad_s + 1e-6


def test_rpm_rad_s_conversion() -> None:
    rad_s = 2.0 * math.pi * 2.0  # 2 rev/s = 120 RPM
    rpm = DiffDriveKinematics.rad_s_to_rpm(rad_s)
    assert rpm == pytest.approx(120.0, abs=1e-6)
    assert DiffDriveKinematics.rpm_to_rad_s(rpm) == pytest.approx(rad_s, abs=1e-6)


def test_wheel_pid_tracks_target_velocity(rover_config: RoverConfig) -> None:
    pid = WheelPidController()
    plant = WheelActuatorPlant(rover_config, encoder_noise_std=0.0)
    dt = 0.02
    target_rad_s = 10.0  # ~0.325 m/s wheel speed

    measured = 0.0
    for _ in range(100):  # 2.0 seconds
        effort = pid.update(target_rad_s, measured, dt)
        dl, _, true_wl, _ = plant.step(effort, effort, motors_enabled=True, dt_s=dt)
        measured = (dl * (2.0 * math.pi / rover_config.encoder_ticks_per_rev)) / dt

    assert plant.true_left_rad_s == pytest.approx(target_rad_s, abs=0.45)
