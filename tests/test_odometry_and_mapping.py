"""Tests for encoder odometry, VL53L0X ToF raycasting, and 2D occupancy grid mapping."""

from __future__ import annotations

import math

import pytest

from simulator.config import RoverConfig
from simulator.engine import RoverSimulationEngine
from simulator.mapping import OccupancyGrid2D
from simulator.sensors import Vl53l0xArraySim
from simulator.world import RescueWorld, WallSegment


def test_straight_line_odometry_accuracy(sim_engine: RoverSimulationEngine) -> None:
    sim_engine.reset(x_m=0.0, y_m=0.0, theta_rad=0.0)
    # Empty open room so robot can drive unimpeded
    sim_engine.world = RescueWorld(
        bounds_min=-4.5,
        bounds_max=4.5,
        walls=[
            WallSegment(-4.5, -4.5, 4.5, -4.5),
            WallSegment(4.5, -4.5, 4.5, 4.5),
            WallSegment(4.5, 4.5, -4.5, 4.5),
            WallSegment(-4.5, 4.5, -4.5, -4.5),
        ],
    )
    sim_engine.set_manual_velocity(0.35, 0.0)
    for step in range(100):  # 2.0s at 50 Hz
        if step % 10 == 0:
            sim_engine.feed_heartbeat()
        sim_engine.step(0.02)

    assert sim_engine.odom.pose.x_m == pytest.approx(sim_engine.true_x, abs=0.04)
    assert sim_engine.odom.pose.y_m == pytest.approx(0.0, abs=0.03)
    assert sim_engine.true_x > 0.55


def test_tof_raycast_measures_exact_wall_distance(rover_config: RoverConfig) -> None:
    world = RescueWorld(
        bounds_min=-4.0,
        bounds_max=4.0,
        walls=[WallSegment(1.0, -2.0, 1.0, 2.0)],
    )
    tof = Vl53l0xArraySim(rover_config, noise_std_m=0.0)
    ranges = tof.scan(true_x=0.0, true_y=0.0, true_theta=0.0, world=world)
    # Sensor mounted +0.105m forward -> distance to wall at x=1.0 is 0.895m
    assert ranges[2] == pytest.approx(1.0 - rover_config.tof_mount_offset_x_m, abs=0.005)
    # Side beams at +/- 30 deg hit wall at 0.895 / cos(30 deg)
    expected_30 = (1.0 - rover_config.tof_mount_offset_x_m) / math.cos(math.radians(30.0))
    assert ranges[1] == pytest.approx(expected_30, abs=0.01)
    assert ranges[3] == pytest.approx(expected_30, abs=0.01)


def test_occupancy_grid_marks_free_space_and_wall_hit(rover_config: RoverConfig) -> None:
    grid = OccupancyGrid2D(rover_config)
    world = RescueWorld(
        bounds_min=-4.0,
        bounds_max=4.0,
        walls=[WallSegment(1.05, -2.0, 1.05, 2.0)],
    )
    tof = Vl53l0xArraySim(rover_config, noise_std_m=0.0)
    ranges = tof.scan(0.0, 0.0, 0.0, world)

    for _ in range(4):
        grid.update_from_scan(0.0, 0.0, 0.0, ranges)

    free_cell = grid.world_to_grid(0.45, 0.0)
    hit_cell = grid.world_to_grid(1.05, 0.0)
    assert free_cell is not None and hit_cell is not None
    assert grid.cell_probability(*free_cell) < 0.35
    assert grid.cell_probability(*hit_cell) > 0.70
    stats = grid.summary_stats()
    assert stats["explored_cells"] > 10
    assert stats["occupied_cells"] >= 1
