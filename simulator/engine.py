"""Integrated 50 Hz RoverSAR differential-drive simulation engine."""

from __future__ import annotations

import math
import time

from simulator.config import RoverConfig
from simulator.kinematics import (
    DiffDriveKinematics,
    Twist2D,
    WheelActuatorPlant,
    WheelPidController,
    wrap_angle,
)
from simulator.mapping import OccupancyGrid2D
from simulator.navigation import WaypointFollower
from simulator.safety import DriveMode, SafetyFault, SafetySupervisor
from simulator.sensors import BatterySim, EncoderOdometry, ImuState, Vl53l0xArraySim
from simulator.world import RescueWorld


class RoverSimulationEngine:
    """Full physics, sensor, SLAM mapping, autonomous navigation, and safety engine."""

    def __init__(
        self,
        config: RoverConfig | None = None,
        world: RescueWorld | None = None,
        sensor_noise: bool = True,
    ) -> None:
        self.cfg = config or RoverConfig()
        self.world = world or RescueWorld.default_sar_arena()

        self.kinematics = DiffDriveKinematics(self.cfg)
        self.left_pid = WheelPidController()
        self.right_pid = WheelPidController()
        self.plant = WheelActuatorPlant(
            self.cfg,
            encoder_noise_std=0.002 if sensor_noise else 0.0,
        )
        self.tof_sim = Vl53l0xArraySim(
            self.cfg,
            noise_std_m=0.008 if sensor_noise else 0.0,
        )
        self.odom = EncoderOdometry(self.cfg)
        self.battery = BatterySim(self.cfg)
        self.grid = OccupancyGrid2D(self.cfg)
        self.nav = WaypointFollower(self.cfg)
        self.safety = SafetySupervisor(self.cfg)

        self.sim_time_ms = 0
        self.mode = DriveMode.IDLE
        self.manual_cmd = Twist2D(0.0, 0.0)
        self.avoidance_active = False
        self.last_left_effort = 0.0
        self.last_right_effort = 0.0

        # Ground truth pose (matches default start inside west SAR room)
        self.true_x = -2.2
        self.true_y = 0.0
        self.true_theta = 0.0
        self.imu = ImuState()
        self.last_tof = [2.0] * len(self.cfg.tof_angles_rad)

        self.reset()

    def reset(self, x_m: float = -2.2, y_m: float = 0.0, theta_rad: float = 0.0) -> None:
        self.sim_time_ms = 0
        self.mode = DriveMode.IDLE
        self.manual_cmd = Twist2D(0.0, 0.0)
        self.avoidance_active = False
        self.last_left_effort = 0.0
        self.last_right_effort = 0.0
        self.true_x = x_m
        self.true_y = y_m
        self.true_theta = wrap_angle(theta_rad)
        self.imu = ImuState(yaw_rad=self.true_theta)
        self.plant.reset()
        self.left_pid.reset()
        self.right_pid.reset()
        self.odom.reset(x_m, y_m, theta_rad)
        self.battery.reset(8.15)
        self.grid.reset()
        self.nav.clear_waypoint()
        self.safety.begin(self.sim_time_ms)
        for t in self.world.targets:
            t.detected = False
        self.last_tof = self.tof_sim.scan(self.true_x, self.true_y, self.true_theta, self.world)
        self.grid.update_from_scan(self.true_x, self.true_y, self.true_theta, self.last_tof)

    def feed_heartbeat(self) -> None:
        self.safety.feed_heartbeat(self.sim_time_ms)

    def set_mode(self, mode: DriveMode | str) -> None:
        target = DriveMode(mode) if isinstance(mode, str) else mode
        if self.mode == DriveMode.ESTOP and target != DriveMode.ESTOP:
            return
        self.mode = target
        self.feed_heartbeat()

    def set_manual_velocity(self, linear_mps: float, angular_rps: float) -> None:
        self.manual_cmd = Twist2D(linear_mps=linear_mps, angular_rps=angular_rps)
        if self.mode != DriveMode.ESTOP:
            self.mode = DriveMode.MANUAL
        self.feed_heartbeat()

    def set_waypoint(self, x_m: float, y_m: float) -> None:
        self.nav.set_waypoint(x_m, y_m)
        if self.mode != DriveMode.ESTOP:
            self.mode = DriveMode.AUTONOMOUS
        self.feed_heartbeat()

    def trigger_estop(self) -> None:
        self.safety.trigger_estop()
        self.mode = DriveMode.ESTOP
        self.manual_cmd = Twist2D(0.0, 0.0)

    def clear_estop(self) -> bool:
        if self.safety.clear_faults(self.sim_time_ms):
            self.mode = DriveMode.IDLE
            self.left_pid.reset()
            self.right_pid.reset()
            return True
        return False

    def inject_comm_loss(self) -> None:
        self.safety.inject_comm_loss()

    def restore_comm(self) -> None:
        self.safety.restore_comm(self.sim_time_ms)

    def step(self, dt_s: float = 0.02) -> dict:
        step_ms = max(1, int(round(dt_s * 1000.0)))
        self.sim_time_ms += step_ms

        # 1. Sensor scan from true pose
        self.last_tof = self.tof_sim.scan(self.true_x, self.true_y, self.true_theta, self.world)
        self.world.update_target_detections(self.true_x, self.true_y)

        # 2. Safety evaluation
        fault = self.safety.evaluate(self.sim_time_ms, self.mode, self.imu, self.battery.state)
        if fault != SafetyFault.NONE or self.mode == DriveMode.IDLE:
            self.left_pid.reset()
            self.right_pid.reset()
            self.last_left_effort = 0.0
            self.last_right_effort = 0.0
            self.avoidance_active = False
        else:
            if self.mode == DriveMode.AUTONOMOUS:
                decision = self.nav.compute_control(
                    self.odom.pose.x_m,
                    self.odom.pose.y_m,
                    self.odom.pose.theta_rad,
                    self.last_tof,
                )
                target_twist = decision.command
                self.avoidance_active = decision.obstacle_avoidance_active
            elif self.mode == DriveMode.MANUAL:
                target_twist = self.manual_cmd
                self.avoidance_active = False
            else:
                target_twist = Twist2D(0.0, 0.0)
                self.avoidance_active = False

            wheel_targets = self.kinematics.inverse(target_twist.linear_mps, target_twist.angular_rps)
            self.last_left_effort = self.left_pid.update(
                wheel_targets.left_rad_s, self.odom.pose.left_rad_s, dt_s
            )
            self.last_right_effort = self.right_pid.update(
                wheel_targets.right_rad_s, self.odom.pose.right_rad_s, dt_s
            )

        # 3. Wheel actuator & encoder tick integration
        dl, dr, true_wl, true_wr = self.plant.step(
            self.last_left_effort,
            self.last_right_effort,
            self.safety.motors_enabled,
            dt_s,
        )

        # 4. Ground-truth chassis kinematics with wall collision guard
        true_twist = self.kinematics.forward(true_wl, true_wr)
        d_theta = true_twist.angular_rps * dt_s
        ds = true_twist.linear_mps * dt_s
        next_theta = wrap_angle(self.true_theta + d_theta)
        theta_mid = self.true_theta + 0.5 * d_theta
        cand_x = self.true_x + ds * math.cos(theta_mid)
        cand_y = self.true_y + ds * math.sin(theta_mid)

        if not self.world.collides(cand_x, cand_y, self.cfg.collision_radius_m):
            self.true_x = cand_x
            self.true_y = cand_y
        else:
            # Slide/stop gently against wall without clipping through obstacle
            if not self.world.collides(cand_x, self.true_y, self.cfg.collision_radius_m):
                self.true_x = cand_x
            elif not self.world.collides(self.true_x, cand_y, self.cfg.collision_radius_m):
                self.true_y = cand_y

        self.true_theta = next_theta
        self.imu.yaw_rate_rps = true_twist.angular_rps
        self.imu.yaw_rad = self.true_theta

        # 5. Encoder odometry + occupancy map update
        self.odom.update(dl, dr, dt_s, imu_yaw_rate_rps=self.imu.yaw_rate_rps)
        self.grid.update_from_scan(
            self.odom.pose.x_m,
            self.odom.pose.y_m,
            self.odom.pose.theta_rad,
            self.last_tof,
        )
        self.battery.step(self.last_left_effort, self.last_right_effort, dt_s)

        return self.get_telemetry(include_grid=False)

    def get_telemetry(self, include_grid: bool = True) -> dict:
        left_rpm = self.kinematics.rad_s_to_rpm(self.odom.pose.left_rad_s)
        right_rpm = self.kinematics.rad_s_to_rpm(self.odom.pose.right_rad_s)
        payload = {
            "t_ms": self.sim_time_ms,
            "wall_ts": round(time.time(), 3),
            "mode": self.mode.value,
            "fault": self.safety.active_fault.value,
            "motors_enabled": self.safety.motors_enabled,
            "comm_loss_injected": self.safety.comm_loss_injected,
            "watchdog_ms": self.safety.watchdog.elapsed_ms(self.sim_time_ms),
            "watchdog_timeout_ms": self.cfg.watchdog_timeout_ms,
            "avoidance": self.avoidance_active,
            "pose": {
                "x": round(self.odom.pose.x_m, 4),
                "y": round(self.odom.pose.y_m, 4),
                "theta": round(self.odom.pose.theta_rad, 4),
                "v": round(self.odom.pose.linear_mps, 3),
                "w": round(self.odom.pose.angular_rps, 3),
            },
            "true_pose": {
                "x": round(self.true_x, 4),
                "y": round(self.true_y, 4),
                "theta": round(self.true_theta, 4),
            },
            "rpm": {
                "left": round(left_rpm, 1),
                "right": round(right_rpm, 1),
            },
            "tof_m": self.last_tof,
            "tof_angles_rad": list(self.cfg.tof_angles_rad),
            "imu": {
                "roll": round(self.imu.roll_deg, 1),
                "pitch": round(self.imu.pitch_deg, 1),
                "yaw": round(self.imu.yaw_rad, 3),
            },
            "battery": {
                "v": self.battery.state.voltage_v,
                "soc": self.battery.state.soc_percent,
                "low_warn": self.battery.state.low_warning,
            },
            "waypoint": (
                {"x": round(self.nav.waypoint[0], 3), "y": round(self.nav.waypoint[1], 3)}
                if self.nav.waypoint is not None
                else None
            ),
            "map_stats": self.grid.summary_stats(),
            "targets": [
                {"id": t.id, "x": t.x, "y": t.y, "label": t.label, "detected": t.detected}
                for t in self.world.targets
            ],
        }
        if include_grid:
            payload["grid"] = {
                "width": self.grid.width,
                "height": self.grid.height,
                "res": self.grid.res,
                "origin_x": self.grid.origin_x,
                "origin_y": self.grid.origin_y,
                "cells": self.grid.encode_cells_compact(),
            }
            payload["world"] = self.world.serialize()
        return payload
