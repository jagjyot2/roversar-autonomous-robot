"""VFH+ reactive local obstacle avoidance and waypoint pursuit planner."""

from __future__ import annotations

import math
from dataclasses import dataclass

from simulator.config import RoverConfig
from simulator.kinematics import Twist2D, clamp, wrap_angle


@dataclass
class AvoidanceDecision:
    command: Twist2D
    obstacle_avoidance_active: bool = False
    chosen_steer_rad: float = 0.0


class VfhPlanner:
    """Vector Field Histogram local obstacle avoidance over 5 VL53L0X ToF beams."""

    NUM_SECTORS = 36
    OBSTACLE_THRESHOLD_M = 0.85
    EMERGENCY_REVERSE_M = 0.26

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self._sector_step = (2.0 * math.pi) / self.NUM_SECTORS
        self._evade_turn_bias = 0.0

    def _angle_to_sector(self, angle_rad: float) -> int:
        wrapped = wrap_angle(angle_rad) + math.pi
        return int(math.floor(wrapped / self._sector_step)) % self.NUM_SECTORS

    def _sector_to_angle(self, sector: int) -> float:
        return -math.pi + (sector + 0.5) * self._sector_step

    def compute_safe_command(
        self,
        ranges_m: list[float],
        desired_heading_err_rad: float,
        desired_speed_mps: float,
    ) -> AvoidanceDecision:
        min_front = min(ranges_m[1:4])  # -30, 0, +30 deg forward corridor
        left_clear = ranges_m[0] + ranges_m[1]
        right_clear = ranges_m[3] + ranges_m[4]

        # Emergency proximity guard: back out slightly and spin toward clearer side.
        if min_front < self.EMERGENCY_REVERSE_M:
            if self._evade_turn_bias == 0.0:
                self._evade_turn_bias = 1.0 if right_clear >= left_clear else -1.0
            return AvoidanceDecision(
                command=Twist2D(linear_mps=-0.06, angular_rps=self._evade_turn_bias * 2.4),
                obstacle_avoidance_active=True,
                chosen_steer_rad=self._evade_turn_bias * 1.57,
            )

        histogram = [0.0] * self.NUM_SECTORS
        any_close = False
        for rel_ang, dist in zip(self.cfg.tof_angles_rad, ranges_m):
            if dist >= self.OBSTACLE_THRESHOLD_M:
                continue
            any_close = True
            mag = (self.OBSTACLE_THRESHOLD_M - dist) / self.OBSTACLE_THRESHOLD_M
            center_sec = self._angle_to_sector(rel_ang)
            for offset in range(-4, 5):
                sec = (center_sec + offset) % self.NUM_SECTORS
                decay = max(0.1, 1.0 - abs(offset) * 0.18)
                histogram[sec] = max(histogram[sec], mag * decay)

        target_err = wrap_angle(desired_heading_err_rad)
        if not any_close:
            self._evade_turn_bias = 0.0
            angular = clamp(2.6 * target_err, -self.cfg.max_angular_rps, self.cfg.max_angular_rps)
            turn_penalty = max(0.18, 1.0 - abs(target_err) / math.pi)
            linear = clamp(desired_speed_mps * turn_penalty, 0.0, self.cfg.max_linear_mps)
            return AvoidanceDecision(
                command=Twist2D(linear_mps=linear, angular_rps=angular),
                obstacle_avoidance_active=False,
                chosen_steer_rad=target_err,
            )

        # Select valley sector balancing obstacle clearance vs goal heading alignment
        best_cost = 1e9
        best_angle = target_err
        for s in range(self.NUM_SECTORS):
            ang = self._sector_to_angle(s)
            if abs(ang) > 1.95:  # ~112 deg forward hemisphere
                continue
            side_bias = 0.0
            if self._evade_turn_bias != 0.0 and ang * self._evade_turn_bias > 0:
                side_bias = -0.25  # Prevent left/right oscillation near walls
            cost = (5.0 * histogram[s]) + 0.9 * abs(wrap_angle(ang - target_err)) + side_bias
            if cost < best_cost:
                best_cost = cost
                best_angle = ang

        if abs(best_angle) > 0.35:
            self._evade_turn_bias = 1.0 if best_angle > 0 else -1.0

        speed_scale = clamp((min_front - self.EMERGENCY_REVERSE_M) / 0.55, 0.22, 1.0)
        linear = clamp(desired_speed_mps * speed_scale, 0.0, self.cfg.max_linear_mps)
        angular = clamp(2.9 * best_angle, -self.cfg.max_angular_rps, self.cfg.max_angular_rps)
        return AvoidanceDecision(
            command=Twist2D(linear_mps=linear, angular_rps=angular),
            obstacle_avoidance_active=True,
            chosen_steer_rad=best_angle,
        )


class WaypointFollower:
    """Goal-seeking waypoint follower backed by VFH+ obstacle avoidance."""

    GOAL_TOLERANCE_M = 0.16

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self.planner = VfhPlanner(self.cfg)
        self.waypoint: tuple[float, float] | None = None
        self.reached_count = 0

    def set_waypoint(self, x_m: float, y_m: float) -> None:
        self.waypoint = (float(x_m), float(y_m))

    def clear_waypoint(self) -> None:
        self.waypoint = None

    def compute_control(
        self,
        pose_x: float,
        pose_y: float,
        pose_theta: float,
        ranges_m: list[float],
        cruise_speed_mps: float = 0.38,
    ) -> AvoidanceDecision:
        if self.waypoint is None:
            return AvoidanceDecision(command=Twist2D(0.0, 0.0), obstacle_avoidance_active=False)

        wx, wy = self.waypoint
        dx = wx - pose_x
        dy = wy - pose_y
        dist = math.hypot(dx, dy)
        if dist <= self.GOAL_TOLERANCE_M:
            self.waypoint = None
            self.reached_count += 1
            return AvoidanceDecision(command=Twist2D(0.0, 0.0), obstacle_avoidance_active=False)

        desired_theta = math.atan2(dy, dx)
        heading_err = wrap_angle(desired_theta - pose_theta)
        speed = min(cruise_speed_mps, max(0.18, dist * 0.85))
        return self.planner.compute_safe_command(ranges_m, heading_err, speed)
