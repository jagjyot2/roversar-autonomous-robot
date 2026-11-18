"""2D Log-Odds Bayesian Occupancy Grid SLAM mapper using Bresenham raycasting."""

from __future__ import annotations

import math

import numpy as np

from simulator.config import RoverConfig


class OccupancyGrid2D:
    """2D probabilistic occupancy grid updated incrementally from ToF laser scans."""

    LOG_ODDS_FREE = -0.45
    LOG_ODDS_HIT = 1.15
    LOG_ODDS_MIN = -5.0
    LOG_ODDS_MAX = 5.0

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self.width = self.cfg.grid_width
        self.height = self.cfg.grid_height
        self.res = self.cfg.grid_resolution_m
        self.origin_x = self.cfg.grid_origin_x_m
        self.origin_y = self.cfg.grid_origin_y_m
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def reset(self) -> None:
        self.log_odds.fill(0.0)

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        gx = int(math.floor((x_m - self.origin_x) / self.res))
        gy = int(math.floor((y_m - self.origin_y) / self.res))
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return gx, gy
        return None

    def update_from_scan(
        self,
        pose_x: float,
        pose_y: float,
        pose_theta: float,
        ranges_m: list[float],
    ) -> None:
        sensor_x = pose_x + self.cfg.tof_mount_offset_x_m * math.cos(pose_theta)
        sensor_y = pose_y + self.cfg.tof_mount_offset_x_m * math.sin(pose_theta)
        start_cell = self.world_to_grid(sensor_x, sensor_y)
        if start_cell is None:
            return
        sx, sy = start_cell

        for rel_angle, r in zip(self.cfg.tof_angles_rad, ranges_m):
            hit = r < (self.cfg.tof_max_range_m - 0.05)
            beam_angle = pose_theta + rel_angle
            end_x = sensor_x + r * math.cos(beam_angle)
            end_y = sensor_y + r * math.sin(beam_angle)
            end_cell = self.world_to_grid(end_x, end_y)
            if end_cell is None:
                continue
            self._trace_ray(sx, sy, end_cell[0], end_cell[1], hit)

    def _trace_ray(self, x0: int, y0: int, x1: int, y1: int, hit: bool) -> None:
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cx, cy = x0, y0
        while True:
            if cx == x1 and cy == y1:
                delta = self.LOG_ODDS_HIT if hit else self.LOG_ODDS_FREE
                self._update_cell(cx, cy, delta)
                break
            self._update_cell(cx, cy, self.LOG_ODDS_FREE)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def _update_cell(self, gx: int, gy: int, delta: float) -> None:
        val = float(self.log_odds[gy, gx]) + delta
        self.log_odds[gy, gx] = max(self.LOG_ODDS_MIN, min(self.LOG_ODDS_MAX, val))

    def cell_probability(self, gx: int, gy: int) -> float:
        l_val = float(self.log_odds[gy, gx])
        return 1.0 / (1.0 + math.exp(-l_val))

    def probability_grid(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def summary_stats(self) -> dict[str, float | int]:
        explored = int(np.count_nonzero(np.abs(self.log_odds) > 0.25))
        occupied = int(np.count_nonzero(self.log_odds > 0.7))
        free = int(np.count_nonzero(self.log_odds < -0.35))
        total = self.width * self.height
        return {
            "explored_cells": explored,
            "occupied_cells": occupied,
            "free_cells": free,
            "explored_pct": round(100.0 * explored / total, 1),
        }

    def encode_cells_compact(self) -> list[int]:
        """Encode grid into compact ternary states: -1=unknown, 0=free, 1=occupied."""
        out = np.full((self.height, self.width), -1, dtype=np.int8)
        out[self.log_odds < -0.35] = 0
        out[self.log_odds > 0.65] = 1
        return out.flatten().tolist()
