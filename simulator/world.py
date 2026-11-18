"""2D indoor search-and-rescue environment with walls, rubble obstacles, and raycasting."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class WallSegment:
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class BoxObstacle:
    x: float
    y: float
    width: float
    height: float
    label: str = "rubble"

    def to_segments(self) -> list[WallSegment]:
        x0, y0 = self.x, self.y
        x1, y1 = self.x + self.width, self.y + self.height
        return [
            WallSegment(x0, y0, x1, y0),
            WallSegment(x1, y0, x1, y1),
            WallSegment(x1, y1, x0, y1),
            WallSegment(x0, y1, x0, y0),
        ]


@dataclass
class RescueTarget:
    id: str
    x: float
    y: float
    label: str
    detected: bool = False


def _point_to_segment_dist(px: float, py: float, seg: WallSegment) -> float:
    vx = seg.x2 - seg.x1
    vy = seg.y2 - seg.y1
    wx = px - seg.x1
    wy = py - seg.y1
    len_sq = vx * vx + vy * vy
    if len_sq < 1e-12:
        return math.hypot(px - seg.x1, py - seg.y1)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / len_sq))
    proj_x = seg.x1 + t * vx
    proj_y = seg.y1 + t * vy
    return math.hypot(px - proj_x, py - proj_y)


def _ray_segment_intersect(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    seg: WallSegment,
    max_range: float,
) -> float | None:
    sx = seg.x2 - seg.x1
    sy = seg.y2 - seg.y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    qpx = seg.x1 - ox
    qpy = seg.y1 - oy
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * dy - qpy * dx) / denom
    if 0.0 <= t <= max_range and 0.0 <= u <= 1.0:
        return t
    return None


@dataclass
class RescueWorld:
    """Indoor SAR building floorplan ([-5, +5] m arena)."""

    bounds_min: float = -4.6
    bounds_max: float = 4.6
    obstacles: list[BoxObstacle] = field(default_factory=list)
    walls: list[WallSegment] = field(default_factory=list)
    targets: list[RescueTarget] = field(default_factory=list)

    @classmethod
    def default_sar_arena(cls) -> "RescueWorld":
        bmin, bmax = -4.5, 4.5
        outer_walls = [
            WallSegment(bmin, bmin, bmax, bmin),
            WallSegment(bmax, bmin, bmax, bmax),
            WallSegment(bmax, bmax, bmin, bmax),
            WallSegment(bmin, bmax, bmin, bmin),
            # Interior corridor partition with North & South doorways
            WallSegment(-0.5, -4.5, -0.5, -1.6),
            WallSegment(-0.5, -0.3, -0.5, 1.8),
            WallSegment(-0.5, 3.0, -0.5, 4.5),
            # East room wing wall
            WallSegment(1.6, 0.8, 4.5, 0.8),
        ]
        obstacles = [
            BoxObstacle(1.0, -0.55, 0.65, 1.10, "Collapsed Beam"),
            BoxObstacle(-2.8, 1.5, 0.90, 0.80, "Debris Pile A"),
            BoxObstacle(2.3, -2.9, 0.95, 0.95, "Cabinet Block"),
            BoxObstacle(-2.5, -3.2, 0.80, 0.70, "Rubble Crate"),
            BoxObstacle(2.0, 2.2, 0.85, 0.85, "Pillar"),
        ]
        all_segments = list(outer_walls)
        for obs in obstacles:
            all_segments.extend(obs.to_segments())

        targets = [
            RescueTarget("BEACON-01", 3.4, -0.8, "Thermal Signal Alpha"),
            RescueTarget("BEACON-02", -3.2, 3.2, "Audio Beacon Bravo"),
            RescueTarget("BEACON-03", 3.5, 3.4, "Survivor Marker Charlie"),
        ]
        return cls(bounds_min=bmin, bounds_max=bmax, obstacles=obstacles, walls=all_segments, targets=targets)

    def cast_ray(self, ox: float, oy: float, angle_rad: float, max_range: float) -> float:
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        best = max_range
        for seg in self.walls:
            hit = _ray_segment_intersect(ox, oy, dx, dy, seg, best)
            if hit is not None and hit < best:
                best = hit
        return best

    def collides(self, x: float, y: float, radius: float) -> bool:
        for seg in self.walls:
            if _point_to_segment_dist(x, y, seg) < radius:
                return True
        return False

    def update_target_detections(self, rx: float, ry: float, detect_radius: float = 1.25) -> None:
        for tgt in self.targets:
            if not tgt.detected and math.hypot(tgt.x - rx, tgt.y - ry) <= detect_radius:
                tgt.detected = True

    def serialize(self) -> dict:
        return {
            "bounds": [self.bounds_min, self.bounds_max],
            "walls": [[w.x1, w.y1, w.x2, w.y2] for w in self.walls],
            "obstacles": [
                {"x": o.x, "y": o.y, "w": o.width, "h": o.height, "label": o.label}
                for o in self.obstacles
            ],
            "targets": [
                {"id": t.id, "x": t.x, "y": t.y, "label": t.label, "detected": t.detected}
                for t in self.targets
            ],
        }
