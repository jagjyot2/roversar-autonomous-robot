"""Synthetic ESP32-CAM first-person SAR raycasted camera frame generator."""

from __future__ import annotations

import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

from simulator.engine import RoverSimulationEngine


def render_fpv_frame_jpeg(engine: RoverSimulationEngine, width: int = 320, height: int = 180) -> bytes:
    """Render a raycasted 3D-perspective indoor corridor view + HUD overlay as JPEG."""
    if cv2 is None:
        return b""

    frame = np.zeros((height, width, 3), dtype=np.uint8)
    # Dark ceiling and concrete floor gradient
    half_h = height // 2
    for y in range(half_h):
        c = int(12 + (y / half_h) * 14)
        frame[y, :] = (c + 6, c + 2, c)
    for y in range(half_h, height):
        ratio = (y - half_h) / max(1, height - half_h)
        c = int(18 + ratio * 28)
        frame[y, :] = (c, c + 4, c + 2)

    # Raycast columns across 64 deg horizontal FOV
    fov = math.radians(64.0)
    rx, ry, rtheta = engine.true_x, engine.true_y, engine.true_theta
    num_cols = 80
    col_w = max(1, width // num_cols)

    for i in range(num_cols):
        rel = ((i + 0.5) / num_cols - 0.5) * fov
        # Left of image corresponds to +fov/2 in standard CCW robot yaw
        ray_angle = rtheta - rel
        dist = engine.world.cast_ray(rx, ry, ray_angle, 6.0)
        perp_dist = max(0.18, dist * math.cos(rel))
        wall_h = min(height - 10, int((0.55 * height) / perp_dist))
        y0 = max(0, half_h - wall_h // 2)
        y1 = min(height - 1, half_h + wall_h // 2)
        shade = max(35, min(215, int(225 / (1.0 + 0.55 * perp_dist))))
        x0 = i * col_w
        x1 = min(width, x0 + col_w)
        # Cyan-slate search-and-rescue industrial wall tint (BGR)
        frame[y0:y1, x0:x1] = (int(shade * 0.85), int(shade * 0.72), int(shade * 0.45))

    # Project visible rescue targets into camera frame
    for tgt in engine.world.targets:
        dx = tgt.x - rx
        dy = tgt.y - ry
        dist = math.hypot(dx, dy)
        ang_diff = (math.atan2(dy, dx) - rtheta + math.pi) % (2 * math.pi) - math.pi
        if abs(ang_diff) < fov * 0.55 and dist < 5.0:
            occluded = engine.world.cast_ray(rx, ry, rtheta + ang_diff, dist) < (dist - 0.15)
            if not occluded:
                px = int((0.5 - ang_diff / fov) * width)
                py = half_h + int(min(28, 18 / max(0.5, dist)))
                rad = max(4, min(18, int(20 / max(0.5, dist))))
                cv2.circle(frame, (px, py), rad, (0, 145, 255), -1)
                cv2.circle(frame, (px, py), rad + 3, (40, 230, 255), 1)
                cv2.putText(
                    frame,
                    tgt.id,
                    (max(4, px - 26), max(14, py - rad - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (70, 240, 255),
                    1,
                    cv2.LINE_AA,
                )

    # Tactical camera HUD reticle & telemetry overlay
    cx, cy = width // 2, half_h
    hud_col = (212, 238, 70) if engine.safety.active_fault.value == "NONE" else (60, 75, 255)
    cv2.line(frame, (cx - 14, cy), (cx - 5, cy), hud_col, 1)
    cv2.line(frame, (cx + 5, cy), (cx + 14, cy), hud_col, 1)
    cv2.line(frame, (cx, cy - 10), (cx, cy - 3), hud_col, 1)
    cv2.line(frame, (cx, cy + 3), (cx, cy + 10), hud_col, 1)

    front_dist = engine.last_tof[2] if len(engine.last_tof) >= 3 else 2.0
    cv2.putText(
        frame,
        f"ESP32-CAM | {engine.mode.value} | TOF:{front_dist:.2f}m",
        (8, 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        hud_col,
        1,
        cv2.LINE_AA,
    )
    if engine.safety.active_fault.value != "NONE":
        cv2.putText(
            frame,
            f"HALT: {engine.safety.active_fault.value}",
            (8, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (50, 60, 255),
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            frame,
            f"X:{engine.true_x:+.2f} Y:{engine.true_y:+.2f} HDG:{math.degrees(engine.true_theta):+.0f}deg",
            (8, height - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (180, 220, 200),
            1,
            cv2.LINE_AA,
        )

    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    return bytes(buf) if ok else b""
