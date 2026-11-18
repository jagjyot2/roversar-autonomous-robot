"""Generate watertight binary STL meshes matching RoverSAR OpenSCAD CAD assemblies."""

from __future__ import annotations

import math
import struct
from pathlib import Path

Triangle = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def _cross_normal(
    v0: tuple[float, float, float],
    v1: tuple[float, float, float],
    v2: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    if mag < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / mag, ny / mag, nz / mag)


def _add_box(tris: list[Triangle], cx: float, cy: float, cz: float, lx: float, ly: float, lz: float) -> None:
    hx, hy, hz = lx * 0.5, ly * 0.5, lz * 0.5
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    z0, z1 = cz - hz, cz + hz

    # 8 vertices
    p000 = (x0, y0, z0)
    p100 = (x1, y0, z0)
    p110 = (x1, y1, z0)
    p010 = (x0, y1, z0)
    p001 = (x0, y0, z1)
    p101 = (x1, y0, z1)
    p111 = (x1, y1, z1)
    p011 = (x0, y1, z1)

    quads = [
        (p000, p010, p110, p100),  # Bottom (-Z)
        (p001, p101, p111, p011),  # Top (+Z)
        (p000, p100, p101, p001),  # Front (-Y)
        (p110, p010, p011, p111),  # Back (+Y)
        (p010, p000, p001, p011),  # Left (-X)
        (p100, p110, p111, p101),  # Right (+X)
    ]
    for a, b, c, d in quads:
        tris.append((a, b, c))
        tris.append((a, c, d))


def _add_rotated_box(
    tris: list[Triangle],
    cx: float,
    cy: float,
    cz: float,
    lx: float,
    ly: float,
    lz: float,
    yaw_rad: float = 0.0,
    pitch_rad: float = 0.0,
) -> None:
    local: list[Triangle] = []
    _add_box(local, 0.0, 0.0, 0.0, lx, ly, lz)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cyw, syw = math.cos(yaw_rad), math.sin(yaw_rad)

    def xform(pt: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = pt
        # Pitch around Y
        xp = x * cp + z * sp
        zp = -x * sp + z * cp
        yp = y
        # Yaw around Z
        xr = xp * cyw - yp * syw
        yr = xp * syw + yp * cyw
        return (xr + cx, yr + cy, zp + cz)

    for a, b, c in local:
        tris.append((xform(a), xform(b), xform(c)))


def _add_prism_arc(
    tris: list[Triangle],
    r_inner: float,
    r_outer: float,
    z0: float,
    z1: float,
    ang_start: float,
    ang_end: float,
    segments: int = 24,
) -> None:
    for i in range(segments):
        a0 = ang_start + (ang_end - ang_start) * (i / segments)
        a1 = ang_start + (ang_end - ang_start) * ((i + 1) / segments)
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)

        i0b = (r_inner * c0, r_inner * s0, z0)
        i1b = (r_inner * c1, r_inner * s1, z0)
        o0b = (r_outer * c0, r_outer * s0, z0)
        o1b = (r_outer * c1, r_outer * s1, z0)

        i0t = (r_inner * c0, r_inner * s0, z1)
        i1t = (r_inner * c1, r_inner * s1, z1)
        o0t = (r_outer * c0, r_outer * s0, z1)
        o1t = (r_outer * c1, r_outer * s1, z1)

        # Top & Bottom
        tris.append((i0t, o0t, o1t))
        tris.append((i0t, o1t, i1t))
        tris.append((i0b, o1b, o0b))
        tris.append((i0b, i1b, o1b))
        # Outer & Inner
        tris.append((o0b, o0t, o1t))
        tris.append((o0b, o1t, o1b))
        tris.append((i0b, i1t, i0t))
        tris.append((i0b, i1b, i1t))

        if i == 0:
            tris.append((i0b, o0b, o0t))
            tris.append((i0b, o0t, i0t))
        if i == segments - 1:
            tris.append((i1b, o1t, o1b))
            tris.append((i1b, i1t, o1t))


def write_binary_stl(path: Path, header_label: str, triangles: list[Triangle]) -> None:
    header = header_label.encode("ascii", errors="replace")[:80].ljust(80, b" ")
    with path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for v0, v1, v2 in triangles:
            n = _cross_normal(v0, v1, v2)
            f.write(struct.pack("<12fH", *n, *v0, *v1, *v2, 0))


def build_base_plate() -> list[Triangle]:
    """240mm x 180mm chassis deck with dual motor cradles, battery rails, and caster boss."""
    tris: list[Triangle] = []
    # Main structural deck (240 x 180 x 4 mm)
    _add_box(tris, 0.0, 0.0, 2.0, 240.0, 180.0, 4.0)
    # Left & Right JGA25-370 motor cradles (at X = +18mm, Y = +/- 66mm)
    for sy in (-1.0, 1.0):
        _add_box(tris, 18.0, sy * 66.0, 11.0, 44.0, 26.0, 14.0)
    # 2S Battery pack retention rails (at X = -22mm, Y = +/- 28mm)
    for sy in (-1.0, 1.0):
        _add_box(tris, -22.0, sy * 28.0, 7.0, 76.0, 4.0, 6.0)
    # Rear caster stiffening pad (at X = -92mm)
    _add_box(tris, -92.0, 0.0, 5.5, 32.0, 36.0, 3.0)
    # Front ToF bracket mounting riser (at X = +102mm)
    _add_box(tris, 102.0, 0.0, 6.0, 18.0, 96.0, 4.0)
    return tris


def build_vl53l0x_mount() -> list[Triangle]:
    """5-channel VL53L0X curved front arc bracket (-60, -30, 0, +30, +60 deg)."""
    tris: list[Triangle] = []
    _add_prism_arc(
        tris,
        r_inner=56.0,
        r_outer=74.0,
        z0=0.0,
        z1=4.0,
        ang_start=math.radians(-75.0),
        ang_end=math.radians(75.0),
        segments=30,
    )
    for deg in (-60.0, -30.0, 0.0, 30.0, 60.0):
        ang = math.radians(deg)
        cx = 68.0 * math.cos(ang)
        cy = 68.0 * math.sin(ang)
        # Left & right vertical PCB retention pillars around optical window
        _add_rotated_box(tris, cx, cy, 14.0, 7.0, 27.0, 20.0, yaw_rad=ang)
    return tris


def build_esp32_cam_bracket() -> list[Triangle]:
    """ESP32-CAM 15-deg tilt bracket with dual upright arms and base flange."""
    tris: list[Triangle] = []
    # Base mounting flange (38 x 48 x 4 mm)
    _add_box(tris, 0.0, 0.0, 2.0, 38.0, 48.0, 4.0)
    # Left & right upright pivot towers
    for sy in (-1.0, 1.0):
        _add_box(tris, 0.0, sy * 21.0, 24.0, 24.0, 5.0, 40.0)
    # Tilted ESP32-CAM frame cradle (-15 deg downward pitch)
    _add_rotated_box(tris, 0.0, 0.0, 34.0, 8.0, 34.0, 44.0, pitch_rad=math.radians(-15.0))
    return tris


def generate_all(cad_dir: Path | None = None) -> dict[str, tuple[Path, int]]:
    out_dir = cad_dir or Path(__file__).resolve().parent
    parts = {
        "roversar_base_plate.stl": (
            "RoverSAR Base Plate 240x180x18mm PETG",
            build_base_plate(),
        ),
        "vl53l0x_sensor_mount.stl": (
            "RoverSAR 5-Ch VL53L0X ToF Sensor Arc Mount",
            build_vl53l0x_mount(),
        ),
        "esp32_cam_tilt_bracket.stl": (
            "RoverSAR ESP32-CAM 15-Deg Tilt Bracket",
            build_esp32_cam_bracket(),
        ),
    }
    results: dict[str, tuple[Path, int]] = {}
    for fname, (hdr, tris) in parts.items():
        fpath = out_dir / fname
        write_binary_stl(fpath, hdr, tris)
        results[fname] = (fpath, len(tris))
    return results


if __name__ == "__main__":
    summary = generate_all()
    for name, (p, count) in summary.items():
        print(f"Generated {p} ({count} triangles, {p.stat().st_size} bytes)")
