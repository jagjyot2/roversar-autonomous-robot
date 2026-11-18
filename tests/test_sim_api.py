"""Tests for FastAPI endpoints, `/ws/rover` WebSocket, and CAD/KiCad/BenchTwin artifacts."""

from __future__ import annotations

import csv
import json
import struct
from pathlib import Path

from fastapi.testclient import TestClient

ROOT_DIR = Path(__file__).resolve().parent.parent


def test_api_health_and_dashboard_served(api_client: TestClient) -> None:
    health = api_client.get("/api/health")
    assert health.status_code == 200
    body = health.json()
    assert body["status"] == "ok"
    assert body["watchdog_timeout_ms"] == 500

    dash = api_client.get("/")
    assert dash.status_code == 200
    assert "RoverSAR" in dash.text


def test_api_command_waypoint_estop_and_camera(api_client: TestClient) -> None:
    res_cmd = api_client.post("/api/command", json={"linear": 0.35, "angular": 0.0})
    assert res_cmd.status_code == 200
    assert res_cmd.json()["mode"] == "MANUAL"

    res_wp = api_client.post("/api/waypoint", json={"x": -1.5, "y": 1.2})
    assert res_wp.status_code == 200
    assert res_wp.json()["mode"] == "AUTONOMOUS"

    res_estop = api_client.post("/api/estop")
    assert res_estop.status_code == 200
    assert res_estop.json()["fault"] == "ESTOP_SOFTWARE"

    res_clear = api_client.post("/api/clear-estop")
    assert res_clear.status_code == 200
    assert res_clear.json()["ok"] is True

    cam = api_client.get("/api/camera/frame.jpg")
    assert cam.status_code == 200
    assert cam.headers["content-type"].startswith("image/jpeg")
    assert cam.content[:2] == b"\xff\xd8"  # Valid JPEG SOI marker


def test_websocket_rover_telemetry_and_comm_loss(api_client: TestClient) -> None:
    with api_client.websocket_connect("/ws/rover") as ws:
        initial = ws.receive_json()
        assert "pose" in initial
        assert "tof_m" in initial
        assert len(initial["tof_m"]) == 5
        assert "grid" in initial

        ws.send_json({"type": "cmd_vel", "linear": 0.3, "angular": 0.2})
        ack = ws.receive_json()
        assert ack["mode"] == "MANUAL"

        ws.send_json({"type": "comm_loss", "active": True})
        ack_loss = ws.receive_json()
        assert ack_loss["comm_loss_injected"] is True

        ws.send_json({"type": "reset"})
        ack_reset = ws.receive_json()
        assert ack_reset["mode"] == "IDLE"


def _assert_valid_binary_stl(stl_path: Path, expected_xy_span: tuple[float, float] | None = None) -> None:
    data = stl_path.read_bytes()
    assert len(data) > 84, f"{stl_path} too small"
    tri_count = struct.unpack("<I", data[80:84])[0]
    assert tri_count > 0
    assert len(data) == 84 + 50 * tri_count

    xs: list[float] = []
    ys: list[float] = []
    for i in range(tri_count):
        offset = 84 + i * 50
        vals = struct.unpack("<12f", data[offset : offset + 48])
        nx, ny, nz = vals[0:3]
        assert abs((nx * nx + ny * ny + nz * nz) - 1.0) < 1e-3
        for v_idx in (3, 6, 9):
            xs.append(vals[v_idx])
            ys.append(vals[v_idx + 1])

    if expected_xy_span is not None:
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        assert abs(span_x - expected_xy_span[0]) < 1e-3
        assert abs(span_y - expected_xy_span[1]) < 1e-3


def test_cad_stl_files_are_valid() -> None:
    _assert_valid_binary_stl(ROOT_DIR / "cad" / "roversar_base_plate.stl", expected_xy_span=(240.0, 180.0))
    _assert_valid_binary_stl(ROOT_DIR / "cad" / "vl53l0x_sensor_mount.stl")
    _assert_valid_binary_stl(ROOT_DIR / "cad" / "esp32_cam_tilt_bracket.stl")
    _assert_valid_binary_stl(ROOT_DIR / "benchtwin" / "roversar_base_plate.stl", expected_xy_span=(240.0, 180.0))


def test_kicad_bom_and_benchtwin_files_are_valid() -> None:
    sch_text = (ROOT_DIR / "hardware" / "roversar.kicad_sch").read_text(encoding="utf-8")
    assert sch_text.startswith("(kicad_sch")
    assert sch_text.count("(") == sch_text.count(")")
    assert "ESP32-WROOM-32D" in sch_text
    assert "TB6612FNG" in sch_text

    net_text = (ROOT_DIR / "hardware" / "roversar.net").read_text(encoding="utf-8")
    assert net_text.startswith("(export")
    assert net_text.count("(") == net_text.count(")")
    assert "ESTOP_HW" in net_text
    assert "I2C_SDA" in net_text

    with (ROOT_DIR / "hardware" / "BOM.csv").open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    parts = {r["Ref"]: r["Part"] for r in rows}
    assert "U1" in parts and "U2" in parts and "U4-U8" in parts

    bt_manifest = json.loads((ROOT_DIR / "benchtwin" / "roversar-benchtwin.json").read_text(encoding="utf-8"))
    assert bt_manifest["schema"] == "benchtwin/project/v1"
    assert bt_manifest["envelope_mm"] == {"length": 240.0, "width": 180.0, "height": 95.0}

    btw = json.loads((ROOT_DIR / "benchtwin" / "roversar.btw").read_text(encoding="utf-8"))
    assert btw["format"] == "benchtwin-workbench"
