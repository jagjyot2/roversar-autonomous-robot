"""Tests for simulator-independent HAL, Wokwi configuration, scenarios, and BenchTwin preservation."""

from __future__ import annotations

import json
import re
import tomllib
from pathlib import Path

from simulation.adapters.common.rover_hal_interface import RoverHalProtocol
from simulation.adapters.wokwi.wokwi_rover_adapter import WokwiRoverAdapter

ROOT_DIR = Path(__file__).resolve().parents[1]


def _parse_roversar_config_pins() -> dict[str, int | list[int]]:
    header = (ROOT_DIR / "firmware" / "include" / "roversar_config.h").read_text(encoding="utf-8")
    pins: dict[str, int | list[int]] = {}
    for match in re.finditer(r"constexpr\s+uint8_t\s+(\w+)\s*=\s*(\d+)\s*;", header):
        pins[match.group(1)] = int(match.group(2))
    xshut_match = re.search(
        r"TOF_XSHUT_PINS\[TOF_COUNT\]\s*=\s*\{([^}]+)\}", header
    )
    assert xshut_match is not None, "TOF_XSHUT_PINS not found in roversar_config.h"
    pins["TOF_XSHUT_PINS"] = [int(x.strip()) for x in xshut_match.group(1).split(",")]
    return pins


def test_wokwi_diagram_pins_match_firmware_config() -> None:
    pins = _parse_roversar_config_pins()
    diagram_path = ROOT_DIR / "wokwi" / "diagram.json"
    diagram = json.loads(diagram_path.read_text(encoding="utf-8"))

    assert diagram["version"] == 1
    part_types = {p["id"]: p["type"] for p in diagram["parts"]}
    assert part_types["esp"] == "board-esp32-devkit-c-v4"
    assert part_types["imu"] == "wokwi-mpu6050"
    assert part_types["ultrasonic"] == "wokwi-hc-sr04"
    assert part_types["estop"] == "wokwi-pushbutton"
    assert part_types["vbat_pot"] == "wokwi-potentiometer"
    assert part_types["led_pwm_left"] == "wokwi-led"
    assert part_types["led_pwm_right"] == "wokwi-led"
    assert part_types["led_fault"] == "wokwi-led"
    assert part_types["quad_enc"] == "chip-quad-encoder"
    assert part_types["tof_array"] == "chip-vl53l0x-array"

    conn_map = {src: dst for src, dst, *_ in diagram["connections"]}
    assert conn_map["imu:SDA"] == f"esp:{pins['I2C_SDA']}"
    assert conn_map["imu:SCL"] == f"esp:{pins['I2C_SCL']}"
    assert conn_map["tof_array:SDA"] == f"esp:{pins['I2C_SDA']}"
    assert conn_map["tof_array:SCL"] == f"esp:{pins['I2C_SCL']}"

    assert conn_map["estop:1.l"] == f"esp:{pins['ESTOP_HW']}"
    assert conn_map["vbat_pot:SIG"] == f"esp:{pins['VBAT_ADC']}"
    assert conn_map["led_fault:A"] == f"esp:{pins['STATUS_LED']}"
    assert conn_map["led_pwm_left:A"] == f"esp:{pins['MOTOR_LEFT_PWM']}"
    assert conn_map["led_pwm_right:A"] == f"esp:{pins['MOTOR_RIGHT_PWM']}"

    assert conn_map["quad_enc:PWMA"] == f"esp:{pins['MOTOR_LEFT_PWM']}"
    assert conn_map["quad_enc:AIN1"] == f"esp:{pins['MOTOR_LEFT_IN1']}"
    assert conn_map["quad_enc:AIN2"] == f"esp:{pins['MOTOR_LEFT_IN2']}"
    assert conn_map["quad_enc:PWMB"] == f"esp:{pins['MOTOR_RIGHT_PWM']}"
    assert conn_map["quad_enc:BIN1"] == f"esp:{pins['MOTOR_RIGHT_IN1']}"
    assert conn_map["quad_enc:BIN2"] == f"esp:{pins['MOTOR_RIGHT_IN2']}"
    assert conn_map["quad_enc:STBY"] == f"esp:{pins['MOTOR_STBY']}"

    assert conn_map["quad_enc:ENC_LA"] == f"esp:{pins['ENC_LEFT_A']}"
    assert conn_map["quad_enc:ENC_LB"] == f"esp:{pins['ENC_LEFT_B']}"
    assert conn_map["quad_enc:ENC_RA"] == f"esp:{pins['ENC_RIGHT_A']}"
    assert conn_map["quad_enc:ENC_RB"] == f"esp:{pins['ENC_RIGHT_B']}"

    xshut_pins = pins["TOF_XSHUT_PINS"]
    assert isinstance(xshut_pins, list)
    for idx, gpio in enumerate(xshut_pins):
        assert conn_map[f"tof_array:XSHUT{idx}"] == f"esp:{gpio}"


def test_wokwi_toml_and_custom_chips_valid() -> None:
    wokwi_dir = ROOT_DIR / "wokwi"
    toml_data = tomllib.loads((wokwi_dir / "wokwi.toml").read_text(encoding="utf-8"))
    wokwi_cfg = toml_data["wokwi"]
    assert wokwi_cfg["version"] == 1
    assert wokwi_cfg["firmware"] == "../firmware/.pio/build/esp32dev/firmware.bin"
    assert wokwi_cfg["elf"] == "../firmware/.pio/build/esp32dev/firmware.elf"

    assert (wokwi_dir / wokwi_cfg["firmware"]).is_file()
    assert (wokwi_dir / wokwi_cfg["elf"]).is_file()

    for chip_base in ("chip-quad-encoder", "chip-vl53l0x-array"):
        chip_json = json.loads((wokwi_dir / f"{chip_base}.chip.json").read_text(encoding="utf-8"))
        assert len(chip_json["pins"]) >= 8
        chip_c = (wokwi_dir / f"{chip_base}.chip.c").read_text(encoding="utf-8")
        assert "chip_init" in chip_c


def test_hal_protocol_and_scenarios() -> None:
    adapter = WokwiRoverAdapter()
    assert isinstance(adapter, RoverHalProtocol)

    scenarios_dir = ROOT_DIR / "simulation" / "scenarios"

    comm_loss_res = adapter.run_scenario(scenarios_dir / "comm_loss_500ms_estop.json")
    assert comm_loss_res["motors_enabled"] is False
    assert comm_loss_res["fault"] == "COMM_LOSS_500MS"

    tilt_res = adapter.run_scenario(scenarios_dir / "imu_tilt_cutoff.json")
    assert tilt_res["motors_enabled"] is False
    assert tilt_res["fault"] == "TILT_ROLLOVER"

    nav_res = adapter.run_scenario(scenarios_dir / "autonomous_corridor_avoidance.json")
    assert nav_res["motors_enabled"] is True
    assert nav_res["fault"] == "NONE"
    assert nav_res["final_pose"]["x_m"] > -1.5

    adapter.init()
    adapter.set_hardware_estop(True)
    frame = adapter.step(0.02)
    assert frame.power_safety.estop_hw_tripped is True
    assert frame.motors.standby_active is False


def test_benchtwin_preserved_and_support_matrix_present() -> None:
    manifest_path = ROOT_DIR / "simulation" / "adapters" / "benchtwin" / "benchtwin_adapter_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "preserved_dormant"
    assert manifest["active_simulator"] == "wokwi"

    for rel_path in manifest["artifacts"].values():
        resolved = (manifest_path.parent / rel_path).resolve()
        assert resolved.is_file(), f"Missing preserved BenchTwin artifact: {resolved}"

    matrix_text = (ROOT_DIR / "simulation" / "documentation" / "COMPONENT-SUPPORT-MATRIX.md").read_text(
        encoding="utf-8"
    )
    assert "board-esp32-devkit-c-v4" in matrix_text
    assert "wokwi-mpu6050" in matrix_text
    assert "wokwi-hc-sr04" in matrix_text
    assert "chip-vl53l0x-array" in matrix_text
    assert "chip-quad-encoder" in matrix_text
    assert "No Silent Substitutions" in matrix_text
