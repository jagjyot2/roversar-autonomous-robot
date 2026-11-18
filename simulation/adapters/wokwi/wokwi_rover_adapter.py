"""Wokwi HAL adapter bridging Wokwi ESP32 simulation with the 50 Hz RoverSAR physics engine."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from simulation.adapters.common.rover_hal_interface import (
    DistanceArrayReading,
    EncoderReading,
    HalTelemetryFrame,
    ImuReading,
    MotorCommand,
    PowerAndSafetyReading,
)
from simulator.engine import RoverSimulationEngine
from simulator.safety import SafetyFault


class WokwiRoverAdapter:
    """Adapter implementing RoverHalProtocol for Wokwi + 50 Hz Python physics harness.

    Native Wokwi components (ESP32, MPU6050, HC-SR04, pushbutton E-STOP, potentiometer
    battery divider, PWM/fault LEDs) map directly to GPIO/I2C in ``wokwi/diagram.json``.
    Unsupported hardware (5-channel VL53L0X ToF array, TB6612FNG + DC geared motors +
    1440 CPR quadrature encoders, ESP32-CAM) is represented by custom Wokwi chip stubs
    and synchronized against ``RoverSimulationEngine`` at 50 Hz.
    """

    def __init__(self, engine: RoverSimulationEngine | None = None) -> None:
        self.engine = engine or RoverSimulationEngine(sensor_noise=False)
        self._last_motor_cmd = MotorCommand()
        self._hw_estop_tripped = False
        self._status_led = False
        self._left_ticks = 0
        self._right_ticks = 0

    def init(self) -> bool:
        self.engine.reset()
        self._last_motor_cmd = MotorCommand()
        self._hw_estop_tripped = False
        self._status_led = False
        self._left_ticks = 0
        self._right_ticks = 0
        return True

    def write_motors(self, cmd: MotorCommand) -> None:
        self._last_motor_cmd = cmd
        if not cmd.standby_active or self._hw_estop_tripped:
            self.engine.set_manual_velocity(0.0, 0.0)
            return

        left_dir = 1.0 if (cmd.left_in1 and not cmd.left_in2) else (-1.0 if (cmd.left_in2 and not cmd.left_in1) else 0.0)
        right_dir = 1.0 if (cmd.right_in1 and not cmd.right_in2) else (-1.0 if (cmd.right_in2 and not cmd.right_in1) else 0.0)

        wl = left_dir * (cmd.left_pwm / 1023.0) * self.engine.cfg.max_wheel_rad_s
        wr = right_dir * (cmd.right_pwm / 1023.0) * self.engine.cfg.max_wheel_rad_s
        twist = self.engine.kinematics.forward(wl, wr)
        self.engine.set_manual_velocity(twist.linear_mps, twist.angular_rps)

    def read_encoders(self) -> EncoderReading:
        return EncoderReading(left_ticks=self._left_ticks, right_ticks=self._right_ticks)

    def read_distances(self) -> DistanceArrayReading:
        tofs = tuple(round(float(d), 4) for d in self.engine.last_tof[:5])
        if len(tofs) < 5:
            tofs = tofs + (2.0,) * (5 - len(tofs))
        center_tof = tofs[2]
        return DistanceArrayReading(
            tof_distances_m=(tofs[0], tofs[1], tofs[2], tofs[3], tofs[4]),
            tof_valid_mask=0x1F,
            aux_ultrasonic_m=center_tof,
            aux_valid=True,
        )

    def read_imu(self) -> ImuReading:
        imu = self.engine.imu
        return ImuReading(
            accel_mps2=(0.0, 0.0, 9.80665),
            gyro_rps=(0.0, 0.0, float(imu.yaw_rate_rps)),
            roll_deg=float(imu.roll_deg),
            pitch_deg=float(imu.pitch_deg),
            yaw_deg=float(math.degrees(imu.yaw_rad)),
        )

    def read_power_safety(self) -> PowerAndSafetyReading:
        vbat = float(self.engine.battery.state.voltage_v)
        # 22k / 10k divider -> V_adc = vbat * (10 / 32), 12-bit ADC @ 3.3V
        v_adc = vbat * (10000.0 / (22000.0 + 10000.0))
        adc_raw = int(max(0, min(4095, round((v_adc / 3.3) * 4095.0))))
        fault = self.engine.safety.active_fault
        return PowerAndSafetyReading(
            vbat_adc_raw=adc_raw,
            vbat_volts=round(vbat, 3),
            estop_hw_tripped=self._hw_estop_tripped,
            estop_sw_tripped=(fault == SafetyFault.ESTOP_SOFTWARE),
        )

    def set_status_led(self, on: bool) -> None:
        self._status_led = bool(on)

    def set_hardware_estop(self, tripped: bool) -> None:
        self._hw_estop_tripped = bool(tripped)
        if tripped:
            self.engine.trigger_estop()

    def inject_comm_loss(self) -> None:
        self.engine.inject_comm_loss()

    def inject_imu_tilt(self, roll_deg: float = 0.0, pitch_deg: float = 0.0) -> None:
        self.engine.imu.roll_deg = roll_deg
        self.engine.imu.pitch_deg = pitch_deg

    def step(self, dt_s: float = 0.02) -> HalTelemetryFrame:
        prev_l = self.engine.odom.pose.left_rad_s
        prev_r = self.engine.odom.pose.right_rad_s
        self.engine.step(dt_s)
        ticks_per_rad = self.engine.cfg.encoder_ticks_per_rev / (2.0 * math.pi)
        avg_wl = 0.5 * (prev_l + self.engine.odom.pose.left_rad_s)
        avg_wr = 0.5 * (prev_r + self.engine.odom.pose.right_rad_s)
        self._left_ticks += int(round(avg_wl * dt_s * ticks_per_rad))
        self._right_ticks += int(round(avg_wr * dt_s * ticks_per_rad))

        fault = self.engine.safety.active_fault
        self._status_led = fault != SafetyFault.NONE
        motors = (
            MotorCommand(standby_active=False)
            if not self.engine.safety.motors_enabled
            else self._last_motor_cmd
        )

        return HalTelemetryFrame(
            timestamp_ms=self.engine.sim_time_ms,
            motors=motors,
            encoders=self.read_encoders(),
            distances=self.read_distances(),
            imu=self.read_imu(),
            power_safety=self.read_power_safety(),
            status_led=self._status_led,
        )

    def run_scenario(self, scenario_path: Path) -> dict[str, Any]:
        """Execute a JSON fault/navigation scenario and return summary metrics."""
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        self.init()
        start = scenario.get("initial_pose", {})
        self.engine.reset(
            x_m=float(start.get("x_m", -2.2)),
            y_m=float(start.get("y_m", 0.0)),
            theta_rad=float(start.get("theta_rad", 0.0)),
        )

        events = sorted(scenario.get("events", []), key=lambda e: int(e["at_ms"]))
        event_idx = 0
        dt_s = float(scenario.get("dt_s", 0.02))
        duration_ms = int(scenario.get("duration_ms", 2000))
        keep_heartbeat = bool(scenario.get("keep_heartbeat", True))

        while self.engine.sim_time_ms < duration_ms:
            while event_idx < len(events) and int(events[event_idx]["at_ms"]) <= self.engine.sim_time_ms:
                ev = events[event_idx]
                action = ev["action"]
                if action == "set_waypoint":
                    self.engine.set_waypoint(float(ev["x_m"]), float(ev["y_m"]))
                elif action == "set_manual":
                    self.engine.set_manual_velocity(float(ev["linear_mps"]), float(ev["angular_rps"]))
                elif action == "inject_comm_loss":
                    keep_heartbeat = False
                    self.inject_comm_loss()
                elif action == "trigger_hw_estop":
                    self.set_hardware_estop(True)
                elif action == "inject_tilt":
                    self.inject_imu_tilt(float(ev.get("roll_deg", 0.0)), float(ev.get("pitch_deg", 0.0)))
                event_idx += 1

            if keep_heartbeat:
                self.engine.feed_heartbeat()
            frame = self.step(dt_s)

        return {
            "scenario_id": scenario.get("scenario_id", scenario_path.stem),
            "final_time_ms": self.engine.sim_time_ms,
            "final_pose": {
                "x_m": round(self.engine.true_x, 4),
                "y_m": round(self.engine.true_y, 4),
                "theta_rad": round(self.engine.true_theta, 4),
            },
            "motors_enabled": self.engine.safety.motors_enabled,
            "fault": self.engine.safety.active_fault.value,
            "mode": self.engine.mode.value,
            "last_frame": frame,
        }
