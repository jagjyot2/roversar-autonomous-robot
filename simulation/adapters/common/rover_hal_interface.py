"""Simulator-independent canonical Hardware Abstraction Layer (HAL) for RoverSAR."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class MotorCommand:
    """Differential-drive motor driver command (TB6612FNG / L298N)."""

    left_pwm: int = 0  # 0..1023 (10-bit LEDC)
    left_in1: bool = False
    left_in2: bool = False
    right_pwm: int = 0  # 0..1023 (10-bit LEDC)
    right_in1: bool = False
    right_in2: bool = False
    standby_active: bool = False  # True = STBY HIGH (enabled)


@dataclass(frozen=True)
class EncoderReading:
    """Quadrature encoder counts (1440 CPR)."""

    left_ticks: int = 0
    right_ticks: int = 0


@dataclass(frozen=True)
class DistanceArrayReading:
    """5-channel VL53L0X ToF distances + auxiliary HC-SR04 ultrasonic."""

    tof_distances_m: tuple[float, float, float, float, float] = (2.0, 2.0, 2.0, 2.0, 2.0)
    tof_valid_mask: int = 0x1F
    aux_ultrasonic_m: float = 2.0
    aux_valid: bool = True


@dataclass(frozen=True)
class ImuReading:
    """MPU6050 6-DOF IMU + fused orientation angles."""

    accel_mps2: tuple[float, float, float] = (0.0, 0.0, 9.80665)
    gyro_rps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class PowerAndSafetyReading:
    """2S Li-Ion battery divider ADC and E-STOP interlock status."""

    vbat_adc_raw: int = 3100
    vbat_volts: float = 7.40
    estop_hw_tripped: bool = False
    estop_sw_tripped: bool = False


@dataclass
class HalTelemetryFrame:
    """Aggregated HAL snapshot exchanged between simulator backends and scenarios."""

    timestamp_ms: int = 0
    motors: MotorCommand = field(default_factory=MotorCommand)
    encoders: EncoderReading = field(default_factory=EncoderReading)
    distances: DistanceArrayReading = field(default_factory=DistanceArrayReading)
    imu: ImuReading = field(default_factory=ImuReading)
    power_safety: PowerAndSafetyReading = field(default_factory=PowerAndSafetyReading)
    status_led: bool = False


@runtime_checkable
class RoverHalProtocol(Protocol):
    """Protocol implemented by Wokwi, Python 50 Hz, and BenchTwin HAL adapters."""

    def init(self) -> bool:
        """Initialize the underlying simulator or hardware transport."""
        ...

    def write_motors(self, cmd: MotorCommand) -> None:
        """Apply differential-drive PWM, direction, and STBY state."""
        ...

    def read_encoders(self) -> EncoderReading:
        """Read cumulative left/right quadrature encoder ticks."""
        ...

    def read_distances(self) -> DistanceArrayReading:
        """Read 5-channel ToF array and auxiliary HC-SR04 distance."""
        ...

    def read_imu(self) -> ImuReading:
        """Read MPU6050 accelerometer, gyroscope, and fused Euler angles."""
        ...

    def read_power_safety(self) -> PowerAndSafetyReading:
        """Read battery voltage divider ADC and E-STOP state."""
        ...

    def set_status_led(self, on: bool) -> None:
        """Set fault/status LED state."""
        ...

    def step(self, dt_s: float) -> HalTelemetryFrame:
        """Advance simulation by dt_s and return the updated HAL snapshot."""
        ...
