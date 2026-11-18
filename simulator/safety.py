"""500ms comm-loss watchdog, rollover interlock, low-voltage, and E-STOP supervisor."""

from __future__ import annotations

from enum import Enum

from simulator.config import RoverConfig
from simulator.sensors import BatteryState, ImuState


class DriveMode(str, Enum):
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    ESTOP = "ESTOP"


class SafetyFault(str, Enum):
    NONE = "NONE"
    COMM_LOSS_500MS = "COMM_LOSS_500MS"
    ESTOP_SOFTWARE = "ESTOP_SOFTWARE"
    ESTOP_HARDWARE = "ESTOP_HARDWARE"
    TILT_ROLLOVER = "TILT_ROLLOVER"
    LOW_BATTERY = "LOW_BATTERY"


class CommWatchdog:
    """Deterministic millisecond-resolution communication heartbeat watchdog."""

    def __init__(self, timeout_ms: int = 500) -> None:
        self.timeout_ms = timeout_ms
        self.last_feed_ms = 0
        self.armed = False

    def reset(self, now_ms: int) -> None:
        self.last_feed_ms = now_ms
        self.armed = True

    def feed(self, now_ms: int) -> None:
        self.last_feed_ms = now_ms
        self.armed = True

    def elapsed_ms(self, now_ms: int) -> int:
        if not self.armed:
            return self.timeout_ms
        return max(0, now_ms - self.last_feed_ms)

    def is_expired(self, now_ms: int) -> bool:
        if not self.armed:
            return True
        return (now_ms - self.last_feed_ms) >= self.timeout_ms


class SafetySupervisor:
    """Enforces mandatory safety interlocks and immediate motor shutoff."""

    def __init__(self, config: RoverConfig | None = None) -> None:
        self.cfg = config or RoverConfig()
        self.watchdog = CommWatchdog(timeout_ms=self.cfg.watchdog_timeout_ms)
        self.active_fault = SafetyFault.NONE
        self.sw_estop_latched = False
        self.hw_estop_tripped = False
        self.comm_loss_injected = False
        self.motors_enabled = False
        self._low_batt_start_ms: int | None = None

    def begin(self, now_ms: int) -> None:
        self.watchdog.reset(now_ms)
        self.active_fault = SafetyFault.NONE
        self.sw_estop_latched = False
        self.hw_estop_tripped = False
        self.comm_loss_injected = False
        self.motors_enabled = False
        self._low_batt_start_ms = None

    def feed_heartbeat(self, now_ms: int) -> None:
        if self.comm_loss_injected:
            return
        self.watchdog.feed(now_ms)
        if self.active_fault == SafetyFault.COMM_LOSS_500MS:
            self.active_fault = SafetyFault.NONE

    def inject_comm_loss(self) -> None:
        self.comm_loss_injected = True

    def restore_comm(self, now_ms: int) -> None:
        self.comm_loss_injected = False
        self.watchdog.feed(now_ms)
        if self.active_fault == SafetyFault.COMM_LOSS_500MS:
            self.active_fault = SafetyFault.NONE

    def trigger_estop(self) -> None:
        self.sw_estop_latched = True
        self.active_fault = SafetyFault.ESTOP_SOFTWARE
        self.motors_enabled = False

    def clear_faults(self, now_ms: int) -> bool:
        if self.hw_estop_tripped:
            self.active_fault = SafetyFault.ESTOP_HARDWARE
            return False
        self.sw_estop_latched = False
        self.comm_loss_injected = False
        self.active_fault = SafetyFault.NONE
        self.watchdog.feed(now_ms)
        return True

    def evaluate(
        self,
        now_ms: int,
        requested_mode: DriveMode,
        imu: ImuState,
        battery: BatteryState,
    ) -> SafetyFault:
        if self.hw_estop_tripped:
            self.active_fault = SafetyFault.ESTOP_HARDWARE
            self.motors_enabled = False
            return self.active_fault

        if self.sw_estop_latched or requested_mode == DriveMode.ESTOP:
            self.sw_estop_latched = True
            self.active_fault = SafetyFault.ESTOP_SOFTWARE
            self.motors_enabled = False
            return self.active_fault

        if abs(imu.roll_deg) > self.cfg.max_safe_tilt_deg or abs(imu.pitch_deg) > self.cfg.max_safe_tilt_deg:
            self.active_fault = SafetyFault.TILT_ROLLOVER
            self.motors_enabled = False
            return self.active_fault

        if battery.critical_cutoff:
            if self._low_batt_start_ms is None:
                self._low_batt_start_ms = now_ms
            elif (now_ms - self._low_batt_start_ms) >= 1000:
                self.active_fault = SafetyFault.LOW_BATTERY
                self.motors_enabled = False
                return self.active_fault
        else:
            self._low_batt_start_ms = None

        if requested_mode in (DriveMode.MANUAL, DriveMode.AUTONOMOUS) and self.watchdog.is_expired(now_ms):
            self.active_fault = SafetyFault.COMM_LOSS_500MS
            self.motors_enabled = False
            return self.active_fault

        if self.active_fault not in (SafetyFault.ESTOP_SOFTWARE, SafetyFault.ESTOP_HARDWARE):
            self.active_fault = SafetyFault.NONE

        self.motors_enabled = (self.active_fault == SafetyFault.NONE) and (requested_mode != DriveMode.IDLE)
        return self.active_fault
