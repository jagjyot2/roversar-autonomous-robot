#include "safety/safety_supervisor.h"

#include <Arduino.h>
#include <cmath>

namespace roversar::safety {

void SafetySupervisor::begin(uint32_t now_ms) {
    pinMode(pins::ESTOP_HW, INPUT_PULLUP);
    watchdog_.reset(now_ms);
    active_fault_ = SafetyFault::NONE;
    sw_estop_latched_ = false;
}

void SafetySupervisor::feedHeartbeat(uint32_t now_ms) {
    watchdog_.feed(now_ms);
    if (active_fault_ == SafetyFault::COMM_LOSS) {
        active_fault_ = SafetyFault::NONE;
    }
}

void SafetySupervisor::triggerSoftwareEstop() {
    sw_estop_latched_ = true;
    active_fault_ = SafetyFault::ESTOP_SW;
}

bool SafetySupervisor::clearFaults(uint32_t now_ms) {
    if (digitalRead(pins::ESTOP_HW) == HIGH) {
        active_fault_ = SafetyFault::ESTOP_HW;
        return false;
    }
    sw_estop_latched_ = false;
    active_fault_ = SafetyFault::NONE;
    watchdog_.feed(now_ms);
    return true;
}

SafetyFault SafetySupervisor::evaluate(uint32_t now_ms,
                                       DriveMode requested_mode,
                                       const sensors::ImuState& imu,
                                       const sensors::BatteryStatus& battery,
                                       motor_control::MotorDriver& motors) {
    // 1. Hardware NC E-STOP pin check (HIGH = switch opened or cable cut).
    if (digitalRead(pins::ESTOP_HW) == HIGH) {
        active_fault_ = SafetyFault::ESTOP_HW;
        motors.enable(false);
        return active_fault_;
    }

    // 2. Latched software E-STOP.
    if (sw_estop_latched_ || requested_mode == DriveMode::ESTOP) {
        sw_estop_latched_ = true;
        active_fault_ = SafetyFault::ESTOP_SW;
        motors.enable(false);
        return active_fault_;
    }

    // 3. IMU rollover / extreme tilt cutoff.
    if (imu.healthy &&
        (std::fabs(imu.roll_deg) > MAX_SAFE_TILT_DEG ||
         std::fabs(imu.pitch_deg) > MAX_SAFE_TILT_DEG)) {
        active_fault_ = SafetyFault::TILT_ROLLOVER;
        motors.enable(false);
        return active_fault_;
    }

    // 4. Critical low-voltage cutoff sustained >= 1000 ms to avoid transient sag trips.
    if (battery.critical_cutoff) {
        if (low_batt_start_ms_ == 0U) {
            low_batt_start_ms_ = now_ms;
        } else if ((now_ms - low_batt_start_ms_) >= 1000U) {
            active_fault_ = SafetyFault::LOW_BATTERY;
            motors.enable(false);
            return active_fault_;
        }
    } else {
        low_batt_start_ms_ = 0U;
    }

    // 5. Mandatory 500ms comm-loss watchdog when motion modes are active.
    if ((requested_mode == DriveMode::MANUAL || requested_mode == DriveMode::AUTONOMOUS) &&
        watchdog_.isExpired(now_ms)) {
        active_fault_ = SafetyFault::COMM_LOSS;
        motors.enable(false);
        return active_fault_;
    }

    if (active_fault_ != SafetyFault::ESTOP_SW && active_fault_ != SafetyFault::ESTOP_HW) {
        active_fault_ = SafetyFault::NONE;
    }

    const bool allow_drive =
        (active_fault_ == SafetyFault::NONE) && (requested_mode != DriveMode::IDLE);
    motors.enable(allow_drive);
    return active_fault_;
}

const char* SafetySupervisor::faultToString(SafetyFault fault) {
    switch (fault) {
        case SafetyFault::NONE:
            return "NONE";
        case SafetyFault::COMM_LOSS:
            return "COMM_LOSS_500MS";
        case SafetyFault::ESTOP_SW:
            return "ESTOP_SOFTWARE";
        case SafetyFault::ESTOP_HW:
            return "ESTOP_HARDWARE";
        case SafetyFault::TILT_ROLLOVER:
            return "TILT_ROLLOVER";
        case SafetyFault::LOW_BATTERY:
            return "LOW_BATTERY";
        default:
            return "UNKNOWN";
    }
}

}  // namespace roversar::safety
