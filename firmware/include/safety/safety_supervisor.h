#pragma once

#include "motor_control/motor_driver.h"
#include "roversar_config.h"
#include "safety/watchdog.h"
#include "sensors/battery_monitor.h"
#include "sensors/mpu6050_imu.h"
#include <cstdint>

namespace roversar::safety {

/**
 * @brief Central safety interlock enforcing 500ms comm-loss cutoff, software/hardware
 *        E-STOP latch, IMU rollover cutoff, and critical 2S battery protection.
 */
class SafetySupervisor {
public:
    void begin(uint32_t now_ms);
    void feedHeartbeat(uint32_t now_ms);
    void triggerSoftwareEstop();
    bool clearFaults(uint32_t now_ms);

    SafetyFault evaluate(uint32_t now_ms,
                         DriveMode requested_mode,
                         const sensors::ImuState& imu,
                         const sensors::BatteryStatus& battery,
                         motor_control::MotorDriver& motors);

    SafetyFault activeFault() const { return active_fault_; }
    bool isMotionPermitted() const { return active_fault_ == SafetyFault::NONE; }
    uint32_t watchdogAgeMs(uint32_t now_ms) const { return watchdog_.elapsedMs(now_ms); }

    static const char* faultToString(SafetyFault fault);

private:
    CommWatchdog watchdog_{COMM_WATCHDOG_TIMEOUT_MS};
    SafetyFault active_fault_{SafetyFault::NONE};
    bool sw_estop_latched_{false};
    uint32_t low_batt_start_ms_{0};
};

}  // namespace roversar::safety
