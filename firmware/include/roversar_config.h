#pragma once

#include <Arduino.h>
#include <cstdint>

namespace roversar {

// -----------------------------------------------------------------------------
// Physical Robot Constants (must match simulator/config.py & hardware BOM)
// -----------------------------------------------------------------------------
constexpr float WHEEL_RADIUS_M = 0.0325f;          // 65 mm diameter wheel
constexpr float WHEEL_BASE_M = 0.1600f;            // 160 mm track width
constexpr uint32_t ENCODER_TICKS_PER_REV = 1440U;  // Quadrature 4x * 360 CPR
constexpr float MAX_WHEEL_RAD_S = 18.85f;          // ~180 RPM max wheel speed
constexpr float MAX_LINEAR_VEL_MPS = 0.60f;        // m/s
constexpr float MAX_ANGULAR_VEL_RPS = 3.50f;       // rad/s

// -----------------------------------------------------------------------------
// Hardware GPIO Map (ESP32-WROOM-32D, ADC1-only with WiFi active)
// -----------------------------------------------------------------------------
namespace pins {
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

// Quadrature wheel encoders (input-only GPIOs 34, 35, 36, 39)
constexpr uint8_t ENC_LEFT_A = 34;
constexpr uint8_t ENC_LEFT_B = 35;
constexpr uint8_t ENC_RIGHT_A = 36;
constexpr uint8_t ENC_RIGHT_B = 39;

// TB6612FNG / L298N Motor Driver
constexpr uint8_t MOTOR_LEFT_PWM = 25;
constexpr uint8_t MOTOR_LEFT_IN1 = 26;
constexpr uint8_t MOTOR_LEFT_IN2 = 27;

constexpr uint8_t MOTOR_RIGHT_PWM = 19;
constexpr uint8_t MOTOR_RIGHT_IN1 = 18;
constexpr uint8_t MOTOR_RIGHT_IN2 = 23;

constexpr uint8_t MOTOR_STBY = 4;

// VL53L0X 5-sensor ToF array XSHUT control (-60, -30, 0, +30, +60 deg)
constexpr uint8_t TOF_COUNT = 5;
constexpr uint8_t TOF_XSHUT_PINS[TOF_COUNT] = {13, 14, 16, 17, 5};
constexpr uint8_t TOF_I2C_ADDRS[TOF_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr float TOF_ANGLES_RAD[TOF_COUNT] = {
    -1.04719755f,  // -60 deg
    -0.52359878f,  // -30 deg
     0.00000000f,  //   0 deg (center)
     0.52359878f,  // +30 deg
     1.04719755f   // +60 deg
};

// Safety & Monitoring
constexpr uint8_t VBAT_ADC = 33;          // ADC1_CH5 via 22k/10k divider
constexpr uint8_t ESTOP_HW = 32;          // NC E-STOP switch to GND (HIGH = tripped)
constexpr uint8_t STATUS_LED = 2;
}  // namespace pins

// -----------------------------------------------------------------------------
// Battery Voltage Divider (2S Li-Ion: 8.4V full, 7.4V nom, 6.0V cutoff)
// -----------------------------------------------------------------------------
constexpr float VBAT_DIVIDER_R1_OHM = 22000.0f;
constexpr float VBAT_DIVIDER_R2_OHM = 10000.0f;
constexpr float VBAT_LOW_WARN_V = 6.40f;
constexpr float VBAT_CRITICAL_CUTOFF_V = 6.00f;

// -----------------------------------------------------------------------------
// Safety Watchdog & Interlock Thresholds
// -----------------------------------------------------------------------------
constexpr uint32_t COMM_WATCHDOG_TIMEOUT_MS = 500U;  // Mandatory 500ms motor halt
constexpr float MAX_SAFE_TILT_DEG = 35.0f;           // Roll/pitch rollover cutoff
constexpr float OBSTACLE_STOP_DIST_M = 0.22f;        // Emergency obstacle proximity

// -----------------------------------------------------------------------------
// Control Loop Timing & Networking Defaults
// -----------------------------------------------------------------------------
constexpr uint32_t CONTROL_LOOP_HZ = 50U;
constexpr uint32_t SENSOR_LOOP_HZ = 20U;
constexpr uint32_t TELEMETRY_LOOP_HZ = 10U;
constexpr uint16_t WS_SERVER_PORT = 80;
constexpr const char* DEFAULT_AP_SSID = "RoverSAR-AP";
constexpr const char* DEFAULT_CAM_URL = "http://192.168.4.2:81/stream";

enum class DriveMode : uint8_t {
    IDLE = 0,
    MANUAL = 1,
    AUTONOMOUS = 2,
    ESTOP = 3
};

enum class SafetyFault : uint8_t {
    NONE = 0,
    COMM_LOSS = 1,
    ESTOP_SW = 2,
    ESTOP_HW = 3,
    TILT_ROLLOVER = 4,
    LOW_BATTERY = 5
};

}  // namespace roversar
