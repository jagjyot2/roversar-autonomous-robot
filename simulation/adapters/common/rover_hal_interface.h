#pragma once

#include <array>
#include <cstdint>

namespace roversar::hal {

/**
 * @brief Canonical differential-drive motor driver command (TB6612FNG / L298N).
 */
struct MotorCommand {
    uint16_t left_pwm = 0;    // 10-bit LEDC duty [0..1023]
    bool left_in1 = false;
    bool left_in2 = false;
    uint16_t right_pwm = 0;   // 10-bit LEDC duty [0..1023]
    bool right_in1 = false;
    bool right_in2 = false;
    bool standby_active = false;  // true = STBY HIGH (motors enabled)
};

/**
 * @brief Quadrature encoder tick counts (1440 CPR at wheel shaft).
 */
struct EncoderReading {
    int64_t left_ticks = 0;
    int64_t right_ticks = 0;
};

/**
 * @brief 5-channel VL53L0X ToF array + auxiliary HC-SR04 ultrasonic sensor.
 */
struct DistanceArrayReading {
    std::array<float, 5> tof_distances_m{2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
    uint8_t tof_valid_mask = 0x1F;
    float aux_ultrasonic_m = 2.0f;
    bool aux_valid = true;
};

/**
 * @brief MPU6050 6-DOF IMU + fused orientation angles.
 */
struct ImuReading {
    std::array<float, 3> accel_mps2{0.0f, 0.0f, 9.80665f};
    std::array<float, 3> gyro_rps{0.0f, 0.0f, 0.0f};
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
};

/**
 * @brief 2S Li-Ion battery ADC & E-STOP interlock state.
 */
struct PowerAndSafetyReading {
    uint16_t vbat_adc_raw = 3100;  // 12-bit ADC1 reading on GPIO33
    float vbat_volts = 7.40f;      // Scaled via 22k / 10k divider
    bool estop_hw_tripped = false;  // NC switch on GPIO32 (HIGH = tripped)
    bool estop_sw_tripped = false;
};

/**
 * @brief Simulator-independent Hardware Abstraction Layer (HAL) for RoverSAR.
 *
 * Implemented by onboard ESP32 drivers, Wokwi custom-chip/UART bridge,
 * Python 50 Hz physics simulator, or preserved BenchTwin HIL adapter.
 */
class IRoverHal {
public:
    virtual ~IRoverHal() = default;

    virtual bool init() = 0;
    virtual void write_motors(const MotorCommand& cmd) = 0;
    virtual EncoderReading read_encoders() = 0;
    virtual DistanceArrayReading read_distances() = 0;
    virtual ImuReading read_imu() = 0;
    virtual PowerAndSafetyReading read_power_safety() = 0;
    virtual void set_status_led(bool on) = 0;
    virtual uint32_t millis_now() const = 0;
};

}  // namespace roversar::hal
