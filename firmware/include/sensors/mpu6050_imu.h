#pragma once

#include <cstdint>

namespace roversar::sensors {

struct ImuState {
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    float yaw_rad{0.0f};
    float yaw_rate_rps{0.0f};
    float accel_x_mps2{0.0f};
    float accel_y_mps2{0.0f};
    float accel_z_mps2{9.81f};
    bool healthy{false};
};

/**
 * @brief MPU6050 6-DoF IMU driver with startup gyro-bias calibration and
 *        complementary tilt + yaw rate integration.
 */
class Mpu6050Imu {
public:
    static constexpr uint8_t kI2cAddr = 0x68;

    bool begin();
    void calibrateGyro(uint16_t samples = 200);
    ImuState update(float dt_s);

    const ImuState& state() const { return state_; }

private:
    bool readRawBlock(int16_t raw[7]);

    float gyro_z_bias_dps_{0.0f};
    float gyro_x_bias_dps_{0.0f};
    float gyro_y_bias_dps_{0.0f};
    ImuState state_{};
};

}  // namespace roversar::sensors
