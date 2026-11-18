#include "sensors/mpu6050_imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <cmath>

namespace roversar::sensors {

namespace {
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegAccelXoutH = 0x3B;

constexpr float kAccelScaleG = 16384.0f;     // +/- 2g
constexpr float kGyroScaleDps = 65.5f;       // +/- 500 deg/s
constexpr float kGravityMps2 = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kPi = 3.14159265358979323846f;

float wrapAngle(float rad) {
    while (rad > kPi) rad -= 2.0f * kPi;
    while (rad < -kPi) rad += 2.0f * kPi;
    return rad;
}
}  // namespace

bool Mpu6050Imu::begin() {
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegWhoAmI);
    if (Wire.endTransmission(false) != 0) {
        state_.healthy = false;
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(kI2cAddr), 1) != 1) {
        state_.healthy = false;
        return false;
    }
    const uint8_t who = Wire.read();
    if (who != 0x68 && who != 0x70 && who != 0x72) {
        state_.healthy = false;
        return false;
    }

    // Wake up MPU6050 and select PLL with X-axis gyro reference.
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegPwrMgmt1);
    Wire.write(0x01);
    Wire.endTransmission();

    // DLPF bandwidth ~44 Hz (CONFIG = 0x03).
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegConfig);
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro +/- 500 deg/s (FS_SEL = 1 -> 0x08).
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegGyroConfig);
    Wire.write(0x08);
    Wire.endTransmission();

    // Accel +/- 2g (AFS_SEL = 0 -> 0x00).
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegAccelConfig);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(40);
    calibrateGyro(150);
    state_.healthy = true;
    return true;
}

void Mpu6050Imu::calibrateGyro(uint16_t samples) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t valid = 0;

    int16_t raw[7]{};
    for (uint16_t i = 0; i < samples; ++i) {
        if (readRawBlock(raw)) {
            sum_x += static_cast<float>(raw[4]) / kGyroScaleDps;
            sum_y += static_cast<float>(raw[5]) / kGyroScaleDps;
            sum_z += static_cast<float>(raw[6]) / kGyroScaleDps;
            ++valid;
        }
        delay(2);
    }

    if (valid > 0) {
        gyro_x_bias_dps_ = sum_x / static_cast<float>(valid);
        gyro_y_bias_dps_ = sum_y / static_cast<float>(valid);
        gyro_z_bias_dps_ = sum_z / static_cast<float>(valid);
    }
}

ImuState Mpu6050Imu::update(float dt_s) {
    int16_t raw[7]{};
    if (!readRawBlock(raw)) {
        state_.healthy = false;
        return state_;
    }

    const float ax = (static_cast<float>(raw[0]) / kAccelScaleG) * kGravityMps2;
    const float ay = (static_cast<float>(raw[1]) / kAccelScaleG) * kGravityMps2;
    const float az = (static_cast<float>(raw[2]) / kAccelScaleG) * kGravityMps2;

    const float gx_dps = (static_cast<float>(raw[4]) / kGyroScaleDps) - gyro_x_bias_dps_;
    const float gy_dps = (static_cast<float>(raw[5]) / kGyroScaleDps) - gyro_y_bias_dps_;
    const float gz_dps = (static_cast<float>(raw[6]) / kGyroScaleDps) - gyro_z_bias_dps_;

    const float acc_roll_deg = std::atan2(ay, std::sqrt(ax * ax + az * az)) * kRadToDeg;
    const float acc_pitch_deg = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;

    constexpr float kAlpha = 0.96f;
    state_.roll_deg = kAlpha * (state_.roll_deg + gx_dps * dt_s) + (1.0f - kAlpha) * acc_roll_deg;
    state_.pitch_deg = kAlpha * (state_.pitch_deg + gy_dps * dt_s) + (1.0f - kAlpha) * acc_pitch_deg;

    // Deadband tiny gyro noise when stationary.
    const float filtered_gz_dps = (std::fabs(gz_dps) < 0.35f) ? 0.0f : gz_dps;
    state_.yaw_rate_rps = filtered_gz_dps * kDegToRad;
    state_.yaw_rad = wrapAngle(state_.yaw_rad + state_.yaw_rate_rps * dt_s);
    state_.accel_x_mps2 = ax;
    state_.accel_y_mps2 = ay;
    state_.accel_z_mps2 = az;
    state_.healthy = true;

    return state_;
}

bool Mpu6050Imu::readRawBlock(int16_t raw[7]) {
    Wire.beginTransmission(kI2cAddr);
    Wire.write(kRegAccelXoutH);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(kI2cAddr), 14) != 14) {
        return false;
    }
    for (int i = 0; i < 7; ++i) {
        const uint8_t hi = Wire.read();
        const uint8_t lo = Wire.read();
        raw[i] = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    }
    return true;
}

}  // namespace roversar::sensors
