#include "sensors/encoder_odometry.h"

#include "roversar_config.h"

#include <Arduino.h>
#include <cmath>

namespace roversar::sensors {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr int8_t kQuadLut[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

volatile int64_t g_left_ticks = 0;
volatile int64_t g_right_ticks = 0;
volatile uint8_t g_left_state = 0;
volatile uint8_t g_right_state = 0;

float normalizeAngle(float rad) {
    while (rad > kPi) rad -= 2.0f * kPi;
    while (rad < -kPi) rad += 2.0f * kPi;
    return rad;
}
}  // namespace

void IRAM_ATTR EncoderOdometry::isrLeft() {
    const uint8_t a = static_cast<uint8_t>(digitalRead(pins::ENC_LEFT_A));
    const uint8_t b = static_cast<uint8_t>(digitalRead(pins::ENC_LEFT_B));
    const uint8_t next = static_cast<uint8_t>((a << 1) | b);
    const uint8_t idx = static_cast<uint8_t>((g_left_state << 2) | next);
    g_left_ticks += kQuadLut[idx];
    g_left_state = next;
}

void IRAM_ATTR EncoderOdometry::isrRight() {
    const uint8_t a = static_cast<uint8_t>(digitalRead(pins::ENC_RIGHT_A));
    const uint8_t b = static_cast<uint8_t>(digitalRead(pins::ENC_RIGHT_B));
    const uint8_t next = static_cast<uint8_t>((a << 1) | b);
    const uint8_t idx = static_cast<uint8_t>((g_right_state << 2) | next);
    g_right_ticks += kQuadLut[idx];
    g_right_state = next;
}

void EncoderOdometry::begin() {
    pinMode(pins::ENC_LEFT_A, INPUT);
    pinMode(pins::ENC_LEFT_B, INPUT);
    pinMode(pins::ENC_RIGHT_A, INPUT);
    pinMode(pins::ENC_RIGHT_B, INPUT);

    g_left_state = static_cast<uint8_t>((digitalRead(pins::ENC_LEFT_A) << 1) |
                                        digitalRead(pins::ENC_LEFT_B));
    g_right_state = static_cast<uint8_t>((digitalRead(pins::ENC_RIGHT_A) << 1) |
                                         digitalRead(pins::ENC_RIGHT_B));

    attachInterrupt(digitalPinToInterrupt(pins::ENC_LEFT_A), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ENC_LEFT_B), isrLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ENC_RIGHT_A), isrRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins::ENC_RIGHT_B), isrRight, CHANGE);
}

Pose2D EncoderOdometry::update(float dt_s, bool imu_healthy, float imu_yaw_rate_rps) {
    if (dt_s <= 1e-5f) {
        return pose_;
    }

    noInterrupts();
    const int64_t ticks_l = g_left_ticks;
    const int64_t ticks_r = g_right_ticks;
    interrupts();

    const int64_t dl_ticks = ticks_l - prev_left_ticks_;
    const int64_t dr_ticks = ticks_r - prev_right_ticks_;
    prev_left_ticks_ = ticks_l;
    prev_right_ticks_ = ticks_r;

    const float rad_per_tick = (2.0f * kPi) / static_cast<float>(ENCODER_TICKS_PER_REV);
    const float d_phi_l = static_cast<float>(dl_ticks) * rad_per_tick;
    const float d_phi_r = static_cast<float>(dr_ticks) * rad_per_tick;

    const float ds_l = d_phi_l * WHEEL_RADIUS_M;
    const float ds_r = d_phi_r * WHEEL_RADIUS_M;
    const float ds = 0.5f * (ds_l + ds_r);
    const float d_theta_enc = (ds_r - ds_l) / WHEEL_BASE_M;

    // Fuse wheel-odometry heading change with MPU6050 yaw rate when healthy.
    float d_theta = d_theta_enc;
    if (imu_healthy) {
        const float d_theta_imu = imu_yaw_rate_rps * dt_s;
        d_theta = 0.70f * d_theta_imu + 0.30f * d_theta_enc;
    }

    // Mid-point RK2 pose integration.
    const float theta_mid = pose_.theta_rad + 0.5f * d_theta;
    pose_.x_m += ds * std::cos(theta_mid);
    pose_.y_m += ds * std::sin(theta_mid);
    pose_.theta_rad = normalizeAngle(pose_.theta_rad + d_theta);

    pose_.left_rad_s = d_phi_l / dt_s;
    pose_.right_rad_s = d_phi_r / dt_s;
    pose_.linear_mps = ds / dt_s;
    pose_.angular_rps = d_theta / dt_s;
    pose_.left_ticks = ticks_l;
    pose_.right_ticks = ticks_r;

    return pose_;
}

void EncoderOdometry::resetPose(float x_m, float y_m, float theta_rad) {
    noInterrupts();
    g_left_ticks = 0;
    g_right_ticks = 0;
    interrupts();
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    pose_.x_m = x_m;
    pose_.y_m = y_m;
    pose_.theta_rad = normalizeAngle(theta_rad);
    pose_.left_ticks = 0;
    pose_.right_ticks = 0;
}

}  // namespace roversar::sensors
