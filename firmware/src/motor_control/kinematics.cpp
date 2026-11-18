#include "motor_control/kinematics.h"

#include <algorithm>
#include <cmath>

namespace roversar::motor_control {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}  // namespace

DiffDriveKinematics::DiffDriveKinematics(float wheel_radius_m,
                                         float wheel_base_m,
                                         float max_wheel_rad_s)
    : wheel_radius_m_(wheel_radius_m),
      wheel_base_m_(wheel_base_m),
      max_wheel_rad_s_(max_wheel_rad_s) {}

WheelTargets DiffDriveKinematics::inverse(float linear_mps, float angular_rps) const {
    const float v = std::clamp(linear_mps, -MAX_LINEAR_VEL_MPS, MAX_LINEAR_VEL_MPS);
    const float w = std::clamp(angular_rps, -MAX_ANGULAR_VEL_RPS, MAX_ANGULAR_VEL_RPS);

    float omega_l = (v - 0.5f * wheel_base_m_ * w) / wheel_radius_m_;
    float omega_r = (v + 0.5f * wheel_base_m_ * w) / wheel_radius_m_;

    // Preserve turn curvature by proportionally scaling both wheels if either saturates.
    const float max_mag = std::max(std::fabs(omega_l), std::fabs(omega_r));
    if (max_mag > max_wheel_rad_s_ && max_mag > 1e-6f) {
        const float scale = max_wheel_rad_s_ / max_mag;
        omega_l *= scale;
        omega_r *= scale;
    }

    return WheelTargets{omega_l, omega_r};
}

Twist2D DiffDriveKinematics::forward(float left_rad_s, float right_rad_s) const {
    const float linear = 0.5f * wheel_radius_m_ * (right_rad_s + left_rad_s);
    const float angular = (wheel_radius_m_ / wheel_base_m_) * (right_rad_s - left_rad_s);
    return Twist2D{linear, angular};
}

float DiffDriveKinematics::radPerSecToRpm(float rad_s) {
    return rad_s * 60.0f / (2.0f * kPi);
}

float DiffDriveKinematics::rpmToRadPerSec(float rpm) {
    return rpm * (2.0f * kPi) / 60.0f;
}

}  // namespace roversar::motor_control
