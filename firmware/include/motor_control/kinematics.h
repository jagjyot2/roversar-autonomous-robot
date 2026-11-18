#pragma once

#include "roversar_config.h"

namespace roversar::motor_control {

struct WheelTargets {
    float left_rad_s{0.0f};
    float right_rad_s{0.0f};
};

struct Twist2D {
    float linear_mps{0.0f};
    float angular_rps{0.0f};
};

/**
 * @brief Differential-drive forward and inverse kinematics converter.
 */
class DiffDriveKinematics {
public:
    explicit DiffDriveKinematics(float wheel_radius_m = WHEEL_RADIUS_M,
                                 float wheel_base_m = WHEEL_BASE_M,
                                 float max_wheel_rad_s = MAX_WHEEL_RAD_S);

    WheelTargets inverse(float linear_mps, float angular_rps) const;
    Twist2D forward(float left_rad_s, float right_rad_s) const;

    static float radPerSecToRpm(float rad_s);
    static float rpmToRadPerSec(float rpm);

private:
    float wheel_radius_m_;
    float wheel_base_m_;
    float max_wheel_rad_s_;
};

}  // namespace roversar::motor_control
