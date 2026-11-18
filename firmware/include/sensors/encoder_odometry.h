#pragma once

#include <Arduino.h>
#include <cstdint>

namespace roversar::sensors {

struct Pose2D {
    float x_m{0.0f};
    float y_m{0.0f};
    float theta_rad{0.0f};
    float left_rad_s{0.0f};
    float right_rad_s{0.0f};
    float linear_mps{0.0f};
    float angular_rps{0.0f};
    int64_t left_ticks{0};
    int64_t right_ticks{0};
};

/**
 * @brief Quadrature encoder interrupt decoder and RK2 differential-drive odometry
 *        with optional IMU yaw complementary fusion.
 */
class EncoderOdometry {
public:
    void begin();
    Pose2D update(float dt_s, bool imu_healthy, float imu_yaw_rate_rps);
    void resetPose(float x_m = 0.0f, float y_m = 0.0f, float theta_rad = 0.0f);

    const Pose2D& pose() const { return pose_; }

private:
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();

    int64_t prev_left_ticks_{0};
    int64_t prev_right_ticks_{0};
    Pose2D pose_{};
};

}  // namespace roversar::sensors
