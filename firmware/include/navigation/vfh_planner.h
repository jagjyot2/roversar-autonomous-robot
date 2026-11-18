#pragma once

#include "motor_control/kinematics.h"
#include "sensors/vl53l0x_array.h"
#include <array>

namespace roversar::navigation {

struct AvoidanceOutput {
    motor_control::Twist2D command{};
    bool obstacle_avoidance_active{false};
    float chosen_steer_rad{0.0f};
};

/**
 * @brief Vector Field Histogram (VFH) reactive local obstacle avoidance planner
 *        operating on the 5-beam VL53L0X ToF array.
 */
class VfhPlanner {
public:
    static constexpr int kNumSectors = 36;  // 10 deg per sector over [-180, +180)
    static constexpr float kObstacleThresholdM = 0.70f;
    static constexpr float kEmergencyReverseM = 0.22f;

    AvoidanceOutput computeSafeCommand(const sensors::TofScan& scan,
                                         float desired_heading_error_rad,
                                         float desired_speed_mps) const;

private:
    static int angleToSector(float angle_rad);
    static float sectorToAngle(int sector);
    static float wrapAngle(float angle_rad);
};

}  // namespace roversar::navigation
