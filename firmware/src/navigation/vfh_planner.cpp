#include "navigation/vfh_planner.h"

#include "roversar_config.h"

#include <algorithm>
#include <cmath>

namespace roversar::navigation {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kSectorStepRad = (2.0f * kPi) / VfhPlanner::kNumSectors;
}  // namespace

float VfhPlanner::wrapAngle(float angle_rad) {
    while (angle_rad > kPi) angle_rad -= 2.0f * kPi;
    while (angle_rad < -kPi) angle_rad += 2.0f * kPi;
    return angle_rad;
}

int VfhPlanner::angleToSector(float angle_rad) {
    const float wrapped = wrapAngle(angle_rad) + kPi;  // [0, 2*pi]
    int sector = static_cast<int>(std::floor(wrapped / kSectorStepRad)) % kNumSectors;
    if (sector < 0) sector += kNumSectors;
    return sector;
}

float VfhPlanner::sectorToAngle(int sector) {
    return -kPi + (static_cast<float>(sector) + 0.5f) * kSectorStepRad;
}

AvoidanceOutput VfhPlanner::computeSafeCommand(const sensors::TofScan& scan,
                                               float desired_heading_error_rad,
                                               float desired_speed_mps) const {
    AvoidanceOutput out{};

    // Emergency proximity guard: rotate in place away from closest side.
    if (scan.min_front_m < kEmergencyReverseM) {
        const float left_clear = scan.ranges_m[0] + scan.ranges_m[1];
        const float right_clear = scan.ranges_m[3] + scan.ranges_m[4];
        const float turn_dir = (right_clear > left_clear) ? 1.0f : -1.0f;
        out.command.linear_mps = -0.08f;
        out.command.angular_rps = turn_dir * 2.2f;
        out.obstacle_avoidance_active = true;
        out.chosen_steer_rad = turn_dir * 1.57f;
        return out;
    }

    std::array<float, kNumSectors> histogram{};
    histogram.fill(0.0f);

    bool any_close = false;
    for (uint8_t i = 0; i < pins::TOF_COUNT; ++i) {
        const float d = scan.ranges_m[i];
        if (!scan.valid[i] || d >= kObstacleThresholdM) {
            continue;
        }
        any_close = true;
        const float magnitude = (kObstacleThresholdM - d) / kObstacleThresholdM;
        // Inflate angular footprint around obstacle beam (+/- 3 sectors = +/- 30 deg).
        const int center_sec = angleToSector(pins::TOF_ANGLES_RAD[i]);
        for (int offset = -3; offset <= 3; ++offset) {
            const int s = (center_sec + offset + kNumSectors) % kNumSectors;
            const float decay = 1.0f - (static_cast<float>(std::abs(offset)) * 0.22f);
            histogram[s] = std::max(histogram[s], magnitude * decay);
        }
    }

    const float target_err = wrapAngle(desired_heading_error_rad);
    if (!any_close) {
        const float angular = std::clamp(2.4f * target_err, -MAX_ANGULAR_VEL_RPS, MAX_ANGULAR_VEL_RPS);
        const float turn_penalty = std::max(0.25f, 1.0f - std::fabs(target_err) / kPi);
        out.command.linear_mps = std::clamp(desired_speed_mps * turn_penalty, 0.0f, MAX_LINEAR_VEL_MPS);
        out.command.angular_rps = angular;
        out.obstacle_avoidance_active = false;
        out.chosen_steer_rad = target_err;
        return out;
    }

    // Score all forward-hemisphere sectors [-100 deg .. +100 deg] for valley selection.
    float best_cost = 1e9f;
    float best_angle = target_err;
    for (int s = 0; s < kNumSectors; ++s) {
        const float ang = sectorToAngle(s);
        if (std::fabs(ang) > 1.75f) {  // ~100 deg forward cone
            continue;
        }
        const float cost = (4.5f * histogram[s]) + 1.0f * std::fabs(wrapAngle(ang - target_err));
        if (cost < best_cost) {
            best_cost = cost;
            best_angle = ang;
        }
    }

    const float speed_scale = std::clamp((scan.min_front_m - kEmergencyReverseM) / 0.50f, 0.20f, 1.0f);
    out.command.linear_mps = std::clamp(desired_speed_mps * speed_scale, 0.0f, MAX_LINEAR_VEL_MPS);
    out.command.angular_rps = std::clamp(2.8f * best_angle, -MAX_ANGULAR_VEL_RPS, MAX_ANGULAR_VEL_RPS);
    out.obstacle_avoidance_active = true;
    out.chosen_steer_rad = best_angle;
    return out;
}

}  // namespace roversar::navigation
