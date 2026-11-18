#include "navigation/waypoint_follower.h"

#include <algorithm>
#include <cmath>

namespace roversar::navigation {

namespace {
constexpr float kPi = 3.14159265358979323846f;

float wrapAngle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}
}  // namespace

void WaypointFollower::setWaypoint(float x_m, float y_m) {
    waypoint_.x_m = x_m;
    waypoint_.y_m = y_m;
    waypoint_.active = true;
}

void WaypointFollower::clearWaypoint() {
    waypoint_.active = false;
}

AvoidanceOutput WaypointFollower::computeControl(const sensors::Pose2D& pose,
                                                 const sensors::TofScan& scan,
                                                 float cruise_speed_mps) {
    AvoidanceOutput out{};
    if (!waypoint_.active) {
        return out;
    }

    const float dx = waypoint_.x_m - pose.x_m;
    const float dy = waypoint_.y_m - pose.y_m;
    const float dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= kGoalToleranceM) {
        waypoint_.active = false;
        return out;
    }

    const float desired_theta = std::atan2(dy, dx);
    const float heading_err = wrapAngle(desired_theta - pose.theta_rad);
    const float speed = std::min(cruise_speed_mps, std::max(0.14f, dist * 0.8f));

    return planner_.computeSafeCommand(scan, heading_err, speed);
}

}  // namespace roversar::navigation
