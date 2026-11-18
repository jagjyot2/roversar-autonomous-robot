#pragma once

#include "navigation/vfh_planner.h"
#include "sensors/encoder_odometry.h"
#include "sensors/vl53l0x_array.h"

namespace roversar::navigation {

struct Waypoint {
    float x_m{0.0f};
    float y_m{0.0f};
    bool active{false};
};

class WaypointFollower {
public:
    static constexpr float kGoalToleranceM = 0.12f;

    void setWaypoint(float x_m, float y_m);
    void clearWaypoint();
    bool hasActiveWaypoint() const { return waypoint_.active; }
    Waypoint currentWaypoint() const { return waypoint_; }

    AvoidanceOutput computeControl(const sensors::Pose2D& pose,
                                   const sensors::TofScan& scan,
                                   float cruise_speed_mps = 0.35f);

private:
    Waypoint waypoint_{};
    VfhPlanner planner_{};
};

}  // namespace roversar::navigation
