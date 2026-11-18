#pragma once

#include "sensors/encoder_odometry.h"
#include "sensors/vl53l0x_array.h"
#include <array>
#include <cstdint>

namespace roversar::navigation {

/**
 * @brief Compact 2D Log-Odds Bayesian Occupancy Grid (80x80 cells @ 0.10m = 8m x 8m).
 */
class OccupancyGrid2D {
public:
    static constexpr int kWidth = 80;
    static constexpr int kHeight = 80;
    static constexpr float kResolutionM = 0.10f;
    static constexpr float kOriginXM = -4.0f;
    static constexpr float kOriginYM = -4.0f;

    OccupancyGrid2D();

    void reset();
    void updateFromScan(const sensors::Pose2D& pose, const sensors::TofScan& scan);
    bool worldToGrid(float x_m, float y_m, int& gx, int& gy) const;
    float cellProbability(int gx, int gy) const;
    int8_t cellLogOdds(int gx, int gy) const;

private:
    void traceRay(int x0, int y0, int x1, int y1, bool hit_obstacle);
    void updateCell(int gx, int gy, int8_t delta);

    std::array<int8_t, kWidth * kHeight> log_odds_{};
};

}  // namespace roversar::navigation
