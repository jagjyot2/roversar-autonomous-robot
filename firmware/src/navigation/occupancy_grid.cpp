#include "navigation/occupancy_grid.h"

#include "roversar_config.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace roversar::navigation {

namespace {
constexpr int8_t kLogOddsMin = -50;
constexpr int8_t kLogOddsMax = 50;
constexpr int8_t kLogOddsFreeDelta = -4;
constexpr int8_t kLogOddsHitDelta = 12;
}  // namespace

OccupancyGrid2D::OccupancyGrid2D() {
    reset();
}

void OccupancyGrid2D::reset() {
    log_odds_.fill(0);
}

bool OccupancyGrid2D::worldToGrid(float x_m, float y_m, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((x_m - kOriginXM) / kResolutionM));
    gy = static_cast<int>(std::floor((y_m - kOriginYM) / kResolutionM));
    return gx >= 0 && gx < kWidth && gy >= 0 && gy < kHeight;
}

void OccupancyGrid2D::updateFromScan(const sensors::Pose2D& pose,
                                     const sensors::TofScan& scan) {
    int rx = 0;
    int ry = 0;
    if (!worldToGrid(pose.x_m, pose.y_m, rx, ry)) {
        return;
    }

    for (uint8_t i = 0; i < pins::TOF_COUNT; ++i) {
        if (!scan.valid[i]) {
            continue;
        }
        const float range = scan.ranges_m[i];
        const bool hit = range < (sensors::Vl53l0xArray::kMaxRangeM - 0.08f);
        const float beam_world_angle = pose.theta_rad + pins::TOF_ANGLES_RAD[i];
        const float end_x = pose.x_m + range * std::cos(beam_world_angle);
        const float end_y = pose.y_m + range * std::sin(beam_world_angle);

        int ex = 0;
        int ey = 0;
        if (worldToGrid(end_x, end_y, ex, ey)) {
            traceRay(rx, ry, ex, ey, hit);
        }
    }
}

void OccupancyGrid2D::traceRay(int x0, int y0, int x1, int y1, bool hit_obstacle) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int cx = x0;
    int cy = y0;
    while (true) {
        if (cx == x1 && cy == y1) {
            updateCell(cx, cy, hit_obstacle ? kLogOddsHitDelta : kLogOddsFreeDelta);
            break;
        }
        updateCell(cx, cy, kLogOddsFreeDelta);
        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            cx += sx;
        }
        if (e2 < dx) {
            err += dx;
            cy += sy;
        }
    }
}

void OccupancyGrid2D::updateCell(int gx, int gy, int8_t delta) {
    if (gx < 0 || gx >= kWidth || gy < 0 || gy >= kHeight) {
        return;
    }
    const size_t idx = static_cast<size_t>(gy) * kWidth + static_cast<size_t>(gx);
    const int updated = static_cast<int>(log_odds_[idx]) + static_cast<int>(delta);
    log_odds_[idx] = static_cast<int8_t>(
        std::clamp(updated, static_cast<int>(kLogOddsMin), static_cast<int>(kLogOddsMax)));
}

int8_t OccupancyGrid2D::cellLogOdds(int gx, int gy) const {
    if (gx < 0 || gx >= kWidth || gy < 0 || gy >= kHeight) {
        return 0;
    }
    return log_odds_[static_cast<size_t>(gy) * kWidth + static_cast<size_t>(gx)];
}

float OccupancyGrid2D::cellProbability(int gx, int gy) const {
    const float l = static_cast<float>(cellLogOdds(gx, gy)) * 0.1f;
    const float odds = std::exp(l);
    return odds / (1.0f + odds);
}

}  // namespace roversar::navigation
