#pragma once

namespace roversar::sensors {

struct BatteryStatus {
    float voltage_v{7.4f};
    float soc_percent{100.0f};
    bool low_warning{false};
    bool critical_cutoff{false};
};

/**
 * @brief Calibrated ADC1 battery pack monitor for 2S Li-Ion via 22k/10k divider.
 */
class BatteryMonitor {
public:
    void begin();
    BatteryStatus sample();
    const BatteryStatus& status() const { return status_; }

private:
    static float estimateSocPercent(float voltage_v);

    float filtered_v_{7.4f};
    bool seeded_{false};
    BatteryStatus status_{};
};

}  // namespace roversar::sensors
