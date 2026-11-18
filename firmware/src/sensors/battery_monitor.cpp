#include "sensors/battery_monitor.h"

#include "roversar_config.h"

#include <Arduino.h>
#include <algorithm>

namespace roversar::sensors {

void BatteryMonitor::begin() {
    pinMode(pins::VBAT_ADC, INPUT);
    analogSetPinAttenuation(pins::VBAT_ADC, ADC_11db);
    sample();
}

BatteryStatus BatteryMonitor::sample() {
    uint32_t mv_sum = 0;
    constexpr uint8_t kSamples = 8;
    for (uint8_t i = 0; i < kSamples; ++i) {
        mv_sum += analogReadMilliVolts(pins::VBAT_ADC);
    }

    const float adc_v = (static_cast<float>(mv_sum) / static_cast<float>(kSamples)) * 0.001f;
    const float divider_ratio =
        (VBAT_DIVIDER_R1_OHM + VBAT_DIVIDER_R2_OHM) / VBAT_DIVIDER_R2_OHM;
    const float raw_bat_v = adc_v * divider_ratio;

    if (!seeded_) {
        filtered_v_ = raw_bat_v;
        seeded_ = true;
    } else {
        filtered_v_ = 0.85f * filtered_v_ + 0.15f * raw_bat_v;
    }

    status_.voltage_v = filtered_v_;
    status_.soc_percent = estimateSocPercent(filtered_v_);
    status_.low_warning = filtered_v_ <= VBAT_LOW_WARN_V;
    status_.critical_cutoff = filtered_v_ <= VBAT_CRITICAL_CUTOFF_V;
    return status_;
}

float BatteryMonitor::estimateSocPercent(float voltage_v) {
    // 2S Li-Ion resting/load OCV curve approximation [6.0V .. 8.4V].
    const float norm = (voltage_v - 6.00f) / (8.40f - 6.00f);
    return std::clamp(norm * 100.0f, 0.0f, 100.0f);
}

}  // namespace roversar::sensors
