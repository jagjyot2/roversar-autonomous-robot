#pragma once

#include "roversar_config.h"
#include <array>
#include <cstdint>

namespace roversar::sensors {

struct TofScan {
    std::array<float, pins::TOF_COUNT> ranges_m{2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
    std::array<bool, pins::TOF_COUNT> valid{false, false, false, false, false};
    float min_front_m{2.0f};
    uint8_t active_sensors{0};
};

/**
 * @brief 5-channel VL53L0X Time-of-Flight array driver over shared I2C bus
 *        using sequential XSHUT address reassignment (0x30..0x34).
 */
class Vl53l0xArray {
public:
    static constexpr float kMinRangeM = 0.03f;
    static constexpr float kMaxRangeM = 2.00f;

    bool begin();
    TofScan readAll();

    const TofScan& lastScan() const { return last_scan_; }

private:
    bool writeReg8(uint8_t i2c_addr, uint8_t reg, uint8_t val);
    bool readReg8(uint8_t i2c_addr, uint8_t reg, uint8_t& val);
    bool readReg16(uint8_t i2c_addr, uint8_t reg, uint16_t& val);
    bool initSingleSensor(uint8_t idx);
    float readSingleRangeM(uint8_t idx);

    std::array<bool, pins::TOF_COUNT> initialized_{false, false, false, false, false};
    TofScan last_scan_{};
};

}  // namespace roversar::sensors
