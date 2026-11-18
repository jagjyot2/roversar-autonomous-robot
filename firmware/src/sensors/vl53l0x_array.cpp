#include "sensors/vl53l0x_array.h"

#include <Arduino.h>
#include <Wire.h>
#include <algorithm>

namespace roversar::sensors {

namespace {
constexpr uint8_t kDefaultAddr = 0x29;
constexpr uint8_t kRegModelId = 0xC0;
constexpr uint8_t kExpectedModelId = 0xEE;
constexpr uint8_t kRegI2cSlaveAddr = 0x8A;
constexpr uint8_t kRegSysrangeStart = 0x00;
constexpr uint8_t kRegResultIntStatus = 0x13;
constexpr uint8_t kRegResultRangeStatus = 0x14;
}  // namespace

bool Vl53l0xArray::begin() {
    // Hold all sensors in hardware shutdown before sequential address assignment.
    for (uint8_t i = 0; i < pins::TOF_COUNT; ++i) {
        pinMode(pins::TOF_XSHUT_PINS[i], OUTPUT);
        digitalWrite(pins::TOF_XSHUT_PINS[i], LOW);
        initialized_[i] = false;
    }
    delay(15);

    uint8_t active = 0;
    for (uint8_t i = 0; i < pins::TOF_COUNT; ++i) {
        digitalWrite(pins::TOF_XSHUT_PINS[i], HIGH);
        delay(10);
        if (initSingleSensor(i)) {
            initialized_[i] = true;
            ++active;
        }
    }
    last_scan_.active_sensors = active;
    return active > 0;
}

bool Vl53l0xArray::initSingleSensor(uint8_t idx) {
    uint8_t model_id = 0;
    const uint8_t target_addr = pins::TOF_I2C_ADDRS[idx];

    if (readReg8(kDefaultAddr, kRegModelId, model_id) && model_id == kExpectedModelId) {
        if (!writeReg8(kDefaultAddr, kRegI2cSlaveAddr, target_addr & 0x7F)) {
            return false;
        }
        delay(2);
    } else if (!(readReg8(target_addr, kRegModelId, model_id) && model_id == kExpectedModelId)) {
        return false;
    }

    // Standard VL53L0X continuous back-to-back ranging initialization sequence.
    writeReg8(target_addr, 0x88, 0x00);
    writeReg8(target_addr, 0x80, 0x01);
    writeReg8(target_addr, 0xFF, 0x01);
    writeReg8(target_addr, 0x00, 0x00);
    uint8_t stop_variable = 0;
    readReg8(target_addr, 0x91, stop_variable);
    writeReg8(target_addr, 0x00, 0x01);
    writeReg8(target_addr, 0xFF, 0x00);
    writeReg8(target_addr, 0x80, 0x00);

    // Start continuous back-to-back ranging mode (bit 1 = 1).
    return writeReg8(target_addr, kRegSysrangeStart, 0x02);
}

TofScan Vl53l0xArray::readAll() {
    TofScan scan{};
    float min_front = kMaxRangeM;
    uint8_t active = 0;

    for (uint8_t i = 0; i < pins::TOF_COUNT; ++i) {
        if (!initialized_[i]) {
            scan.ranges_m[i] = kMaxRangeM;
            scan.valid[i] = false;
            continue;
        }
        const float dist = readSingleRangeM(i);
        scan.ranges_m[i] = dist;
        scan.valid[i] = true;
        ++active;

        // Sensors 1, 2, 3 (-30, 0, +30 deg) form the forward collision corridor.
        if (i >= 1 && i <= 3) {
            min_front = std::min(min_front, dist);
        }
    }

    scan.min_front_m = min_front;
    scan.active_sensors = active;
    last_scan_ = scan;
    return scan;
}

float Vl53l0xArray::readSingleRangeM(uint8_t idx) {
    const uint8_t addr = pins::TOF_I2C_ADDRS[idx];
    uint8_t status = 0;
    if (!readReg8(addr, kRegResultIntStatus, status)) {
        return kMaxRangeM;
    }

    uint16_t raw_mm = 0;
    if (!readReg16(addr, kRegResultRangeStatus + 10, raw_mm)) {
        return kMaxRangeM;
    }

    // Clear interrupt flag.
    writeReg8(addr, 0x0B, 0x01);

    if (raw_mm < 20 || raw_mm > 2200) {
        return kMaxRangeM;
    }
    const float meters = static_cast<float>(raw_mm) * 0.001f;
    return std::clamp(meters, kMinRangeM, kMaxRangeM);
}

bool Vl53l0xArray::writeReg8(uint8_t i2c_addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Vl53l0xArray::readReg8(uint8_t i2c_addr, uint8_t reg, uint8_t& val) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(i2c_addr), 1) != 1) {
        return false;
    }
    val = Wire.read();
    return true;
}

bool Vl53l0xArray::readReg16(uint8_t i2c_addr, uint8_t reg, uint16_t& val) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(i2c_addr), 2) != 2) {
        return false;
    }
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    val = (static_cast<uint16_t>(hi) << 8) | lo;
    return true;
}

}  // namespace roversar::sensors
