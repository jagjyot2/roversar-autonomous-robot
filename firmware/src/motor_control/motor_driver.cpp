#include "motor_control/motor_driver.h"

#include "roversar_config.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace roversar::motor_control {

namespace {
constexpr uint8_t kLeftPwmChannel = 0;
constexpr uint8_t kRightPwmChannel = 1;
constexpr float kMinActiveEffort = 0.03f;
}  // namespace

void MotorDriver::begin() {
    pinMode(pins::MOTOR_LEFT_IN1, OUTPUT);
    pinMode(pins::MOTOR_LEFT_IN2, OUTPUT);
    pinMode(pins::MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(pins::MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(pins::MOTOR_STBY, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pins::MOTOR_LEFT_PWM, kPwmFreqHz, kPwmResolutionBits);
    ledcAttach(pins::MOTOR_RIGHT_PWM, kPwmFreqHz, kPwmResolutionBits);
#else
    ledcSetup(kLeftPwmChannel, kPwmFreqHz, kPwmResolutionBits);
    ledcSetup(kRightPwmChannel, kPwmFreqHz, kPwmResolutionBits);
    ledcAttachPin(pins::MOTOR_LEFT_PWM, kLeftPwmChannel);
    ledcAttachPin(pins::MOTOR_RIGHT_PWM, kRightPwmChannel);
#endif

    hardStop();
}

void MotorDriver::enable(bool enabled) {
    enabled_ = enabled;
#ifdef MOTOR_DRIVER_TB6612FNG
    digitalWrite(pins::MOTOR_STBY, enabled_ ? HIGH : LOW);
#endif
    if (!enabled_) {
        hardStop();
    }
}

void MotorDriver::setNormalizedEffort(float left_effort, float right_effort) {
    if (!enabled_) {
        hardStop();
        return;
    }

    last_left_effort_ = std::clamp(left_effort, -1.0f, 1.0f);
    last_right_effort_ = std::clamp(right_effort, -1.0f, 1.0f);

    applyChannel(pins::MOTOR_LEFT_PWM,
                 kLeftPwmChannel,
                 pins::MOTOR_LEFT_IN1,
                 pins::MOTOR_LEFT_IN2,
                 last_left_effort_);
    applyChannel(pins::MOTOR_RIGHT_PWM,
                 kRightPwmChannel,
                 pins::MOTOR_RIGHT_IN1,
                 pins::MOTOR_RIGHT_IN2,
                 last_right_effort_);
}

void MotorDriver::hardStop() {
    last_left_effort_ = 0.0f;
    last_right_effort_ = 0.0f;

    digitalWrite(pins::MOTOR_LEFT_IN1, LOW);
    digitalWrite(pins::MOTOR_LEFT_IN2, LOW);
    digitalWrite(pins::MOTOR_RIGHT_IN1, LOW);
    digitalWrite(pins::MOTOR_RIGHT_IN2, LOW);
    digitalWrite(pins::MOTOR_STBY, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pins::MOTOR_LEFT_PWM, 0);
    ledcWrite(pins::MOTOR_RIGHT_PWM, 0);
#else
    ledcWrite(kLeftPwmChannel, 0);
    ledcWrite(kRightPwmChannel, 0);
#endif
}

void MotorDriver::applyChannel(uint8_t pwm_pin,
                               uint8_t pwm_channel,
                               uint8_t in1_pin,
                               uint8_t in2_pin,
                               float effort) {
    const float mag = std::fabs(effort);
    if (mag < kMinActiveEffort) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(pwm_pin, 0);
#else
        ledcWrite(pwm_channel, 0);
#endif
        return;
    }

    if (effort > 0.0f) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
    }

    const uint32_t duty = static_cast<uint32_t>(mag * static_cast<float>(kPwmMaxDuty));
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwm_pin, duty);
#else
    ledcWrite(pwm_channel, duty);
#endif
}

}  // namespace roversar::motor_control
