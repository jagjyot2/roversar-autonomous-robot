#pragma once

#include <cstdint>

namespace roversar::motor_control {

/**
 * @brief Hardware dual H-bridge PWM driver supporting TB6612FNG (STBY + IN1/IN2 + PWM)
 *        and L298N (ENA/ENB + IN1..IN4).
 */
class MotorDriver {
public:
    void begin();
    void setNormalizedEffort(float left_effort, float right_effort);
    void enable(bool enabled);
    void hardStop();

    bool isEnabled() const { return enabled_; }
    float leftEffort() const { return last_left_effort_; }
    float rightEffort() const { return last_right_effort_; }

private:
    static constexpr uint32_t kPwmFreqHz = 20000U;
    static constexpr uint8_t kPwmResolutionBits = 10U;
    static constexpr uint32_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

    void applyChannel(uint8_t pwm_pin,
                      uint8_t pwm_channel,
                      uint8_t in1_pin,
                      uint8_t in2_pin,
                      float effort);

    bool enabled_{false};
    float last_left_effort_{0.0f};
    float last_right_effort_{0.0f};
};

}  // namespace roversar::motor_control
