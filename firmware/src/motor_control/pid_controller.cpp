#include "motor_control/pid_controller.h"

#include <algorithm>
#include <cmath>

namespace roversar::motor_control {

WheelPidController::WheelPidController(float kp,
                                       float ki,
                                       float kd,
                                       float kf,
                                       float out_min,
                                       float out_max)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      kf_(kf),
      out_min_(out_min),
      out_max_(out_max) {}

float WheelPidController::update(float setpoint_rad_s, float measured_rad_s, float dt_s) {
    if (dt_s <= 1e-5f) {
        return 0.0f;
    }

    if (std::fabs(setpoint_rad_s) < 0.02f && std::fabs(measured_rad_s) < 0.10f) {
        reset();
        return 0.0f;
    }

    const float error = setpoint_rad_s - measured_rad_s;
    const float p_term = kp_ * error;

    const float tentative_integral = integral_ + error * dt_s;
    float d_term = 0.0f;
    if (has_prev_) {
        d_term = kd_ * ((error - prev_error_) / dt_s);
    }
    prev_error_ = error;
    has_prev_ = true;

    const float ff_term = kf_ * setpoint_rad_s;
    const float raw_output = ff_term + p_term + ki_ * tentative_integral + d_term;

    // Conditional integration anti-windup: only commit integral if unsaturated or unwinding.
    const bool saturated_high = (raw_output > out_max_) && (error > 0.0f);
    const bool saturated_low = (raw_output < out_min_) && (error < 0.0f);
    if (!saturated_high && !saturated_low) {
        integral_ = tentative_integral;
    }

    const float output = ff_term + p_term + ki_ * integral_ + d_term;
    return std::clamp(output, out_min_, out_max_);
}

void WheelPidController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_ = false;
}

void WheelPidController::setGains(float kp, float ki, float kd, float kf) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
}

}  // namespace roversar::motor_control
