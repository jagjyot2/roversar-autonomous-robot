#pragma once

namespace roversar::motor_control {

/**
 * @brief Discrete-time wheel velocity PID controller with anti-windup clamping
 *        and optional open-loop feedforward gain.
 */
class WheelPidController {
public:
    WheelPidController(float kp,
                       float ki,
                       float kd,
                       float kf = 0.045f,
                       float out_min = -1.0f,
                       float out_max = 1.0f);

    float update(float setpoint_rad_s, float measured_rad_s, float dt_s);
    void reset();
    void setGains(float kp, float ki, float kd, float kf);

private:
    float kp_;
    float ki_;
    float kd_;
    float kf_;
    float out_min_;
    float out_max_;

    float integral_{0.0f};
    float prev_error_{0.0f};
    bool has_prev_{false};
};

}  // namespace roversar::motor_control
