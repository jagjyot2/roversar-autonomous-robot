#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include "camera/esp32_cam_link.h"
#include "communication/ws_telemetry_server.h"
#include "motor_control/kinematics.h"
#include "motor_control/motor_driver.h"
#include "motor_control/pid_controller.h"
#include "navigation/occupancy_grid.h"
#include "navigation/waypoint_follower.h"
#include "roversar_config.h"
#include "safety/safety_supervisor.h"
#include "sensors/battery_monitor.h"
#include "sensors/encoder_odometry.h"
#include "sensors/mpu6050_imu.h"
#include "sensors/vl53l0x_array.h"

namespace {

using namespace roversar;

motor_control::DiffDriveKinematics g_kinematics;
motor_control::WheelPidController g_left_pid(0.065f, 0.14f, 0.002f, 0.048f);
motor_control::WheelPidController g_right_pid(0.065f, 0.14f, 0.002f, 0.048f);
motor_control::MotorDriver g_motors;

sensors::Vl53l0xArray g_tof;
sensors::Mpu6050Imu g_imu;
sensors::EncoderOdometry g_odom;
sensors::BatteryMonitor g_battery;

navigation::OccupancyGrid2D g_grid;
navigation::WaypointFollower g_nav;
safety::SafetySupervisor g_safety;
communication::WsTelemetryServer g_ws;
camera::Esp32CamLink g_cam;

DriveMode g_mode = DriveMode::IDLE;
motor_control::Twist2D g_manual_cmd{};
bool g_avoidance_active = false;

uint32_t g_last_control_ms = 0;
uint32_t g_last_sensor_ms = 0;
uint32_t g_last_telemetry_ms = 0;

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(pins::STATUS_LED, OUTPUT);
    digitalWrite(pins::STATUS_LED, LOW);

    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, 400000U);

    g_motors.begin();
    g_odom.begin();
    g_battery.begin();
    g_imu.begin();
    g_tof.begin();

    const uint32_t now_ms = millis();
    g_safety.begin(now_ms);
    g_cam.begin(DEFAULT_CAM_URL);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID);
    g_ws.begin(WS_SERVER_PORT);

    g_last_control_ms = now_ms;
    g_last_sensor_ms = now_ms;
    g_last_telemetry_ms = now_ms;

    Serial.println("[RoverSAR] Firmware booted cleanly in IDLE safe state.");
}

void loop() {
    const uint32_t now_ms = millis();

    // 1. Poll inbound WebSocket / HTTP telemetry commands and feed watchdog.
    const auto cmd = g_ws.poll();
    if (cmd.has_heartbeat) {
        g_safety.feedHeartbeat(now_ms);
    }
    if (cmd.trigger_estop) {
        g_safety.triggerSoftwareEstop();
        g_mode = DriveMode::ESTOP;
    }
    if (cmd.clear_estop && g_safety.clearFaults(now_ms)) {
        g_mode = DriveMode::IDLE;
        g_left_pid.reset();
        g_right_pid.reset();
    }
    if (cmd.has_mode && g_mode != DriveMode::ESTOP) {
        g_mode = cmd.mode;
    }
    if (cmd.has_vel) {
        g_manual_cmd.linear_mps = cmd.linear_mps;
        g_manual_cmd.angular_rps = cmd.angular_rps;
    }
    if (cmd.has_waypoint) {
        g_nav.setWaypoint(cmd.wp_x_m, cmd.wp_y_m);
        if (g_mode != DriveMode::ESTOP) {
            g_mode = DriveMode::AUTONOMOUS;
        }
    }

    // 2. 20 Hz Sensor Scan + Occupancy Grid Map Update
    if ((now_ms - g_last_sensor_ms) >= (1000U / SENSOR_LOOP_HZ)) {
        g_last_sensor_ms = now_ms;
        const auto scan = g_tof.readAll();
        g_grid.updateFromScan(g_odom.pose(), scan);
        g_battery.sample();
        g_cam.probe(now_ms);
    }

    // 3. 50 Hz Safety Evaluation + Odometry + Wheel Velocity PID Loop
    if ((now_ms - g_last_control_ms) >= (1000U / CONTROL_LOOP_HZ)) {
        const float dt_s = static_cast<float>(now_ms - g_last_control_ms) * 0.001f;
        g_last_control_ms = now_ms;

        const auto imu_state = g_imu.update(dt_s);
        const auto pose = g_odom.update(dt_s, imu_state.healthy, imu_state.yaw_rate_rps);
        const auto fault = g_safety.evaluate(
            now_ms, g_mode, imu_state, g_battery.status(), g_motors);

        if (fault != SafetyFault::NONE || g_mode == DriveMode::IDLE) {
            g_left_pid.reset();
            g_right_pid.reset();
            g_motors.hardStop();
            g_avoidance_active = false;
            digitalWrite(pins::STATUS_LED, (fault != SafetyFault::NONE) ? HIGH : LOW);
        } else {
            motor_control::Twist2D target_twist{};
            if (g_mode == DriveMode::AUTONOMOUS) {
                const auto avoid = g_nav.computeControl(pose, g_tof.lastScan());
                target_twist = avoid.command;
                g_avoidance_active = avoid.obstacle_avoidance_active;
            } else if (g_mode == DriveMode::MANUAL) {
                target_twist = g_manual_cmd;
                g_avoidance_active = false;
            }

            const auto wheel_targets =
                g_kinematics.inverse(target_twist.linear_mps, target_twist.angular_rps);
            const float left_effort =
                g_left_pid.update(wheel_targets.left_rad_s, pose.left_rad_s, dt_s);
            const float right_effort =
                g_right_pid.update(wheel_targets.right_rad_s, pose.right_rad_s, dt_s);
            g_motors.setNormalizedEffort(left_effort, right_effort);
        }
    }

    // 4. 10 Hz WebSocket Telemetry Broadcast
    if ((now_ms - g_last_telemetry_ms) >= (1000U / TELEMETRY_LOOP_HZ)) {
        g_last_telemetry_ms = now_ms;

        communication::TelemetrySnapshot snap{};
        snap.timestamp_ms = now_ms;
        snap.mode = g_mode;
        snap.fault = g_safety.activeFault();
        snap.pose = g_odom.pose();
        snap.tof = g_tof.lastScan();
        snap.imu = g_imu.state();
        snap.battery = g_battery.status();
        snap.camera = g_cam.status();
        snap.left_rpm =
            motor_control::DiffDriveKinematics::radPerSecToRpm(snap.pose.left_rad_s);
        snap.right_rpm =
            motor_control::DiffDriveKinematics::radPerSecToRpm(snap.pose.right_rad_s);
        snap.watchdog_age_ms = g_safety.watchdogAgeMs(now_ms);
        snap.obstacle_avoidance_active = g_avoidance_active;

        g_ws.broadcastTelemetry(snap);
    }
}
