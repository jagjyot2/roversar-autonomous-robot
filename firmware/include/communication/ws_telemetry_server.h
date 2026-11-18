#pragma once

#include "camera/esp32_cam_link.h"
#include "roversar_config.h"
#include "sensors/battery_monitor.h"
#include "sensors/encoder_odometry.h"
#include "sensors/mpu6050_imu.h"
#include "sensors/vl53l0x_array.h"

#include <WiFi.h>
#include <array>

namespace roversar::communication {

struct InboundCommand {
    bool has_heartbeat{false};
    bool has_vel{false};
    float linear_mps{0.0f};
    float angular_rps{0.0f};

    bool has_mode{false};
    DriveMode mode{DriveMode::IDLE};

    bool has_waypoint{false};
    float wp_x_m{0.0f};
    float wp_y_m{0.0f};

    bool trigger_estop{false};
    bool clear_estop{false};
};

struct TelemetrySnapshot {
    uint32_t timestamp_ms{0};
    DriveMode mode{DriveMode::IDLE};
    SafetyFault fault{SafetyFault::NONE};
    sensors::Pose2D pose{};
    sensors::TofScan tof{};
    sensors::ImuState imu{};
    sensors::BatteryStatus battery{};
    camera::CameraStatus camera{};
    float left_rpm{0.0f};
    float right_rpm{0.0f};
    uint32_t watchdog_age_ms{0};
    bool obstacle_avoidance_active{false};
};

/**
 * @brief Lightweight RFC 6455 WebSocket + HTTP JSON telemetry & control server.
 */
class WsTelemetryServer {
public:
    static constexpr size_t kMaxClients = 4;

    void begin(uint16_t port = WS_SERVER_PORT);
    InboundCommand poll();
    void broadcastTelemetry(const TelemetrySnapshot& snap);

private:
    void acceptNewClients();
    bool performHandshake(WiFiClient& client, const String& request_headers);
    void handleHttpRequest(WiFiClient& client, const String& first_line);
    bool readWsFrame(WiFiClient& client, String& payload_out);
    void sendWsText(WiFiClient& client, const String& json_text);
    void parseCommandJson(const String& json, InboundCommand& cmd);

    static bool extractJsonFloat(const String& json, const char* key, float& out);
    static bool extractJsonString(const String& json, const char* key, String& out);
    static String computeWsAcceptKey(const String& client_key);

    WiFiServer server_{WS_SERVER_PORT};
    std::array<WiFiClient, kMaxClients> ws_clients_{};
    String last_telemetry_json_{"{}"};
};

}  // namespace roversar::communication
