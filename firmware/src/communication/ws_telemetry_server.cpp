#include "communication/ws_telemetry_server.h"

#include "safety/safety_supervisor.h"

#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>
#include <cstring>

namespace roversar::communication {

namespace {
constexpr const char* kWsMagicGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

const char* modeToString(DriveMode mode) {
    switch (mode) {
        case DriveMode::IDLE:
            return "IDLE";
        case DriveMode::MANUAL:
            return "MANUAL";
        case DriveMode::AUTONOMOUS:
            return "AUTONOMOUS";
        case DriveMode::ESTOP:
            return "ESTOP";
        default:
            return "IDLE";
    }
}
}  // namespace

void WsTelemetryServer::begin(uint16_t port) {
    server_ = WiFiServer(port);
    server_.begin();
    server_.setNoDelay(true);
}

void WsTelemetryServer::acceptNewClients() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    while (WiFiClient incoming = server_.accept()) {
#else
    while (WiFiClient incoming = server_.available()) {
#endif
        incoming.setNoDelay(true);
        String headers;
        const uint32_t start = millis();
        while (incoming.connected() && (millis() - start) < 80U) {
            if (incoming.available()) {
                headers += incoming.readStringUntil('\n');
                headers += '\n';
                if (headers.endsWith("\r\n\r\n") || headers.endsWith("\n\n")) {
                    break;
                }
            }
        }

        if (headers.indexOf("Upgrade: websocket") >= 0 ||
            headers.indexOf("Upgrade: WebSocket") >= 0) {
            if (performHandshake(incoming, headers)) {
                bool stored = false;
                for (auto& slot : ws_clients_) {
                    if (!slot.connected()) {
                        slot = incoming;
                        stored = true;
                        break;
                    }
                }
                if (!stored) {
                    ws_clients_[0].stop();
                    ws_clients_[0] = incoming;
                }
            } else {
                incoming.stop();
            }
        } else {
            const int nl = headers.indexOf('\n');
            const String first_line = (nl > 0) ? headers.substring(0, nl) : headers;
            handleHttpRequest(incoming, first_line);
            incoming.stop();
        }
    }
}

bool WsTelemetryServer::performHandshake(WiFiClient& client, const String& request_headers) {
    int key_pos = request_headers.indexOf("Sec-WebSocket-Key:");
    if (key_pos < 0) {
        key_pos = request_headers.indexOf("sec-websocket-key:");
    }
    if (key_pos < 0) {
        return false;
    }

    const int line_end = request_headers.indexOf('\n', key_pos);
    String client_key = request_headers.substring(key_pos + 18, line_end);
    client_key.trim();

    const String accept_key = computeWsAcceptKey(client_key);
    client.print(
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        accept_key + "\r\n\r\n");
    return true;
}

String WsTelemetryServer::computeWsAcceptKey(const String& client_key) {
    const String combined = client_key + kWsMagicGuid;
    unsigned char sha1_out[20]{};
    mbedtls_sha1(reinterpret_cast<const unsigned char*>(combined.c_str()),
                 combined.length(),
                 sha1_out);

    unsigned char b64_out[64]{};
    size_t out_len = 0;
    mbedtls_base64_encode(b64_out, sizeof(b64_out), &out_len, sha1_out, sizeof(sha1_out));
    b64_out[out_len] = '\0';
    return String(reinterpret_cast<const char*>(b64_out));
}

void WsTelemetryServer::handleHttpRequest(WiFiClient& client, const String& first_line) {
    if (first_line.indexOf("/health") >= 0) {
        client.print(
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
            "{\"status\":\"ok\",\"firmware\":\"" ROVERSAR_FIRMWARE_VERSION "\"}");
        return;
    }

    client.print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n");
    client.print(last_telemetry_json_);
}

InboundCommand WsTelemetryServer::poll() {
    acceptNewClients();

    InboundCommand cmd{};
    for (auto& client : ws_clients_) {
        if (!client.connected()) {
            continue;
        }
        while (client.available() >= 2) {
            String msg;
            if (readWsFrame(client, msg)) {
                parseCommandJson(msg, cmd);
            } else {
                break;
            }
        }
    }
    return cmd;
}

bool WsTelemetryServer::readWsFrame(WiFiClient& client, String& payload_out) {
    if (client.available() < 2) {
        return false;
    }

    const int b1 = client.read();
    const int b2 = client.read();
    if (b1 < 0 || b2 < 0) {
        return false;
    }

    const uint8_t opcode = static_cast<uint8_t>(b1 & 0x0F);
    const bool masked = (b2 & 0x80) != 0;
    uint64_t len = static_cast<uint64_t>(b2 & 0x7F);

    if (opcode == 0x8) {
        client.stop();
        return false;
    }

    if (len == 126) {
        if (client.available() < 2) return false;
        len = (static_cast<uint64_t>(client.read()) << 8) | static_cast<uint64_t>(client.read());
    } else if (len > 126) {
        client.stop();
        return false;
    }

    uint8_t mask[4]{0, 0, 0, 0};
    if (masked) {
        if (client.available() < 4) return false;
        for (int i = 0; i < 4; ++i) {
            mask[i] = static_cast<uint8_t>(client.read());
        }
    }

    payload_out.reserve(static_cast<unsigned int>(len));
    for (uint64_t i = 0; i < len; ++i) {
        const uint32_t wait_start = millis();
        while (!client.available() && (millis() - wait_start) < 20U) {
            delay(1);
        }
        if (!client.available()) {
            return false;
        }
        uint8_t byte_val = static_cast<uint8_t>(client.read());
        if (masked) {
            byte_val ^= mask[i % 4];
        }
        payload_out += static_cast<char>(byte_val);
    }

    return opcode == 0x1 || opcode == 0x2;
}

void WsTelemetryServer::sendWsText(WiFiClient& client, const String& json_text) {
    if (!client.connected()) {
        return;
    }
    const size_t len = json_text.length();
    uint8_t header[4]{};
    size_t hdr_len = 2;
    header[0] = 0x81;  // FIN + text frame
    if (len < 126U) {
        header[1] = static_cast<uint8_t>(len);
    } else {
        header[1] = 126U;
        header[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
        header[3] = static_cast<uint8_t>(len & 0xFF);
        hdr_len = 4;
    }
    client.write(header, hdr_len);
    client.write(reinterpret_cast<const uint8_t*>(json_text.c_str()), len);
}

void WsTelemetryServer::broadcastTelemetry(const TelemetrySnapshot& snap) {
    char buf[512];
    snprintf(
        buf,
        sizeof(buf),
        "{\"t_ms\":%lu,\"mode\":\"%s\",\"fault\":\"%s\","
        "\"pose\":{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f,\"v\":%.3f,\"w\":%.3f},"
        "\"rpm\":{\"left\":%.1f,\"right\":%.1f},"
        "\"tof_m\":[%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"imu\":{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.3f},"
        "\"battery\":{\"v\":%.2f,\"soc\":%.1f},"
        "\"watchdog_ms\":%lu,\"avoidance\":%s,"
        "\"camera\":{\"online\":%s,\"tilt\":%u,\"url\":\"%s\"}}",
        static_cast<unsigned long>(snap.timestamp_ms),
        modeToString(snap.mode),
        safety::SafetySupervisor::faultToString(snap.fault),
        snap.pose.x_m,
        snap.pose.y_m,
        snap.pose.theta_rad,
        snap.pose.linear_mps,
        snap.pose.angular_rps,
        snap.left_rpm,
        snap.right_rpm,
        snap.tof.ranges_m[0],
        snap.tof.ranges_m[1],
        snap.tof.ranges_m[2],
        snap.tof.ranges_m[3],
        snap.tof.ranges_m[4],
        snap.imu.roll_deg,
        snap.imu.pitch_deg,
        snap.imu.yaw_rad,
        snap.battery.voltage_v,
        snap.battery.soc_percent,
        static_cast<unsigned long>(snap.watchdog_age_ms),
        snap.obstacle_avoidance_active ? "true" : "false",
        snap.camera.online ? "true" : "false",
        snap.camera.tilt_deg,
        snap.camera.stream_url.c_str());

    last_telemetry_json_ = String(buf);
    for (auto& client : ws_clients_) {
        if (client.connected()) {
            sendWsText(client, last_telemetry_json_);
        }
    }
}

void WsTelemetryServer::parseCommandJson(const String& json, InboundCommand& cmd) {
    cmd.has_heartbeat = true;

    String type;
    if (extractJsonString(json, "\"type\"", type)) {
        if (type == "estop") {
            cmd.trigger_estop = true;
            return;
        }
        if (type == "clear_estop") {
            cmd.clear_estop = true;
            return;
        }
        if (type == "waypoint") {
            float wx = 0.0f;
            float wy = 0.0f;
            if (extractJsonFloat(json, "\"x\"", wx) && extractJsonFloat(json, "\"y\"", wy)) {
                cmd.has_waypoint = true;
                cmd.wp_x_m = wx;
                cmd.wp_y_m = wy;
            }
        }
    }

    String mode_str;
    if (extractJsonString(json, "\"mode\"", mode_str)) {
        cmd.has_mode = true;
        if (mode_str == "MANUAL") {
            cmd.mode = DriveMode::MANUAL;
        } else if (mode_str == "AUTONOMOUS") {
            cmd.mode = DriveMode::AUTONOMOUS;
        } else if (mode_str == "ESTOP") {
            cmd.mode = DriveMode::ESTOP;
        } else {
            cmd.mode = DriveMode::IDLE;
        }
    }

    float v = 0.0f;
    float w = 0.0f;
    if (extractJsonFloat(json, "\"linear\"", v) || extractJsonFloat(json, "\"v\"", v)) {
        cmd.has_vel = true;
        cmd.linear_mps = v;
    }
    if (extractJsonFloat(json, "\"angular\"", w) || extractJsonFloat(json, "\"w\"", w)) {
        cmd.has_vel = true;
        cmd.angular_rps = w;
    }
}

bool WsTelemetryServer::extractJsonFloat(const String& json, const char* key, float& out) {
    const int kpos = json.indexOf(key);
    if (kpos < 0) return false;
    const int colon = json.indexOf(':', kpos);
    if (colon < 0) return false;
    int end = colon + 1;
    while (end < static_cast<int>(json.length()) &&
           (json[end] == ' ' || json[end] == '\t')) {
        ++end;
    }
    if (end >= static_cast<int>(json.length())) return false;
    out = json.substring(end).toFloat();
    return true;
}

bool WsTelemetryServer::extractJsonString(const String& json, const char* key, String& out) {
    const int kpos = json.indexOf(key);
    if (kpos < 0) return false;
    const int colon = json.indexOf(':', kpos);
    if (colon < 0) return false;
    const int q1 = json.indexOf('"', colon + 1);
    if (q1 < 0) return false;
    const int q2 = json.indexOf('"', q1 + 1);
    if (q2 < 0) return false;
    out = json.substring(q1 + 1, q2);
    return true;
}

}  // namespace roversar::communication
