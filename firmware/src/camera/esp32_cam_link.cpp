#include "camera/esp32_cam_link.h"

#include <WiFiClient.h>
#include <algorithm>

namespace roversar::camera {

void Esp32CamLink::begin(const char* stream_url) {
    status_.stream_url = String(stream_url);
    status_.tilt_deg = 15;
    status_.online = false;
    status_.last_probe_ms = 0;
}

void Esp32CamLink::setTiltAngleDeg(uint8_t deg) {
    status_.tilt_deg = std::clamp<uint8_t>(deg, 0, 60);
}

void Esp32CamLink::probe(uint32_t now_ms) {
    if ((now_ms - status_.last_probe_ms) < kProbeIntervalMs) {
        return;
    }
    status_.last_probe_ms = now_ms;

    // Extract host and port from http://host:port/stream without blocking control loop.
    String host = "192.168.4.2";
    uint16_t port = 81;

    int scheme_idx = status_.stream_url.indexOf("://");
    if (scheme_idx >= 0) {
        String rest = status_.stream_url.substring(scheme_idx + 3);
        int slash_idx = rest.indexOf('/');
        String host_port = (slash_idx >= 0) ? rest.substring(0, slash_idx) : rest;
        int colon_idx = host_port.indexOf(':');
        if (colon_idx >= 0) {
            host = host_port.substring(0, colon_idx);
            port = static_cast<uint16_t>(host_port.substring(colon_idx + 1).toInt());
        } else {
            host = host_port;
            port = 80;
        }
    }

    WiFiClient probe_client;
    probe_client.setTimeout(50);
    status_.online = probe_client.connect(host.c_str(), port, 40);
    if (status_.online) {
        probe_client.stop();
    }
}

}  // namespace roversar::camera
