#pragma once

#include "roversar_config.h"
#include <Arduino.h>

namespace roversar::camera {

struct CameraStatus {
    bool online{false};
    uint8_t tilt_deg{15};
    String stream_url{DEFAULT_CAM_URL};
    uint32_t last_probe_ms{0};
};

/**
 * @brief Coordinates the companion ESP32-CAM MJPEG stream node over Wi-Fi/UART.
 */
class Esp32CamLink {
public:
    void begin(const char* stream_url = DEFAULT_CAM_URL);
    void probe(uint32_t now_ms);
    void setTiltAngleDeg(uint8_t deg);

    const CameraStatus& status() const { return status_; }

private:
    CameraStatus status_{};
    static constexpr uint32_t kProbeIntervalMs = 5000U;
};

}  // namespace roversar::camera
