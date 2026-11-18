#pragma once

#include <cstdint>

namespace roversar::safety {

/**
 * @brief Deterministic 500ms communication-loss heartbeat watchdog.
 */
class CommWatchdog {
public:
    explicit CommWatchdog(uint32_t timeout_ms = 500U);

    void reset(uint32_t now_ms);
    void feed(uint32_t now_ms);
    bool isExpired(uint32_t now_ms) const;
    uint32_t elapsedMs(uint32_t now_ms) const;
    uint32_t timeoutMs() const { return timeout_ms_; }

private:
    uint32_t timeout_ms_;
    uint32_t last_feed_ms_{0};
    bool armed_{false};
};

}  // namespace roversar::safety
