#include "safety/watchdog.h"

namespace roversar::safety {

CommWatchdog::CommWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

void CommWatchdog::reset(uint32_t now_ms) {
    last_feed_ms_ = now_ms;
    armed_ = true;
}

void CommWatchdog::feed(uint32_t now_ms) {
    last_feed_ms_ = now_ms;
    armed_ = true;
}

bool CommWatchdog::isExpired(uint32_t now_ms) const {
    if (!armed_) {
        return true;
    }
    return (now_ms - last_feed_ms_) >= timeout_ms_;
}

uint32_t CommWatchdog::elapsedMs(uint32_t now_ms) const {
    if (!armed_) {
        return timeout_ms_;
    }
    return now_ms - last_feed_ms_;
}

}  // namespace roversar::safety
