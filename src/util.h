#pragma once

#include <cstdint>

#define RT_CORE 1
#define NON_RT_CORE 0

using time_us = uint64_t; // monotonic microseconds, sourced from esp_timer_get_time()
using time_ms = uint64_t; // monotonic milliseconds, derived from wallClockUs()

extern time_us loopWallClockUs_;
extern volatile uint32_t loopWallTicks_;

inline const time_us &wallClockUs() { return loopWallClockUs_; }

inline time_ms wallClockMs() { return loopWallClockUs_ / 1000ULL; }

// 2^20 us (~1.05 s) ticks for cross-core freshness stamps: a 32-bit load/store is atomic on Xtensa,
// and a 32-bit tick count wraps only after ~142 years, so a stale stamp can never read fresh again.
inline uint32_t coarseTicks(time_us us) { return (uint32_t) (us >> 20); }
constexpr uint32_t secToCoarseTicks(uint32_t s) { return (uint32_t) (((uint64_t) s * 1000000ULL) >> 20); }
inline uint32_t coarseTicksToSec(uint32_t t) { return (uint32_t) (((uint64_t) t << 20) / 1000000ULL); }

inline void setWallClockUs(time_us us) {
    loopWallClockUs_ = us;
    loopWallTicks_ = coarseTicks(us);
}

// Tear-free on core 0, unlike a 64-bit read of loopWallClockUs_. Still 0 in tests and the first ~1 s
// of uptime, where the high word is 0 or not written concurrently, so derive it from the us clock.
inline uint32_t wallClockTicks() {
    uint32_t t = loopWallTicks_;
    return t ? t : coarseTicks(loopWallClockUs_);
}


void scan_i2c();

void assertPinState(uint8_t pin, bool digitalVal, const char *pinName = nullptr, bool weakBackPull = false);

#define assert_throw(cond, msg) do { if(!(cond)) throw std::runtime_error(msg " (" #cond ") is false"); } while(0)

#define ESP_ERROR_CHECK_THROW(x) do {                                               \
        esp_err_t err_rc_ = (x);                                                    \
        if (unlikely(err_rc_ != ESP_OK)) {                                          \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__, __LINE__,      \
                                    __ASSERT_FUNC, #x);                             \
            throw std::runtime_error(#x);                                           \
            }                                                                       \
    } while(0)


template<typename T>
T absdiff(const T &lhs, const T &rhs) {
    return lhs > rhs ? lhs - rhs : rhs - lhs;
}


float strntof(const char *dat, int len);