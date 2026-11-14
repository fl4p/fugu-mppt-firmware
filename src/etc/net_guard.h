#pragma once

#include <cstdint>

// Network watchdog decisions, stepped about once per second. Pure logic: the caller samples the
// inputs and carries out the returned actions. Times are uint32 ms; differences survive the wrap.
struct NetGuardIn {
    uint32_t nowMs;
    uint32_t freeHeap;  // bytes, kHeapUnknown if it could not be read
    uint32_t largest;   // largest free internal block, bytes, kHeapUnknown if not sampled
    bool wifiUp;        // associated and not switched off
    bool mqttActive;    // enabled, broker configured, not Stopped
    bool mqttRunning;   // state Running
    bool mqttConnected;
    bool rebootOk;      // uptime and no OTA or flash operation in progress
};

enum NetGuardAct : uint8_t { NG_RETRY_FAILED = 1, NG_RESTART_MQTT = 2, NG_REBOOT = 4 };

class NetGuard {
public:
    static constexpr uint32_t kHeapUnknown = UINT32_MAX;
    static constexpr uint32_t kRetryMs = 30000;        // Failed network services
    static constexpr uint32_t kMqttStaleMs = 120000;   // disconnected -> restart mqtt, also its retry period
    static constexpr uint32_t kRebootMs = 600000;      // disconnected + low heap -> reboot
    static constexpr uint32_t kHeapLow = 12 * 1024;
    static constexpr uint32_t kBlockLow = 9 * 1024;    // the mqtt task stack needs 8 KB in one block
    static constexpr uint32_t kHeapLowHoldMs = 60000;  // heap must stay low this long

    // Bit set of NetGuardAct. NG_REBOOT re-arms only after another kRebootMs.
    uint8_t step(const NetGuardIn &in) {
        const uint32_t now = in.nowMs;
        if (!_primed) {
            _primed = true;
            _retryAt = _okAt = _restartAt = _heapOkAt = now;
        }
        uint8_t a = 0;

        if (!in.wifiUp) _retryAt = now;
        else if (now - _retryAt >= kRetryMs) {
            _retryAt = now;
            a |= NG_RETRY_FAILED;
        }

        const bool low = (in.freeHeap != kHeapUnknown && in.freeHeap < kHeapLow) ||
                         (in.largest != kHeapUnknown && in.largest < kBlockLow);
        if (!low) _heapOkAt = now;

        if (!in.wifiUp || !in.mqttActive || in.mqttConnected) _okAt = now;
        const uint32_t down = now - _okAt;

        if (in.mqttRunning && down > kMqttStaleMs && now - _restartAt > kMqttStaleMs) {
            _restartAt = now;
            a |= NG_RESTART_MQTT;
        }

        if (in.rebootOk && down > kRebootMs && now - _heapOkAt >= kHeapLowHoldMs) {
            _okAt = now;
            a |= NG_REBOOT;
        }
        return a;
    }

    uint32_t downMs(uint32_t now) const { return now - _okAt; }

private:
    uint32_t _retryAt = 0, _okAt = 0, _restartAt = 0, _heapOkAt = 0;
    bool _primed = false;
};

// Lets the first `burst` events through, then at most one per `periodMs`.
class LogLimiter {
public:
    constexpr LogLimiter(uint8_t burst, uint32_t periodMs) : _burst(burst), _periodMs(periodMs) {}

    bool allow(uint32_t now) {
        if (_n < _burst || now - _last >= _periodMs) {
            if (_n < _burst) ++_n;
            _last = now;
            return true;
        }
        return false;
    }

private:
    uint8_t _burst, _n = 0;
    uint32_t _periodMs, _last = 0;
};
