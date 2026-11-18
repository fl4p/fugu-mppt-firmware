#pragma once
// Idle (night) power profile: lower CPU clock, slower connectable BLE advert and telemetry point rate
// after 5 min with the converter and every listed activity off; restored on the first busy tick.
// While the CPU runs below its boot clock the converter start is inhibited; a refused start makes
// the profile leave, and the start follows once the RT loop has settled at the boot clock.

#include <cmath>
#include <cstdint>

constexpr uint32_t kTeleMinPointUs = 20'000;
extern uint32_t g_teleMinPointUs; // min interval between telemetry points
// sparse points (>= 1 s apart) carry every periodic field and leave without batching
inline bool teleSparse() { return g_teleMinPointUs >= 1'000'000; }

void idleProfileSuspend();       // leave and stop the profile until reboot (manual `pwr` knobs)
void idleProfileHold(bool hold); // leave now and stay out while held (blocking HTTP OTA)
bool idleProfileOn();
const char *idleProfileForce(const char *arg); // `pwr idle 0|1|auto`; nullptr or the refusal
const char *idleProfileMode();                 // auto, off, on (forced) or susp
const char *idleCpuState();                    // CPU part: boot pre test low settle latched -
void idleProfileAfter(uint32_t s);             // `pwr idle-after`: idle time before the profile, s
uint32_t idleProfileAfter();

// CPU-part guard: the RT sample rate at the idle clock against the rate measured just before
constexpr int64_t kIdleRateWinUs = 3'000'000, kIdleSkipUs = 500'000;
constexpr float kIdleMinRateRatio = 0.97f;
// after the boot clock is back: min time and samples before a start is released
constexpr int64_t kIdleSettleUs = 200'000;
constexpr uint32_t kIdleSettleN = 128;

// samples/s over a window; NAN when not evaluable (no time, counter went back, disturbed)
inline float idleWinRate(uint32_t n0, uint32_t n1, int64_t dtUs, bool disturbed) {
    if (dtUs <= 0 || disturbed || (int32_t) (n1 - n0) < 0) return NAN;
    return (float) (n1 - n0) * 1e6f / (float) dtUs;
}

// Network-loop backstop for the RT wake: true once a start has been refused only by InhibitIdle for
// more than kIdleBackstopMs. since: start of that state + 1 ms, 0 = not in it.
constexpr uint32_t kIdleBackstopMs = 1000;
inline bool idleBackstopWake(bool idleOnly, uint32_t nowMs, uint32_t &since) {
    if (!idleOnly) return since = 0, false;
    if (!since) since = nowMs + 1;
    return nowMs + 1 - since > kIdleBackstopMs;
}

enum class IdleCpuVerdict : uint8_t { Keep, Restore, NoBase };

// NoBase: no usable rate before the switch (do not lower). Keep only for a finite idle rate of at
// least minRatio * pre; anything else, NAN included, restores.
inline IdleCpuVerdict idleCpuVerdict(float pre, float idle, float minRatio = kIdleMinRateRatio) {
    if (!std::isfinite(pre) || !(pre > 0)) return IdleCpuVerdict::NoBase;
    return std::isfinite(idle) && idle >= minRatio * pre ? IdleCpuVerdict::Keep : IdleCpuVerdict::Restore;
}
