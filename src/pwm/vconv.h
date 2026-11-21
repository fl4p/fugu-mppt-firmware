#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "sim/vconv.h"
#include "pwm/mcpwm_timing.h"

// PwmDriver replacement that forwards LEDC-style update_pwm calls to the
// VirtualConverter singleton (g_vconv). Mirrors PWM_Mock / PWM_ESP32_ledc API.
// MCPWM-only members (setHsOff/setLsOff/forceShutdown/...) are NOT provided;
// MCPWM-dependent code in buck.h is excluded under #if WITH_MCPWM.
class PWM_VConv {
public:
    const char *name = "vconv";
    uint16_t pwmMax = 0;
    uint32_t resolutionHz = 0; // MCPWM timing: tick rate; 0 = LEDC timing

    PWM_VConv() = default;

    void init_pwm(int channel, int /*pin*/, uint32_t freq) {
        (void) channel;
        if (const uint32_t clk = g_vconv.pwmClk()) {
            // Mirror MCPWM_SyncLeg (HiLi, no dead-time): bestTiming period, realized frequency
            const PwmTiming t = bestTiming(freq, clk);
            if (pwmMax == 0) pwmMax = (uint16_t) t.period_ticks;
            resolutionHz = t.resolution_hz;
            freq = t.actual_freq;
        } else {
            // Mirror PWM_ESP32_ledc: pick the largest resolution such that the
            // APB-tick divider is >= 2^resolution, capped at 14 bits.
            uint32_t div = 80'000'000u / freq;
            int resolution = std::min((int) std::log2((double) div), 14);
            if (resolution < 1) resolution = 1;
            uint16_t pm = (uint16_t) ((2u << (resolution - 1)) - 1u);
            if (pwmMax == 0) pwmMax = pm;
        }
        g_vconv.setPwmMax(pwmMax);
        g_vconv.setPwmFreq(freq);
    }

    // LEDC: set duty (single-arg form). `duty` is the high-level DURATION.
    //   ch=0 (HS):     duty = pwmCtrl              -> HS on-count = duty
    //   ch=1 EnLogic:  duty = pwmCtrl + pwmRect    -> LS on-count = duty - pwmCtrl
    //   ch=1 HiLi reset: duty = 0                  -> LS on-count = 0
    // ch=1 stores the EN total and LS follows it on every HS write too, so either write order works.
    void update_pwm(int channel, uint32_t duty) {
        if (channel == 1) en_ = duty;
        else g_vconv.setPwmCount((uint8_t) channel, duty);
        const uint32_t hs = g_vconv.getPwm().pwmCtrl;
        if (channel <= 1 && en_ != kNoEn) g_vconv.setPwmCount(1, en_ > hs ? en_ - hs : 0);
    }

    // LEDC: set duty with hpoint (HiLi path). `duty` is the on-count directly;
    // hpoint just shifts where in the period the pulse sits and doesn't change
    // the duration. The plant places LS strictly after HS, so hpoint is ignored.
    void update_pwm(int channel, uint32_t /*hpoint*/, uint32_t duty) {
        if (channel == 1) en_ = kNoEn;
        g_vconv.setPwmCount((uint8_t) channel, duty);
    }

    void stop(int channel, uint8_t idleLevel) {
        if (channel == 1) en_ = kNoEn;
        g_vconv.stopChannel((uint8_t) channel);
        if (idleLevel) g_vconv.setPwmCount((uint8_t) channel, pwmMax);
    }

private:
    static constexpr uint32_t kNoEn = UINT32_MAX; // LS count set directly (HiLi / stopped)
    uint32_t en_ = kNoEn;
};
