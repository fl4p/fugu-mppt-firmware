#pragma once

#include <cstdint>
#include <cmath>

#include "adc.h"
#include "conf.h"
#include "etc/rt.h"
#include "sim/vconv.h"
#include "tele/scope.h"

// AsyncADC backend that reads from the VirtualConverter singleton (g_vconv).
// Channel mapping (fixed): 0=Vin, 1=Vout, 2=Iout, 3=Iin, 4=NTC. Profiles usually leave Iin a
// VirtualSensor (computed from the other side at a fixed efficiency).
//
// On every periodic timer tick we step the plant by (1/adc_freq) seconds and
// notify the consumer task. Channels are read with readMode=SnapshotAllChannels, mirroring
// ADC_Fake. Optional per-channel zero-mean Gaussian noise is added in
// getSample (Box-Muller on xorshift32).
class ADC_VConv;
#ifdef FUGU_HOST_SIM
extern float (*g_simSenseHook)(uint8_t ch, float x); // host sim: measured sensor noise
extern bool g_simAdcMute; // host sim: the plant runs, no sample is delivered
#endif

static bool adc_vconv_periodic_timer_callback(void *arg);

class ADC_VConv : public AsyncADC<float> {
public:
    [[nodiscard]] AdcReadMode readMode() const override {
        return AdcReadMode::SnapshotAllChannels;
    }

    bool init(const ConfFile &boardConf) override {
        // Local vconv.conf: ADC tick rate + noise σ per channel.
        ConfFile vconvConf{"/littlefs/conf/vconv.conf"};
        adcFreq_ = (uint32_t) vconvConf.getLong("adc_freq",
                       boardConf.getLong("adc_freq", 3000));
        noiseVin_  = vconvConf.f("noise_vin",  0.0f);
        noiseVout_ = vconvConf.f("noise_vout", 0.0f);
        noiseIout_ = vconvConf.f("noise_iout", 0.0f);
        noiseIin_  = vconvConf.f("noise_iin",  0.0f);
        noiseNtc_  = vconvConf.f("noise_ntc",  0.0f);
        g_vconv.ntcV = vconvConf.f("ntc_v", g_vconv.ntcV);

        rngState_ = 0x9E3779B9u; // deterministic seed -> tests stay reproducible
#ifdef FUGU_HOST_SIM
        if (const char *e = getenv("SIM_SEED")) rngState_ = (0x9E3779B9u ^ ((uint32_t) atol(e) * 0x85EBCA6Bu)) | 1u;
#endif
        periodicTimer_.begin(adcFreq_, &adc_vconv_periodic_timer_callback, this);
        periodicTimer_.start();
        return true;
    }

    void deinit() override {
        periodicTimer_.destroy();
    }

    float getSamplingRate(uint8_t /*channel*/) override {
        return (float) periodicTimer_.freq();
    }

    void startReading(uint8_t channel) override {
        taskNotification_.subscribe();
        readingChannel_ = channel;
    }

    bool hasData() override {
        uint32_t ticks = taskNotification_.waitCount(10);
        if (ticks) {
            // Step the plant by however many ADC ticks coalesced, not just one, so sim-time tracks
            // wall-clock-ticks even when the control loop overruns and notifications pile up. This
            // keeps the plant clock in sync with any wall-clock-driven controller logic (capped so a
            // gross stall can't become a multi-ms catch-up spike).
            if (ticks > kMaxCatchupTicks) ticks = kMaxCatchupTicks;
            const float dt = 1e-6f * (float) (1000000u / std::max<uint32_t>(adcFreq_, 1)); // timer period, whole us
            g_vconv.stepSeconds(dt * (float) ticks, 39000);
        }
#ifdef FUGU_HOST_SIM
        if (g_simAdcMute) return false;
#endif
        return ticks != 0;
    }

    void setMaxExpectedVoltage(uint8_t, float) override {}

    float getSample() override {
        float x;
        float sigma = 0.0f;
        switch (readingChannel_) {
            case 0: x = g_vconv.getVin();     sigma = noiseVin_;  break;
            case 1: x = g_vconv.getVoutSense();sigma = noiseVout_; break;
            case 2: x = g_vconv.getIoutSense();sigma = noiseIout_; break;
            case 3: x = g_vconv.getIinAvg();  sigma = noiseIin_;  break;
            case 4: x = g_vconv.ntcV;         sigma = noiseNtc_;  break;
            default: x = 0.0f; break;
        }
        if (sigma > 0.0f) x += sigma * gaussian01();
#ifdef FUGU_HOST_SIM
        if (g_simSenseHook) x = g_simSenseHook(readingChannel_, x);
#endif
        if (readingChannel_ == 2 && g_vconv.iUnidir && !(x > 0.0f)) x = 0.0f;
        if (scope) scope->addSample12(this, readingChannel_, x / 3.0f * 4000.0f);
        return x;
    }

    float getInputImpedance(uint8_t) override { return 1e9f; } // ideal voltage divider

    [[nodiscard]] bool bipolar(uint8_t ch) const override { return ch != 2 || !g_vconv.iUnidir; }

    void reset(uint8_t) override {}

    bool periodicTimerCallback() {
        return taskNotification_.notifyFromIsr();
    }

private:
    static constexpr uint32_t kMaxCatchupTicks = 8; // bound the per-call plant catch-up after a stall
    uint8_t readingChannel_ = 0;
    uint32_t adcFreq_ = 3000;
    float noiseVin_ = 0.0f, noiseVout_ = 0.0f, noiseIout_ = 0.0f, noiseIin_ = 0.0f, noiseNtc_ = 0.0f;
    TaskNotification taskNotification_{};
    PeriodicTimer periodicTimer_{};

    // xorshift32 PRNG. Single-task consumer (RT core), no locking needed.
    uint32_t rngState_ = 0x9E3779B9u;
    inline uint32_t xorshift32() {
        uint32_t x = rngState_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        rngState_ = x;
        return x;
    }
    // Box-Muller, one sample per call (caches partner).
    bool haveSpare_ = false;
    float spare_ = 0.0f;
    inline float gaussian01() {
        if (haveSpare_) { haveSpare_ = false; return spare_; }
        // u1 in (0, 1] to keep log finite
        float u1, u2;
        do {
            u1 = (float) (xorshift32() >> 8) * (1.0f / 16777216.0f);
        } while (u1 <= 0.0f);
        u2 = (float) (xorshift32() >> 8) * (1.0f / 16777216.0f);
        float r = std::sqrt(-2.0f * std::log(u1));
        float t = 6.28318530717958647692f * u2;
        spare_ = r * std::sin(t);
        haveSpare_ = true;
        return r * std::cos(t);
    }
};

static IRAM_ATTR bool adc_vconv_periodic_timer_callback(void *arg) {
    auto adc = static_cast<ADC_VConv *>(arg);
    return adc->periodicTimerCallback();
}
