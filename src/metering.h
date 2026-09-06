#pragma once

#include <atomic>
#include <ctime>
#include <vector>
#include "store.h"
#include "math/float16.h"
#include "math/statmath.h"
#include "util.h"

template<typename F=float16>
struct DailyEnergyMeterState {
    F energyYield;
    F vinMax;
    F pinMax;
    F voutMax;
    F voutMin;
    uint8_t tempMax;
    uint8_t usr;
    uint8_t numErrors;

    DailyEnergyMeterState() {
        reset();
    }


    template<typename F2>
    DailyEnergyMeterState(const DailyEnergyMeterState<F2> &o) :
            energyYield{o.energyYield}, vinMax{o.vinMax}, pinMax{o.pinMax}, voutMax{o.voutMax}, voutMin{o.voutMin},
            numErrors{o.numErrors} {}


    inline bool hasEnergy() const { return !energyYield.isZero(); }

    void reset() {
        //memset(this, 0 , sizeof(*this));
        energyYield = 0;
        vinMax = 0;
        pinMax = 0;
        voutMax = 0;
        voutMin = std::numeric_limits<F>::max();
        numErrors = 0;
    }
};


template<int N = 1000>
struct DailyRingStorageState {
    static constexpr int NumDays = N;
    uint16_t totalDays = 0;
    uint16_t ringPtr = 0;
    DailyEnergyMeterState<float16> ringBuf[NumDays];

    void clear() {
        memset((void *) this, 0, sizeof(*this));
    }

    void add(const DailyEnergyMeterState<float16> &day) {
        assert(day.hasEnergy());
        ringBuf[ringPtr] = day;
        ringPtr = (ringPtr + 1) % NumDays;
        ++totalDays;
    }
};

class DailyRingStorage {
public:
    DailyRingStorageState<60> state;
    FlashValueFile<decltype(state)> flash;

    explicit DailyRingStorage(const char *fn = "/littlefs/daily") : flash{fn} {}

    int getNumTotalDays() const { return state.totalDays; }

    std::vector<DailyEnergyMeterState<float>> getAllDays() {
        constexpr auto n = decltype(state)::NumDays;
        std::vector<DailyEnergyMeterState<float>> vec{};
        for (int i = 0; i < n; ++i) {
            auto &d{state.ringBuf[(state.ringPtr + i) % n]};
            if (d.hasEnergy())
                vec.emplace_back(DailyEnergyMeterState<float>(d));
        }
        return vec;
    }

    bool load() {
        if (flash.load(state, true)) {
            if (state.ringPtr >= decltype(state)::NumDays || state.ringPtr > state.totalDays) {
                ESP_LOGW("met", "Unexpected ringPtr");
                state.clear();
                return false;
            }
            return true;
        } else {
            state.clear();
            return false;
        }
    }

    void add(const DailyEnergyMeterState<float> &day) {
        state.add(day);
        flash.update(state);
    }
};


class DailyEnergyMeter {

    const float powerDayStart = 2.5f;
    const float powerDayEnd = 1.5f;

    DailyRingStorage store{};


    float _prevTotalEnergy = 0;

public:
    DailyEnergyMeterState<float> today;

    DailyEnergyMeter() {
        today.reset();
    }


    long timeLastPower = 0;

    void restore(float todayEnergy_, long timeLastPower_) {
        store.load();
        timeLastPower = timeLastPower_;

        // maybe continue the day
        if (todayEnergy_ > 2) {
            today.energyYield = todayEnergy_;
            time_t now;
            for (auto i = 0; i < 20; ++i) {
                now = std::time(nullptr);
                if (now > 1e9) break;
                if (i == 0) ESP_LOGI("meter", "waiting for time sync...");
                delay(500);
            }
            if (now < 1e9)
                ESP_LOGW("meter", "No system time sync!");
            // restore day only if timestamps are valid and last power was within last 3h
            if (now > 1e9 and timeLastPower > 1e9 and now - timeLastPower < 3600 * 3) {
                ESP_LOGI("meter", "Restored day energy %.2f, last power was <3h ago (%ld s)", todayEnergy_,
                         (long) (now - timeLastPower));
            } else {
                ESP_LOGI("mppt", "Store yesterday energy %.2f (last power %ld s ago)", todayEnergy_,
                         (long) (now - timeLastPower));
                store.add(today); // was "// TODO panic": the panic was the by-value ~14k
                                  // copy in FlashValueFile::update, now passed by const ref
                today.reset();
            }
        } else {
            today.reset();
        }
        _prevTotalEnergy = 0;
    }

    void update(float smoothPower, float totalEnergy, float vin = NAN, float vout = NAN) {
        auto now = std::time(nullptr);

        if (smoothPower > powerDayEnd) {
            timeLastPower = now;

            if (smoothPower > today.pinMax) today.pinMax = smoothPower;
            if (vin > today.vinMax) today.vinMax = vin;
            if (vout > today.voutMax) today.voutMax = vout;
            if (vout < today.voutMin) today.voutMin = vout;
        }


        if ((today.energyYield > 0 or smoothPower >= powerDayStart) and totalEnergy > _prevTotalEnergy + 1e-3f) {
            if (_prevTotalEnergy > 0) {
                float e = (totalEnergy - _prevTotalEnergy);
                if (today.energyYield == 0)
                    ESP_LOGI("met",
                             "First energy %.4f today, day #%u, total %.2f", e,
                             store.getNumTotalDays() + 1, totalEnergy);
                today.energyYield += e;
            }

            _prevTotalEnergy = totalEnergy;

        } else if (today.energyYield > 0 && (now - timeLastPower) > 60 * 30) {
            ESP_LOGI("met", "Day #%u ends, energy today %.3f, total %.2f", store.getNumTotalDays() + 1,
                     today.energyYield, totalEnergy);
            store.add(today);
            today.reset();
        }
    }
};


struct PersistentState {
    uint32_t bootCount = 0;
    double totalEnergy = 0;
    std::time_t timeLastPower = 0;
    float todayEnergy = 0;
};


struct SolarEnergyMeter {
    TrapezoidalIntegrator<float, time_us, float> totalEnergy{
            1e-6f / 3600.f,  // /us => /h
            /*maxDt*/static_cast<time_us>(4e6f) // 4sec
    };

    FlashValueStore<PersistentState> flash{
            "/littlefs/stats",
            [](const PersistentState &a, const PersistentState &b) {
                return std::abs(a.totalEnergy - b.totalEnergy) < 5
                       && a.bootCount == b.bootCount
                       && std::abs(a.todayEnergy - b.totalEnergy) < 5;
            },
            1000 * 60 * 2
    };

    DailyEnergyMeter dailyEnergyMeter;

    // RT -> core 0 handoff for update(). Relaxed: each is read independently and a
    // one-sample skew between them is below the day accumulator's resolution.
    std::atomic<float> rtPower{0}, rtVin{NAN}, rtVout{NAN};


    void load() {
        if (flash.load()) {
            auto &stat(flash.getFlashValue());
            totalEnergy.restore(stat.totalEnergy);
            dailyEnergyMeter.restore(stat.todayEnergy, stat.timeLastPower);
            ESP_LOGI("mppt", "Restored: totalE=%.2f boot=%lu dailyE=%.2f",
                     totalEnergy.get(),
                     stat.bootCount, stat.totalEnergy);
        }
        commit(true);
    }

    // RT path. dailyEnergyMeter.update() must NOT run here: it calls std::time() every sample and
    // its day-end branch writes flash. Stash the operating point instead and let update() below,
    // which runs on core 0, do the work.
    void add(float power, float smoothPower, float vin, float vout, time_us timeUs) {
        if (power > 0.1f)
            totalEnergy.add(power, timeUs);
        rtPower.store(smoothPower, std::memory_order_relaxed);
        rtVin.store(vin, std::memory_order_relaxed);
        rtVout.store(vout, std::memory_order_relaxed);
    }

    // Core 0 only. Passing 0 for smoothPower (as the old commented-out call did) cannot work: the
    // accumulator's first branch needs energyYield > 0 OR smoothPower >= powerDayStart, so with a
    // hardcoded 0 it never starts and E_today stays at its restored value forever.
    void update() {
        dailyEnergyMeter.update(rtPower.load(std::memory_order_relaxed),
                                (float) totalEnergy.get(),
                                rtVin.load(std::memory_order_relaxed),
                                rtVout.load(std::memory_order_relaxed));
    }

    void commit(bool increaseBootCounter = false) {
        auto stats = flash.getFlashValue();
        stats.totalEnergy = totalEnergy.get();
        stats.timeLastPower = dailyEnergyMeter.timeLastPower;
        stats.todayEnergy = dailyEnergyMeter.today.energyYield;
        if (increaseBootCounter)++stats.bootCount;
        flash.update(stats);
    }
};