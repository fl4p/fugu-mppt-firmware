#pragma once

#include <ctime>
#include <vector>
#include "store.h"
#include "math/float16.h"
#include "math/statmath.h"

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
    DailyRingStorageState<1000> state;
    FlashValueFile<decltype(state)> flash;

public:
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
                ESP_LOGI("meter", "Restored day energy %.2f, last power was <3h ago (%lli s)", todayEnergy_,
                         now - timeLastPower);
            } else {
                ESP_LOGI("mppt", "Store yesterday energy %.2f (last power %lli s ago)", todayEnergy_,
                         now - timeLastPower);
                //store.add(today); // TODO panic
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
                             "Good Day! First energy %.4f for today! This is day #%u, total energy meter is %.2f", e,
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
    TrapezoidalIntegrator<float, unsigned long, float> totalEnergy{
            1e-6f / 3600.f,  // /us => /h
            (unsigned long) 2e6f // 2sec
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


    void load() {
        if (flash.load()) {
            auto &stat(flash.getFlashValue());
            totalEnergy.restore(stat.totalEnergy);
            dailyEnergyMeter.restore(stat.todayEnergy, stat.timeLastPower);
            ESP_LOGI("mppt", "Restored stats: totalEnergy=%.2f bootCounter=%lu dailyEnergyMeter=%.2f",
                     totalEnergy.get(),
                     stat.bootCount, stat.totalEnergy);
        }
        commit(true);
    }

    void add(float power, float smoothPower, float vin, float vout, unsigned long timeUs) {
        if (power > 0.1f)
            totalEnergy.add(power, timeUs);
        //dailyEnergyMeter.update(smoothPower, totalEnergy.get(), vin, vout);
    }

    void update() {
        //dailyEnergyMeter.update(0, (float) totalEnergy.get());
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