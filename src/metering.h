#pragma once

#include <ctime>
#include <vector>
#include "store.h"
#include "math/float16.h"
#include "math/statmath.h"

struct DailyEnergyMeterState {
    float16 energyDay;

    inline bool hasEnergy() const { return !energyDay.isZero(); }
};


template<int N = 1000>
struct DailyRingStorageState {
    static constexpr int NumDays = N;
    uint16_t totalDays = 0;
    uint16_t ringPtr = 0;
    DailyEnergyMeterState ringBuf[NumDays];

    void clear() {
        memset((void *) this, 0, sizeof(*this));
    }

    void add(const DailyEnergyMeterState &day) {
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

    std::vector<DailyEnergyMeterState> getAllDays() {
        constexpr auto n = decltype(state)::NumDays;
        std::vector<DailyEnergyMeterState> vec{};
        for (int i = 0; i < n; ++i) {
            auto &d{state.ringBuf[(state.ringPtr + i) % n]};
            if (d.hasEnergy())
                vec.push_back(d);
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

    void add(const DailyEnergyMeterState &day) {
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
    DailyEnergyMeterState today{0};
    long timeLastPower = 0;

    void restore(float todayEnergy, long timeLastPower_) {
        store.load();
        timeLastPower = timeLastPower_;

        // maybe continue the day
        if (todayEnergy > 2) {
            today.energyDay = todayEnergy;
            auto now = std::time(nullptr);
            if (now < 1e9)
                ESP_LOGW("mppt", "No system time sync!");
            // restore day only if timestamps are valid and last power was within last 3h
            if (now > 1e9 and timeLastPower > 1e9 and now - timeLastPower < 3600 * 3) {
                ESP_LOGI("mppt", "Restored day energy %.2f, last power was <3h ago (%li s)", todayEnergy,
                         now - timeLastPower);
            } else {
                ESP_LOGI("mppt", "Store yesterday energy %.2f (last power %li s ago)", todayEnergy,
                         now - timeLastPower);
                store.add(today);
                today.energyDay = 0;
            }
        } else {
            today.energyDay = 0;
        }
        _prevTotalEnergy = 0;
    }

    void update(float smoothPower, float totalEnergy) {
        auto now = std::time(nullptr);

        if (smoothPower > powerDayEnd) {
            timeLastPower = now;
        }

        if ((today.hasEnergy() or smoothPower >= powerDayStart) and _prevTotalEnergy > 0 and
            totalEnergy > _prevTotalEnergy + 0.000001f) {
            float e = (totalEnergy - _prevTotalEnergy);
            if (!today.hasEnergy())
                ESP_LOGI("met", "Good Morning! First energy %.4f for today! This is day #%u", e,
                         store.getNumTotalDays() + 1);
            today.energyDay += e;
            // TODO fixw
            // assert(today.hasEnergy());

        } else if (today.hasEnergy() && (now - timeLastPower) > 60 * 30) {
            ESP_LOGI("met", "Day #%u ends, energy today %.3f, total %.2f", store.getNumTotalDays() + 1,
                     today.energyDay.toFloat(), totalEnergy);
            store.add(today);
            today.energyDay = 0;
        }

        _prevTotalEnergy = totalEnergy;
    }
};


struct PersistentState {
    uint32_t bootCount = 0;
    double totalEnergy = 0;
    std::time_t timeLastPower = 0;
    float todayEnergy = 0;
};


struct SolarEnergyMeter {
    TrapezoidalIntegrator<float, unsigned long, double> totalEnergy{
            1e-6f / 3600.f,  // us
            (unsigned long) 2e6f};

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
            ESP_LOGI("mppt", "Restored stats: totalEnergy=%.2f bootCounter=%u dailyEnergyMeter=%.2f", totalEnergy.get(),
                     stat.bootCount, stat.totalEnergy);
        }
        commit(true);
    }

    void add(float power, float smoothPower, unsigned long timeUs) {
        if (std::abs(power) > 0.1f)
            totalEnergy.add(power, timeUs);
        dailyEnergyMeter.update(smoothPower, (float) totalEnergy.get());
    }

    void update() {
        dailyEnergyMeter.update(0, (float) totalEnergy.get());
    }

    void commit(bool increaseBootCounter = false) {
        auto stats = flash.getFlashValue();
        stats.totalEnergy = totalEnergy.get();
        stats.timeLastPower = dailyEnergyMeter.timeLastPower;
        stats.todayEnergy = dailyEnergyMeter.today.energyDay.toFloat();
        if (increaseBootCounter)++stats.bootCount;
        flash.update(stats);
    }
};