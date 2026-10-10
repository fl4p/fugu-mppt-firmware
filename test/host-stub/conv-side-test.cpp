// Host test for src/conv_side.h: HV/LV side keys in sensor.conf / limits.conf map to roles by topo,
// and a file that mixes side and role keys is rejected.
//
// Build & run:
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
//       -o /tmp/conv-side-test test/host-stub/conv-side-test.cpp && /tmp/conv-side-test

#include <cstdio>
#include <stdexcept>
#include <string>

#include "conv_side.h"

static int failures = 0;
#define EXPECT(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

// Message of the runtime_error f() throws, or "" if it does not throw.
template<class F>
static std::string thrown(F f) {
    try { f(); } catch (const std::runtime_error &e) { return e.what(); }
    return "";
}

static bool contains(const std::string &s, const char *sub) { return s.find(sub) != std::string::npos; }

// Reading the sensor code computes for raw value x: (x - midpoint) * factor, as LinearTransform::apply.
static float currentReading(const ConfFile &s, const std::string &chn, bool boost, float x) {
    const bool side = sensorUsesSideKeys(s);
    const auto k = sensorKeyPrefix(chn, side, boost);
    const float factor = sensorCurrentFactor(s.getFloat(k + "_factor", 1.f), side, boost);
    return (x - s.getFloat(k + "_midpoint", 0.f)) * factor;
}

int main() {
    // sides by topo: buck in=HV out=LV, boost in=LV out=HV
    EXPECT(std::string(sideOf(true, false)) == "hv");
    EXPECT(std::string(sideOf(false, false)) == "lv");
    EXPECT(std::string(sideOf(true, true)) == "lv");
    EXPECT(std::string(sideOf(false, true)) == "hv");

    // sensor.conf: side-only config resolves per topo
    ConfFile side{{"hv_v_adc", "esp32adc1"}, {"hv_v_ch", "3"}, {"lv_v_ch", "0"}, {"lv_i_ch", "1"}};
    EXPECT(sensorUsesSideKeys(side));
    EXPECT(sensorKeyPrefix("vin", true, false) == "hv_v");
    EXPECT(sensorKeyPrefix("vout", true, false) == "lv_v");
    EXPECT(sensorKeyPrefix("iin", true, false) == "hv_i");
    EXPECT(sensorKeyPrefix("iout", true, false) == "lv_i");
    EXPECT(sensorKeyPrefix("vin", true, true) == "lv_v");
    EXPECT(sensorKeyPrefix("vout", true, true) == "hv_v");
    EXPECT(sensorKeyPrefix("iin", true, true) == "lv_i");
    EXPECT(sensorKeyPrefix("iout", true, true) == "hv_i");
    EXPECT(sensorKeyPrefix("ntc", true, true) == "ntc");

    // legacy role-only config is read by role name in either topo
    ConfFile role{{"vin_ch", "3"}, {"vout_ch", "0"}, {"iout_factor", "-1"}};
    EXPECT(!sensorUsesSideKeys(role));
    for (bool boost: {false, true})
        for (auto c: {"vin", "vout", "iin", "iout"})
            EXPECT(sensorKeyPrefix(c, false, boost) == c);

    // all-or-nothing: any side channel key next to any role channel key throws, naming one of each,
    // whether the two land on the same channel or not (the reviewer's probe: side voltages plus a
    // legacy iout current)
    ConfFile probe{{"hv_v_ch", "3"}, {"lv_v_ch", "0"}, {"iout_ch", "1"}, {"iout_factor", "-1"}};
    const auto msg = thrown([&] { sensorUsesSideKeys(probe); });
    EXPECT(contains(msg, "sensor.conf"));
    EXPECT(contains(msg, "hv_v_ch"));
    EXPECT(contains(msg, "iout_ch"));
    EXPECT(!thrown([&] { sensorUsesSideKeys(ConfFile{{"vin_adc", "esp32adc1"}, {"hv_v_ch", "3"}}); }).empty());
    EXPECT(!thrown([&] { sensorUsesSideKeys(ConfFile{{"vin_ch", "3"}, {"lv_i_filt_len", "20"}}); }).empty());

    // keys outside the channel suffix set do not count (legacy iin_min_supply_voltage, ntc, adc)
    ConfFile legacy{{"iin_min_supply_voltage", "6"}, {"hv_i_ch", "2"}, {"ntc_ch", "7"}, {"adc", "ina226"}};
    EXPECT(sensorUsesSideKeys(legacy));

    // current factor: side keys use the buck direction, boost negates; role keys are never touched.
    // With a midpoint the raw zero stays put and only the sign of the reading flips.
    EXPECT(sensorCurrentFactor(-1.f, true, false) == -1.f);
    EXPECT(sensorCurrentFactor(-1.f, true, true) == 1.f);
    EXPECT(sensorCurrentFactor(-1.f, false, true) == -1.f);
    ConfFile lvShunt{{"lv_i_ch", "1"}, {"lv_i_factor", "-2"}, {"lv_i_midpoint", "2.5"}};
    ConfFile boostRole{{"iin_ch", "1"}, {"iin_factor", "2"}, {"iin_midpoint", "2.5"}};
    ConfFile buckRole{{"iout_ch", "1"}, {"iout_factor", "-2"}, {"iout_midpoint", "2.5"}};
    for (float x: {0.f, 2.5f, 3.f}) {
        EXPECT(currentReading(lvShunt, "iin", true, x) == currentReading(boostRole, "iin", true, x));
        EXPECT(currentReading(lvShunt, "iout", false, x) == currentReading(buckRole, "iout", false, x));
        EXPECT(currentReading(lvShunt, "iin", true, x) == -currentReading(lvShunt, "iout", false, x));
    }
    EXPECT(currentReading(lvShunt, "iin", true, 3.f) == 1.f);

    // limits.conf: role <-> side mapping
    EXPECT(limitSideKey("vin_max", false) == "hv_max");
    EXPECT(limitSideKey("vout_max", false) == "lv_max");
    EXPECT(limitSideKey("iin_max", false) == "hv_i_max");
    EXPECT(limitSideKey("iout_max", false) == "lv_i_max");
    EXPECT(limitSideKey("vin_max", true) == "lv_max");
    EXPECT(limitSideKey("vout_max", true) == "hv_max");
    EXPECT(limitSideKey("iin_max", true) == "lv_i_max");
    EXPECT(limitSideKey("iout_max", true) == "hv_i_max");

    ConfFile lim{{"hv_max", "85"}, {"lv_max", "60"}, {"hv_i_max", "30"}, {"lv_i_max", "32"}, {"vin_min", "8"}};
    EXPECT(limitsUseSideKeys(lim)); // vin_min is role-named by design and does not count as a mix
    EXPECT(readMappedLimit(lim, "vin_max", true, false) == 85.f);
    EXPECT(readMappedLimit(lim, "vout_max", true, false) == 60.f);
    EXPECT(readMappedLimit(lim, "iin_max", true, false) == 30.f);
    EXPECT(readMappedLimit(lim, "iout_max", true, false) == 32.f);
    EXPECT(readMappedLimit(lim, "vin_max", true, true) == 60.f);
    EXPECT(readMappedLimit(lim, "vout_max", true, true) == 85.f);
    EXPECT(readMappedLimit(lim, "iin_max", true, true) == 32.f);
    EXPECT(readMappedLimit(lim, "iout_max", true, true) == 30.f);

    ConfFile limRole{{"vin_max", "85"}, {"vout_max", "60"}};
    EXPECT(!limitsUseSideKeys(limRole));
    EXPECT(readMappedLimit(limRole, "vin_max", false, true) == 85.f);

    // all-or-nothing in limits.conf too, also across different limits
    const auto limMsg = thrown([&] { limitsUseSideKeys(ConfFile{{"hv_max", "85"}, {"vout_max", "60"}}); });
    EXPECT(contains(limMsg, "limits.conf") && contains(limMsg, "hv_max") && contains(limMsg, "vout_max"));

    // a missing mapped limit names both spellings
    const auto miss = thrown([&] { readMappedLimit(limRole, "iin_max", false, false); });
    EXPECT(contains(miss, "iin_max") && contains(miss, "hv_i_max") && contains(miss, "topo=buck"));
    const auto missSide = thrown([&] { readMappedLimit(ConfFile{{"lv_max", "60"}}, "vout_max", true, true); });
    EXPECT(contains(missSide, "hv_max (or vout_max under topo=boost)"));

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("conv-side-test: all passed\n");
    return 0;
}
