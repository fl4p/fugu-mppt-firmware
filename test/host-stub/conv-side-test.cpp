// Host test for src/conv_side.h: HV/LV side keys in sensor.conf / limits.conf map to roles by topo,
// and a removed role key (vin_ch, iout_max, ...) throws with its replacement named.
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

// Reading the sensor code computes for raw value x: (x - midpoint) * factor, as LinearTransform::apply.
static float currentReading(const ConfFile &s, const std::string &chn, bool boost, float x) {
    const auto k = sensorKeyPrefix(chn, boost);
    const float factor = sensorCurrentFactor(s.getFloat(k + "_factor", 1.f), boost);
    return (x - s.getFloat(k + "_midpoint", 0.f)) * factor;
}

int main() {
    // sides by topo: buck in=HV out=LV, boost in=LV out=HV
    EXPECT(std::string(sideOf(true, false)) == "hv");
    EXPECT(std::string(sideOf(false, false)) == "lv");
    EXPECT(std::string(sideOf(true, true)) == "lv");
    EXPECT(std::string(sideOf(false, true)) == "hv");

    // sensor.conf: role channel -> side key prefix
    EXPECT(sensorKeyPrefix("vin", false) == "hv_v");
    EXPECT(sensorKeyPrefix("vout", false) == "lv_v");
    EXPECT(sensorKeyPrefix("iin", false) == "hv_i");
    EXPECT(sensorKeyPrefix("iout", false) == "lv_i");
    EXPECT(sensorKeyPrefix("vin", true) == "lv_v");
    EXPECT(sensorKeyPrefix("vout", true) == "hv_v");
    EXPECT(sensorKeyPrefix("iin", true) == "lv_i");
    EXPECT(sensorKeyPrefix("iout", true) == "hv_i");
    EXPECT(sensorKeyPrefix("ntc", true) == "ntc");

    // a side-keyed sensor.conf passes; keys outside the channel suffix set are not role keys
    ConfFile side{{"hv_v_adc", "esp32adc1"}, {"hv_v_ch", "3"}, {"lv_v_ch", "0"}, {"lv_i_ch", "1"},
                  {"iin_min_supply_voltage", "6"}, {"ntc_ch", "7"}, {"adc", "ina226"}};
    for (bool boost: {false, true}) EXPECT(thrown([&] { rejectSensorRoleKeys(side, boost); }).empty());

    // every removed role channel key throws, naming the key and its replacement under the topo
    for (bool boost: {false, true})
        for (auto r: SensorRoleChannels)
            for (auto s: SensorKeySuffixes) {
                const std::string key = std::string(r) + s;
                ConfFile f{{"hv_v_ch", "3"}, {key, "1"}};
                const auto msg = thrown([&] { rejectSensorRoleKeys(f, boost); });
                const std::string want = "sensor.conf: " + key + " is no longer read; with topo=" +
                                         (boost ? "boost" : "buck") + " use " + sensorKeyPrefix(r, boost) + s;
                EXPECT(msg.rfind(want, 0) == 0);
            }
    EXPECT(thrown([&] { rejectSensorRoleKeys(ConfFile{{"vout_ch", "0"}}, true); }) ==
           "sensor.conf: vout_ch is no longer read; with topo=boost use hv_v_ch");
    EXPECT(thrown([&] { rejectSensorRoleKeys(ConfFile{{"iin_factor", "2"}}, true); }) ==
           "sensor.conf: iin_factor is no longer read; with topo=boost use lv_i_factor with the sign flipped "
           "(side factors are positive HV->LV)");
    EXPECT(thrown([&] { rejectSensorRoleKeys(ConfFile{{"iout_factor", "-1"}}, false); }) ==
           "sensor.conf: iout_factor is no longer read; with topo=buck use lv_i_factor");

    // current factor: side keys use the buck direction, boost negates. With a midpoint the raw zero
    // stays put and only the sign of the reading flips.
    EXPECT(sensorCurrentFactor(-1.f, false) == -1.f);
    EXPECT(sensorCurrentFactor(-1.f, true) == 1.f);
    ConfFile lvShunt{{"lv_i_ch", "1"}, {"lv_i_factor", "-2"}, {"lv_i_midpoint", "2.5"}};
    for (float x: {0.f, 2.5f, 3.f}) {
        EXPECT(currentReading(lvShunt, "iin", true, x) == (x - 2.5f) * 2.f);    // old boost iin_factor=2
        EXPECT(currentReading(lvShunt, "iout", false, x) == (x - 2.5f) * -2.f); // old buck iout_factor=-2
    }

    // limits.conf: role <-> side mapping
    EXPECT(limitSideKey("vin_max", false) == "hv_max");
    EXPECT(limitSideKey("vout_max", false) == "lv_max");
    EXPECT(limitSideKey("iin_max", false) == "hv_i_max");
    EXPECT(limitSideKey("iout_max", false) == "lv_i_max");
    EXPECT(limitSideKey("vin_max", true) == "lv_max");
    EXPECT(limitSideKey("vout_max", true) == "hv_max");
    EXPECT(limitSideKey("iin_max", true) == "lv_i_max");
    EXPECT(limitSideKey("iout_max", true) == "hv_i_max");

    ConfFile lim{{"hv_max", "85"}, {"lv_max", "60"}, {"hv_i_max", "30"}, {"lv_i_max", "32"}, {"vin_min", "8"},
                 {"iout_short", "40"}, {"p_max", "800"}};
    for (bool boost: {false, true}) EXPECT(thrown([&] { rejectLimitRoleKeys(lim, boost); }).empty());
    EXPECT(readMappedLimit(lim, "vin_max", false) == 85.f);
    EXPECT(readMappedLimit(lim, "vout_max", false) == 60.f);
    EXPECT(readMappedLimit(lim, "iin_max", false) == 30.f);
    EXPECT(readMappedLimit(lim, "iout_max", false) == 32.f);
    EXPECT(readMappedLimit(lim, "vin_max", true) == 60.f);
    EXPECT(readMappedLimit(lim, "vout_max", true) == 85.f);
    EXPECT(readMappedLimit(lim, "iin_max", true) == 32.f);
    EXPECT(readMappedLimit(lim, "iout_max", true) == 30.f);

    // every removed role limit throws, naming its replacement under the topo
    for (bool boost: {false, true})
        for (auto r: LimitRoleKeys) {
            ConfFile f{{"hv_max", "85"}, {r, "1"}};
            EXPECT(thrown([&] { rejectLimitRoleKeys(f, boost); }) ==
                   std::string("limits.conf: ") + r + " is no longer read; with topo=" + (boost ? "boost" : "buck") +
                   " use " + limitSideKey(r, boost));
        }
    EXPECT(thrown([&] { rejectLimitRoleKeys(ConfFile{{"vout_max", "60"}}, true); }) ==
           "limits.conf: vout_max is no longer read; with topo=boost use hv_max");

    // a missing side limit names the key and the role it plays
    EXPECT(thrown([&] { readMappedLimit(ConfFile{{"lv_max", "60"}}, "vout_max", true); }) ==
           "limits.conf: missing hv_max (Vout max under topo=boost)");

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("conv-side-test: all passed\n");
    return 0;
}
