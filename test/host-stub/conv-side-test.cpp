// Host test for src/conv_side.h: HV/LV side keys in sensor.conf / limits.conf map to roles by topo,
// and a value set in both forms is rejected.
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

template<class F>
static bool throws(F f) {
    try { f(); } catch (const std::runtime_error &) { return true; }
    return false;
}

int main() {
    // sides by topo: buck in=HV out=LV, boost in=LV out=HV
    EXPECT(std::string(sideOf(true, false)) == "hv");
    EXPECT(std::string(sideOf(false, false)) == "lv");
    EXPECT(std::string(sideOf(true, true)) == "lv");
    EXPECT(std::string(sideOf(false, true)) == "hv");

    // sensor.conf: side-only config resolves per topo
    ConfFile side{{"hv_v_adc", "esp32adc1"}, {"hv_v_ch", "3"}, {"lv_v_ch", "0"}, {"lv_i_ch", "1"}};
    EXPECT(sensorSideChannel(side, "vin", false) == "hv_v");
    EXPECT(sensorSideChannel(side, "vout", false) == "lv_v");
    EXPECT(sensorSideChannel(side, "iout", false) == "lv_i");
    EXPECT(sensorSideChannel(side, "iin", false) == "");      // hv_i unset -> role keys (virtual)
    EXPECT(sensorSideChannel(side, "vin", true) == "lv_v");
    EXPECT(sensorSideChannel(side, "vout", true) == "hv_v");
    EXPECT(sensorSideChannel(side, "iin", true) == "lv_i");
    EXPECT(sensorSideChannel(side, "iout", true) == "");
    EXPECT(sensorSideChannel(side, "ntc", true) == "");

    // legacy role-only config is untouched
    ConfFile role{{"vin_ch", "3"}, {"vout_ch", "0"}, {"iout_factor", "-1"}};
    for (bool boost: {false, true})
        for (auto c: {"vin", "vout", "iin", "iout"})
            EXPECT(sensorSideChannel(role, c, boost) == "");

    // same channel in both forms: rejected, also when the overlap is in different suffixes
    ConfFile both{{"vin_adc", "esp32adc1"}, {"hv_v_ch", "3"}};
    EXPECT(throws([&] { sensorSideChannel(both, "vin", false); }));
    EXPECT(sensorSideChannel(both, "vin", true) == "");       // boost: hv_v is vout, vin stays role
    EXPECT(sensorSideChannel(both, "vout", true) == "hv_v");

    // mixing forms across channels is allowed, but a topo flip that lands both on one channel throws
    ConfFile mixed{{"vin_ch", "3"}, {"lv_v_ch", "0"}};
    EXPECT(sensorSideChannel(mixed, "vout", false) == "lv_v");
    EXPECT(throws([&] { sensorSideChannel(mixed, "vin", true); }));

    // keys outside the channel suffix set do not count (legacy iin_min_supply_voltage)
    ConfFile legacy{{"iin_min_supply_voltage", "6"}, {"hv_i_ch", "2"}};
    EXPECT(sensorSideChannel(legacy, "iin", false) == "hv_i");

    // limits.conf
    ConfFile lim{{"hv_max", "85"}, {"lv_max", "60"}};
    EXPECT(pickSideKey(lim, "limits.conf", "vin_max", "hv_max", false) == "hv_max");
    EXPECT(pickSideKey(lim, "limits.conf", "vout_max", "lv_max", false) == "lv_max");
    EXPECT(lim.getFloat(pickSideKey(lim, "limits.conf", "vin_max", "lv_max", true)) == 60.f);
    EXPECT(lim.getFloat(pickSideKey(lim, "limits.conf", "vout_max", "hv_max", true)) == 85.f);
    ConfFile limRole{{"vin_max", "85"}};
    EXPECT(pickSideKey(limRole, "limits.conf", "vin_max", "hv_max", false) == "vin_max");
    ConfFile limBoth{{"vin_max", "85"}, {"hv_max", "85"}};
    EXPECT(throws([&] { pickSideKey(limBoth, "limits.conf", "vin_max", "hv_max", false); }));

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("conv-side-test: all passed\n");
    return 0;
}
