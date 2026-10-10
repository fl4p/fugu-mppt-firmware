#pragma once

// Side (HV/LV) naming for hardware config keys. sensor.conf and limits.conf may name a channel or
// limit by the physical side it sits on instead of by converter role. The topology maps sides to
// roles, as buck.h does for board.conf's pwm_hi/pwm_li:
//   buck:  input = HV, output = LV
//   boost: input = LV, output = HV
// A topo change is then a one-line edit in converter.conf.

#include <stdexcept>
#include <string>

#include "conf.h"

// converter.conf::topo, read the way BuckConverter::init() reads it. Anything but buck|boost throws.
inline bool readTopoIsBoost() {
    ConfFile conv{"/littlefs/conf/converter.conf", true};
    const std::string topo = conv.getString("topo", "buck");
    if (topo != "buck" && topo != "boost")
        throw std::runtime_error("converter.conf::topo must be buck|boost, got '" + topo + "'");
    return topo == "boost";
}

// "hv" or "lv": the side that plays the input (input=true) or the output role.
inline const char *sideOf(bool input, bool boost) { return input != boost ? "hv" : "lv"; }

// Key to read for a value that has a role name and a side alias: the side key if present, else the
// role key. Both present throws, so a half-migrated config cannot silently read the wrong side.
inline std::string pickSideKey(const ConfFile &conf, const char *file, const std::string &roleKey,
                               const std::string &sideKey, bool boost) {
    if (!conf.has(sideKey)) return roleKey;
    if (conf.has(roleKey))
        throw std::runtime_error(std::string(file) + ": both " + roleKey + " and " + sideKey + " set (topo=" +
                                 (boost ? "boost" : "buck") + " maps " + sideKey + " to " + roleKey +
                                 "), keep one");
    return sideKey;
}

// Suffixes a sensor channel is configured with (<channel>_<suffix>).
inline constexpr const char *SensorKeySuffixes[] = {"adc", "ch", "rh", "rl", "factor", "midpoint", "filt_len"};

// Side alias (hv_v, lv_v, hv_i, lv_i) of role channel `chn` if sensor.conf configures it, else "" (read
// the legacy role keys). vin/iin sit on the input side. A channel configured in both forms throws;
// mixing forms across different channels is allowed, each channel resolves on its own.
inline std::string sensorSideChannel(const ConfFile &sensConf, const std::string &chn, bool boost) {
    if (chn == "ntc") return "";
    const bool input = chn == "vin" || chn == "iin";
    const std::string side = std::string(sideOf(input, boost)) + '_' + chn[0];
    const char *roleHit = nullptr, *sideHit = nullptr;
    for (auto sfx: SensorKeySuffixes) {
        if (!roleHit && sensConf.has(chn + '_' + sfx)) roleHit = sfx;
        if (!sideHit && sensConf.has(side + '_' + sfx)) sideHit = sfx;
    }
    if (roleHit && sideHit)
        throw std::runtime_error("sensor.conf: channel " + chn + " set as both " + chn + '_' + roleHit + " and " +
                                 side + '_' + sideHit + " (topo=" + (boost ? "boost" : "buck") + " maps " + side +
                                 " to " + chn + "), use one form");
    return sideHit ? side : "";
}
