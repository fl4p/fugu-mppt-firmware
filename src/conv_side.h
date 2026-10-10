#pragma once

// Side (HV/LV) naming for hardware config keys. sensor.conf and limits.conf name each channel and
// limit by the physical side it sits on. The topology maps sides to roles, as buck.h does for
// board.conf's pwm_hi/pwm_li:
//   buck:  input = HV, output = LV
//   boost: input = LV, output = HV
// A topo change is then a one-line edit in converter.conf.
//
// The old role keys (vin_ch, iout_factor, vin_max, ...) are no longer read. A file that still has
// one fails setup with the key's replacement under the board's topo (etc/migrate_side_keys.py).

#include <stdexcept>
#include <string>

#include "conf.h"

// converter.conf::topo, read the way BuckConverter::init() reads it. Anything but buck|boost throws.
// `neededBy` names the reader, for the error message.
inline bool readTopoIsBoost(const char *neededBy) {
    ConfFile conv{"/littlefs/conf/converter.conf", true};
    const std::string topo = conv.getString("topo", "buck");
    if (topo != "buck" && topo != "boost")
        throw std::runtime_error("converter.conf: topo must be buck|boost, got '" + topo + "' (needed by " + neededBy +
                                 ")");
    return topo == "boost";
}

// "hv" or "lv": the side that plays the input (input=true) or the output role.
inline const char *sideOf(bool input, bool boost) { return input != boost ? "hv" : "lv"; }

inline bool isInputRole(const std::string &role) { return role[1] == 'i'; } // vin, iin

// ---- sensor.conf

// Suffixes a sensor channel is configured with (<channel>_<suffix>).
inline constexpr const char *SensorKeySuffixes[] = {"_adc", "_ch", "_rh", "_rl", "_factor", "_midpoint", "_filt_len"};
inline constexpr const char *SensorRoleChannels[] = {"vin", "vout", "iin", "iout"}; // removed key prefixes

// Key prefix role channel `chn` is read from: hv_v, lv_v, hv_i or lv_i. ntc has no side.
inline std::string sensorKeyPrefix(const std::string &chn, bool boost) {
    if (chn == "ntc") return chn;
    return std::string(sideOf(isInputRole(chn), boost)) + '_' + chn[0];
}

// Side current factors use the buck direction: positive when power flows HV -> LV. A side's shunt
// sees the same physical current in either topo, but boost reverses the power flow (into the LV
// terminal, out of the HV one), so the factor is negated to keep Iin/Iout positive for forward
// power. The midpoint is the raw zero and stays. E.g. one LV shunt, lv_i_factor=-1: buck Iout and
// boost Iin both read it.
inline float sensorCurrentFactor(float factor, bool boost) { return boost ? -factor : factor; }

// Throws on the first removed role channel key (vin_ch, iout_factor, ...), naming its replacement.
inline void rejectSensorRoleKeys(const ConfFile &sensConf, bool boost) {
    for (const auto &k: sensConf.keys())
        for (auto r: SensorRoleChannels)
            for (auto s: SensorKeySuffixes)
                if (k == std::string(r) + s) {
                    std::string msg = "sensor.conf: " + k + " is no longer read; with topo=" + (boost ? "boost" : "buck") +
                                      " use " + sensorKeyPrefix(r, boost) + s;
                    if (boost && r[0] == 'i' && std::string(s) == "_factor")
                        msg += " with the sign flipped (side factors are positive HV->LV)";
                    throw std::runtime_error(msg);
                }
}

// ---- limits.conf

// The four per-side hardware ratings, by role. vin_min (source collapse floor), iout_short and
// p_max are role properties and keep their names.
inline constexpr const char *LimitRoleKeys[] = {"vin_max", "vout_max", "iin_max", "iout_max"}; // removed keys

// Side key of mapped role limit `roleKey` under the topo: vin_max -> hv_max (buck) / lv_max (boost), etc.
inline std::string limitSideKey(const std::string &roleKey, bool boost) {
    if (roleKey != "vin_max" && roleKey != "vout_max" && roleKey != "iin_max" && roleKey != "iout_max")
        throw std::invalid_argument("limitSideKey: not a mapped limit: " + roleKey);
    return std::string(sideOf(isInputRole(roleKey), boost)) + (roleKey[0] == 'i' ? "_i_max" : "_max");
}

// Throws on the first removed role limit key, naming its replacement.
inline void rejectLimitRoleKeys(const ConfFile &lim, bool boost) {
    for (auto r: LimitRoleKeys)
        if (lim.has(r))
            throw std::runtime_error(std::string("limits.conf: ") + r + " is no longer read; with topo=" +
                                     (boost ? "boost" : "buck") + " use " + limitSideKey(r, boost));
}

// Value of the side key that plays role limit `roleKey`.
inline float readMappedLimit(const ConfFile &lim, const std::string &roleKey, bool boost) {
    const auto key = limitSideKey(roleKey, boost);
    if (!lim.has(key))
        throw std::runtime_error("limits.conf: missing " + key + " (" + roleKey + " under topo=" +
                                 (boost ? "boost" : "buck") + ")");
    return lim.getFloat(key);
}
