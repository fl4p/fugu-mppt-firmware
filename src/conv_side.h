#pragma once

// Side (HV/LV) naming for hardware config keys. sensor.conf and limits.conf may name a channel or
// limit by the physical side it sits on instead of by converter role. The topology maps sides to
// roles, as buck.h does for board.conf's pwm_hi/pwm_li:
//   buck:  input = HV, output = LV
//   boost: input = LV, output = HV
// A topo change is then a one-line edit in converter.conf.
//
// Each file is all-or-nothing: it names its mapped keys by side or by role, never both, so a
// half-migrated file cannot silently read a channel from the wrong side.

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

inline const char *topoName(bool boost) { return boost ? "boost" : "buck"; }

// First key of `conf` of the form <prefix><suffix>, or "".
template<size_t NP, size_t NS>
inline std::string firstKeyOf(const ConfFile &conf, const char *const (&prefixes)[NP],
                              const char *const (&suffixes)[NS]) {
    for (const auto &k: conf.keys())
        for (auto p: prefixes)
            for (auto s: suffixes)
                if (k == std::string(p) + s) return k;
    return "";
}

// Throws if `conf` has keys of both forms, naming one of each.
template<size_t NR, size_t NSd, size_t NS>
inline bool usesSideKeys(const ConfFile &conf, const char *file, const char *const (&rolePrefixes)[NR],
                         const char *const (&sidePrefixes)[NSd], const char *const (&suffixes)[NS]) {
    const auto side = firstKeyOf(conf, sidePrefixes, suffixes);
    if (side.empty()) return false;
    const auto role = firstKeyOf(conf, rolePrefixes, suffixes);
    if (!role.empty())
        throw std::runtime_error(std::string(file) + ": mixes side key " + side + " with role key " + role +
                                 "; use side keys (hv_/lv_) or role keys throughout, not both");
    return true;
}

// ---- sensor.conf

// Suffixes a sensor channel is configured with (<channel>_<suffix>).
inline constexpr const char *SensorKeySuffixes[] = {"_adc", "_ch", "_rh", "_rl", "_factor", "_midpoint", "_filt_len"};
inline constexpr const char *SensorRoleChannels[] = {"vin", "vout", "iin", "iout"};
inline constexpr const char *SensorSideChannels[] = {"hv_v", "lv_v", "hv_i", "lv_i"};

// True if sensor.conf names its v/i channels by side. A mix of side and role channel keys throws.
inline bool sensorUsesSideKeys(const ConfFile &sensConf) {
    return usesSideKeys(sensConf, "sensor.conf", SensorRoleChannels, SensorSideChannels, SensorKeySuffixes);
}

// Key prefix role channel `chn` is read from: its side alias (hv_v, lv_v, hv_i, lv_i) in a side-keyed
// sensor.conf, else the role name. vin/iin sit on the input side; ntc has no side.
inline std::string sensorKeyPrefix(const std::string &chn, bool sideKeys, bool boost) {
    if (!sideKeys || chn == "ntc") return chn;
    const bool input = chn == "vin" || chn == "iin";
    return std::string(sideOf(input, boost)) + '_' + chn[0];
}

// Side current factors use the buck direction: positive when power flows HV -> LV. A side's shunt
// sees the same physical current in either topo, but boost reverses the power flow (into the LV
// terminal, out of the HV one), so the factor is negated to keep Iin/Iout positive for forward
// power. The midpoint is the raw zero and stays. E.g. one LV shunt: buck iout_factor=-1 == boost
// iin_factor=+1 == lv_i_factor=-1 in both.
inline float sensorCurrentFactor(float factor, bool sideKeys, bool boost) {
    return sideKeys && boost ? -factor : factor;
}

// ---- limits.conf

// The four per-side hardware ratings. vin_min (source collapse floor), iout_short and p_max are role
// properties and stay role-named.
inline constexpr const char *LimitRoleKeys[] = {"vin_max", "vout_max", "iin_max", "iout_max"};
inline constexpr const char *LimitSideKeys[] = {"hv_max", "lv_max", "hv_i_max", "lv_i_max"};

// Side alias of a mapped role limit under the topo: vin_max -> hv_max (buck) / lv_max (boost), etc.
inline std::string limitSideKey(const std::string &roleKey, bool boost) {
    const bool input = roleKey == "vin_max" || roleKey == "iin_max";
    const bool current = roleKey[0] == 'i';
    if (!input && roleKey != "vout_max" && roleKey != "iout_max")
        throw std::invalid_argument("limitSideKey: not a mapped limit: " + roleKey);
    return std::string(sideOf(input, boost)) + (current ? "_i_max" : "_max");
}

// True if limits.conf names its four mapped limits by side. A mix throws.
inline bool limitsUseSideKeys(const ConfFile &lim) {
    static constexpr const char *none[] = {""};
    return usesSideKeys(lim, "limits.conf", LimitRoleKeys, LimitSideKeys, none);
}

// Value of mapped role limit `roleKey`, read from its side alias in a side-keyed file. A missing key
// names both spellings.
inline float readMappedLimit(const ConfFile &lim, const std::string &roleKey, bool sideKeys, bool boost) {
    const auto side = limitSideKey(roleKey, boost);
    const auto &key = sideKeys ? side : roleKey;
    if (!lim.has(key))
        throw std::runtime_error("limits.conf: missing " + key + " (or " + (sideKeys ? roleKey : side) +
                                 " under topo=" + topoName(boost) + ")");
    return lim.getFloat(key);
}
