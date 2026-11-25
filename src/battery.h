#pragma once

#include <cmath>
#include <cstdint>

enum CellChemistry {
    None = 0,
    LiFePo4, // LFP 2.5~3.65V

    NCM, // LiCoMn LiIon, LiPo, 3.0~4.2V. Pedelec, EV (no Tesla), Tesla Powerwall
    NCA, // Nickel-Cobalt-Aluminium, 3.0~4.2V. highest 200 mAh/g, Tesla EV, usually 18650 (LiNiCoAlO2)
    LCO, // LiCoO2

    LTO, // Li4Ti5O12 1.8~2.65V (float 2.25V) https://www.lto-store.de/home/Yinlong-LTO66160K-2-3V-45Ah-A-grade-p542371477

    AGM,
    LeadAcid,// https://batteryuniversity.com/article/bu-403-charging-lead-acid

    SIB, // sodium-ion, Na+

    Custom,

    Max,
};

inline float detectMaxBatteryVoltage(float openCircuitVoltage) {
    auto constexpr CellVolt = 3.65;
    auto constexpr CellPack = 4;
    float voltageStep = CellPack * CellVolt; // lifepo4 4s (3.65V) [1,2,4]
    float minVoltage = 10.f;

    for (int n = 1; n < 4; n *= 2) {
        if (openCircuitVoltage > minVoltage * (float) n && openCircuitVoltage < voltageStep * (float) n) {
            return voltageStep * (float) n;
        }
    }

    // default to 12V systems if voltage is low
    if (openCircuitVoltage < minVoltage)
        return voltageStep;

    // not detectable
    return NAN;
}

// vout_max=auto: a pack of n in {4, 8, 16} series cells matches when n*2.5 V <= V <= n*cv_eoc*1.03
constexpr float kCellVLow = 2.5f, kCellVMargin = 0.03f;

// the windows are non-empty and disjoint: cv_eoc*1.03 in [2.5, 5) V, i.e. cv_eoc in [2.43, 4.85] V
constexpr bool packWindowsOk(float cvEoc) {
    return cvEoc * (1 + kCellVMargin) >= kCellVLow && cvEoc * (1 + kCellVMargin) < 2 * kCellVLow;
}
static_assert(packWindowsOk(3.65f) && packWindowsOk(4.2f) && !packWindowsOk(4.9f) && !packWindowsOk(2.4f));

// series cells whose window holds v, 0 = none (NaN too)
inline uint8_t packCells(float v, float cvEoc) {
    for (uint8_t n = 4; n <= 16; n *= 2)
        if (v >= n * kCellVLow && v <= n * cvEoc * (1 + kCellVMargin)) return n;
    return 0;
}

// BMS frames agree with n cells at pack voltage v: v/vHigh <= n <= v/(vHigh - spread), +-0.5 cell. NaN fails
inline bool bmsCellsAgree(uint8_t n, float v, float vHigh, float spread) {
    if (!(spread >= 0.f)) spread = 0.f;
    return vHigh > spread && v / vHigh - .5f <= n && n <= v / (vHigh - spread) + .5f;
}

// vout_max=auto: one reading of the idle battery-side voltage per call. Returns the cell count once 3 readings in a
// row match the same window within 1 %, else 0 with why set. bmsHigh: fresh highest cell (NaN = no BMS), bmsV: BMS
// pack voltage (NaN: v), bmsSpread: cell spread (NaN: 0). vMax: the board's output limit
struct PackDetect {
    float v0 = NAN;
    uint8_t n = 0, streak = 0;
    const char *why = "no reading yet";

    uint8_t step(float v, float vin, bool idle, float cvEoc, float vMax, float bmsHigh, float bmsV, float bmsSpread) {
        const uint8_t c = packCells(v, cvEoc);
        why = !idle ? "converter switching"
            : !std::isfinite(v) ? "no Vout reading"
            : !(v >= 4 * kCellVLow) ? "no battery (Vout below 10 V)"
            : !c ? "Vout in no 4/8/16-cell window"
            : !(c * cvEoc <= vMax) ? "pack above the board's output limit"
            : !(fabsf(vin - v) > fmaxf(1.5f, .04f * v)) ? "Vin near Vout" // a conducting HS feeds the output
            : bmsHigh > 0 && !bmsCellsAgree(c, std::isfinite(bmsV) ? bmsV : v, bmsHigh, bmsSpread) ? "BMS cells disagree"
            : nullptr;
        if (why) return streak = 0;
        if (c != n || !(fabsf(v - v0) <= .01f * v0)) n = c, v0 = v, streak = 0;
        why = "settling";
        if (++streak < 3) return 0;
        streak = 0;
        return n;
    }

    // a pack of `cells` was detected: true once 3 idle readings in a row leave its band, then detect again. The band
    // [n * min(2.5 V, max(2 V, cv_eoc * 1.03 / 2 + 50 mV)), 2n * 2.5 V) holds the detection window and a deep-discharged
    // pack, no other 4/8/16-cell pack (disjoint windows). A switching reading restarts the count; NaN counts as outside
    bool left(float v, bool idle, uint8_t cells, float cvEoc) {
        const float lo = cells * fminf(kCellVLow, fmaxf(2.f, cvEoc * (1 + kCellVMargin) / 2 + .05f)),
                hi = cells * 2 * kCellVLow;
        if (!idle || (v >= lo && v < hi)) {
            streak = 0;
            return false;
        }
        if (++streak < 3) return false;
        *this = {};
        why = !std::isfinite(v) ? "no Vout reading" : v >= hi ? "above its band" : v < 4 * kCellVLow ? "pack removed"
            : "below its band";
        return true;
    }
};

// Battery disconnected under load (buck charger): Iout collapses, Vout jumps by more than dvMin and Vin
// rises. *Avg are slow averages still holding the pre-step state. A bus step that drives Iout to ~0 with
// the battery connected matches too.
inline bool bmsCutoffSignature(float iout, float ioutAvg, float ioutMin,
                               float vout, float voutAvg, float dvMin,
                               float vin, float vinAvg) {
    return ioutAvg > ioutMin && iout < ioutAvg * 0.25f && vout > voutAvg + dvMin && vin > vinAvg;
}
