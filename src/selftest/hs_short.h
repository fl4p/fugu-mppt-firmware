#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Passive high-side short check of an idle buck. With both switches off no current can pass from the
// input to the output; a forward current with Vin pulled down to Vout means the HS conducts, and so
// does an output above the OV limit that follows the input at ~0 A (the battery disconnected).
// i is the raw (pre-offset) output current; the zero is learnt while no current can flow.
struct HsShortGuard {
    static constexpr uint32_t OffMs = 500, HoldMs = 3000;
    // voltage alone: a shared-bus charger can push Vout above OV while the panel voltage passes by; a short holds it
    static constexpr uint32_t HoldVMs = 60000;
    static constexpr float VinWin = 1.5f, VinWinRel = .04f, Rallow = 0.05f, VoutMin = 5.f;

    uint32_t offSince = 0, suspectSince = 0;
    float zero = NAN;
    bool off = false, suspect = false;
    bool needCheck = true; // a start needs a clean verdict (boot, clear, after OffMs off)
    bool clean = false;    // the last evaluated sample was clean
    bool latched = false;
    bool byVoltage = false; // the latching sample met only the voltage criterion

    // two 1-2 % dividers can disagree by ~3 %
    static float window(float vout) { return std::max(VinWin, VinWinRel * vout); }

    // iMin: smallest current judged with the voltage window; iBig: judged alone; vOv: Vout judged with the
    // window alone (NaN: none, e.g. PSU mode); learn: no current can flow while Vin < Vout (BF off)
    // returns true on the sample that latches
    bool update(uint32_t nowMs, bool disabled, float vin, float vout, float iRaw, float iMin, float iBig, float vOv,
                bool learn) {
        if (latched) return false;
        if (!disabled) {
            off = suspect = needCheck = clean = false;
            return false;
        }
        if (!off) off = true, offSince = nowMs;
        if (nowMs - offSince < OffMs) return false;
        needCheck = true;
        if (!std::isfinite(vin) || !std::isfinite(vout) || !std::isfinite(iRaw)) {
            clean = false; // unevaluable: no verdict, a pending suspicion keeps its age
            return false;
        }
        if (learn && vin < vout - 2.f && std::fabs(iRaw) <= .5f * iBig)
            zero = std::isfinite(zero) ? zero + (iRaw - zero) * (1.f / 1024) : iRaw;
        const float i = current(iRaw), dv = vin - vout, w = window(vout);
        const bool near = dv > -w && dv < w + Rallow * std::max(i, 0.f);
        const bool sI = i > iBig || (i > iMin && vout > VoutMin && near);
        const bool sV = vout > vOv && near; // false for a NaN vOv
        const bool s = sI || sV;
        clean = !s;
        if (!s) {
            suspect = false;
            return false;
        }
        if (!suspect) suspect = true, suspectSince = nowMs;
        if (nowMs - suspectSince < (sI ? HoldMs : HoldVMs)) return false;
        latched = true, byVoltage = !sI;
        return true;
    }

    // a start must wait: latched, suspected, or not yet seen clean while a check is due
    [[nodiscard]] bool blocks() const { return latched || suspect || (needCheck && !clean); }
    [[nodiscard]] float current(float iRaw) const { return std::isfinite(zero) ? iRaw - zero : iRaw; }
};

// LS clamp after a latch: only at night. Vin must stay below VinMax for HoldMs without a break; NaN breaks it.
// Then the LS is pulsed with a width rising linearly from StartNs to the full period over RampUs (advancing at most
// StepMaxUs per step), then held on.
struct HsClampArm {
    static constexpr float VinMax = 9.f;
    static constexpr uint32_t HoldMs = 60000, RampUs = 20000, StepMaxUs = 4000;
    static constexpr float StartNs = 200.f;
    uint32_t since = 0;
    bool low = false;

    // true once Vin has been below VinMax for HoldMs
    bool update(uint32_t nowMs, float vin) {
        if (!(vin < VinMax)) return low = false;
        if (!low) low = true, since = nowMs;
        return nowMs - since >= HoldMs;
    }
    [[nodiscard]] uint32_t heldMs(uint32_t nowMs) const { return low ? nowMs - since : 0; }

    // LS on-time in ticks `us` into the ramp, t0 at 0, `full` from RampUs on
    static uint32_t rampTicks(uint32_t us, uint32_t t0, uint32_t full) {
        if (us >= RampUs || t0 >= full) return full;
        return t0 + (uint32_t) ((uint64_t) (full - t0) * us / RampUs);
    }
};
