#include "../mppt.h"
#include "bflow_test.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <ctime>

#include "../logging.h"
#include "../app_state.h"

#ifndef FUGU_HOST_SIM
#include "nvs.h"
bool flashWriteSafe();
#endif

extern SynchronousConverter converter;
extern MpptController mppt;
extern VIinVout<const Sensor *> sensors;
extern ADC_Sampler adcSampler;

namespace {
enum class St : uint8_t { Idle, Enter, Off, BfOff, Ramp, Seek, Settle, Step, Toggle, Stop, Gap, Restore, Done };

struct Res {
    char v = 0; // 0 not run, P pass, F fail, A aborted
    bool trip;
    float i, vin, vout, d, iMin;
};

struct {
    std::atomic<bool> busy{false};
    St st = St::Idle;
    uint8_t phase = 0; // 0 blocks or toggle, 1 conducts
    bool daily = false, keepMode = false;
    bool tog = false, noBat = false, fp = false; // toggle variant (phase 0 only), no battery, forced PWM on
    uint8_t k = 0; // toggle readings: even BF on, odd BF off
    float a = 0, ya = 0, xo = 0, yo = 0, on = 0, off = 0, dv = 0, vin0 = 0; // toggle: last on I and y, off I and y,
    // sums, smallest y step
    uint32_t ticket = 0, t0 = 0, t1 = 0, dwell = 0, tickMs = 0;
    OpMode savedMode = OpMode::Mppt;
    uint16_t savedManual = 0, d = 0, dCap = 0, bump = 0, dLive = 0; // dLive: MPPT duty at the take-over
    const char *why = nullptr;
    Res res[2];
} s;

// persisted (NVS "selftest"); unix times 0 = unknown
struct Rec {
    uint8_t ver = 3, hs = 0, bf = 0, unsafe = 0; // unsafe: a clamp met battery current, no blocks pass since
    char v0 = 0, v1 = 0;
    uint8_t tog = 0; // the blocks pass came from the toggle variant: no clamp
    uint32_t passUnix = 0, resUnix = 0; // blocks pass, last run with both verdicts
};
static_assert(sizeof(Rec) == 16, "");

// the machine, rec, the RAM ages and the scheduler: loopNetwork_task only (console commands included), no lock
Rec rec, stored;
uint64_t passMs = 0, resMs = 0, writeMs = 0; // uptime [ms] of this boot's events
uint32_t writeGapMs = 0;
bool passRam = false, resRam = false;
uint8_t writeFails = 0;
BfDaily daily;
std::atomic<bool> abortReq{false};
uint8_t seen = 0; // selftestUrgent: latch bits last looked at

const char *const kName[2] = {"blocks", "conducts"};
const char *name() { return s.tog ? "toggle" : kName[s.phase]; }
constexpr uint8_t TogN = 3;
constexpr uint32_t RecentS = 36u * 3600;

uint32_t unixNow() {
    const time_t t = time(nullptr);
    return t > 1000000000 ? (uint32_t) t : 0;
}

uint32_t ageS(bool ram, uint64_t ms, uint32_t unixS) {
    if (ram) return (uint32_t) std::min<uint64_t>((wallClockMs() - ms) / 1000, UINT32_MAX - 1);
    const uint32_t t = unixNow();
    return unixS && t >= unixS ? t - unixS : UINT32_MAX;
}

// 1 current, 2 converted (rewrite it), 0 unreadable. Another version or size keeps the hs/bf bytes (offsets 1 and 2
// since version 1) and drops the rest (the pass time: unverified); an unreadable record counts as a latched HS short.
int recDecode(Rec &r, const uint8_t *b, size_t n) {
    r = {};
    if (n == sizeof r && (b[0] == r.ver || b[0] == 2)) {
        const bool cur = b[0] == r.ver;
        memcpy(&r, b, sizeof r);
        if (cur) return 1;
        r.ver = 3, r.tog = 0; // 2: byte 6 was padding
        return 2;
    }
    if (n < 3) return r.hs = 1, 0;
    r.hs = b[1] != 0, r.bf = b[2] != 0;
    ESP_LOGE("selftest", "state record version %u (%u B): kept hs %u bf %u, pass time dropped", b[0], (unsigned) n,
             r.hs, r.bf);
    return 2;
}

#ifndef FUGU_HOST_SIM
// 0 no record, 1 read, 2 converted or unreadable (rewrite it)
int recRead(Rec &r) {
    r = {};
    nvs_handle_t h;
    uint8_t b[32] = {};
    size_t n = 0;
    esp_err_t e = nvs_open("selftest", NVS_READONLY, &h); // NOT_FOUND: no namespace yet
    if (e == ESP_OK) {
        n = sizeof b;
        e = nvs_get_blob(h, "st", b, &n);
        nvs_close(h);
    }
    if (e == ESP_ERR_NVS_NOT_FOUND) return 0;
    if (e == ESP_OK)
        if (const int rd = recDecode(r, b, n)) return rd;
    r.hs = 1;
    ESP_LOGE("selftest", "state record unreadable (%s, %u B): treated as a latched HS short", esp_err_to_name(e),
             (unsigned) n);
    return 2;
}

bool recWrite(const Rec &r) {
    nvs_handle_t h;
    if (nvs_open("selftest", NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = nvs_set_blob(h, "st", &r, sizeof r) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}
#else
} // namespace
uint8_t simWriteFail = 0;
namespace {
Rec simStore;
uint8_t simBlob[16];
size_t simBlobN = 0; // a raw record for the next load, through the target's decoder
int recRead(Rec &r) {
    if (!simBlobN) return r = simStore, r.ver == 3;
    const int rd = recDecode(r, simBlob, simBlobN);
    simBlobN = 0;
    return rd ? rd : (r.hs = 1, 2);
}
bool recWrite(const Rec &r) { return simWriteFail ? (--simWriteFail, false) : (simStore = r, true); }
bool flashWriteSafe() { return true; }
#endif

// a pack never sits below 2 V; the divider bleeds an open battery side there
bool noBattery() { return sensors.Vout->ewm.avg.get() < 2.f; }

// no battery: the toggle's last output voltage
float noBatV() { return std::min(.5f * sensors.Vin->ewm.avg.get(), .8f * mppt.charger.params.Vbat_max); }

// one reading per switch state, the filters of I, Vin and Vout settled
uint32_t togDwellMs() {
    return (uint32_t) std::max(60.f, std::max(mppt.powerSettleMs(), MpptController::settleMs(sensors.Vin)) + 20.f);
}

// a battery was seen this boot (it may sit behind an open BMS now): BMS data, or Vout above 9 V with the converter
// off for 2 min (an open output pumped up by the converter decays). nbOut: a no-battery run raised the output
bool sawBat = false, nbOut = false;
uint64_t convOnMs = 0;

void watchBat() {
    const uint64_t now = wallClockMs();
    if (mppt.charger.batSt.haveValidCellVoltage()) sawBat = true;
    if (!converter.disabled()) convOnMs = now;
    if (noBattery()) nbOut = false;
    else if (!nbOut && now - convOnMs > 120000 && sensors.Vout->ewm.avg.get() > 9.f) sawBat = true;
}

const char *voltRefusal() {
    const float vi = sensors.Vin->ewm.avg.get(), vo = sensors.Vout->ewm.avg.get();
    if ((noBattery() || !mppt.batIBipolar()) && togDwellMs() > 1000) return "sensor filters too slow for the toggle";
    if (noBattery()) {
        if (!mppt.charger.params.haveVbatMax()) return "no battery: Vbat_max unknown";
        if (mppt.charger.batSt.haveValidCellVoltage()) return "no battery, but BMS data is fresh";
        if (sawBat) return "no battery, but one was seen since boot (reboot without it)";
        // forced PWM from the minimum duty, 3 duty bumps of 1 V above it and room for the ramp
        const float dMinV = (converter.pwmCtrlMin + 1) * vi / converter.periodCounts();
        return noBatV() >= std::max(dMinV, vo + 1.f) + 4.f ? nullptr : "no battery: Vin too low for the toggle";
    }
    if (!(vo > 9.f) || !mppt.charger.params.haveVbatMax()) return "Vout implausible";
    if (!(vi >= 1.2f * vo && vi - vo >= 3.f)) return "Vin too low (needs >= 1.2 Vout and Vout + 3 V)";
    return nullptr;
}

void stopRt() {
    mppt.bfTest.mode.fetch_or(MpptController::BfStop);
    s.st = St::Stop, s.t0 = 0;
}

void abortRun(const char *why, uint32_t now) {
    if (!s.why) s.why = why;
    if (s.res[s.phase].v == 0) s.res[s.phase].v = 'A';
    stopRt();
    s.t0 = now;
}

void finish() {
    mppt.bfTest.mode.store(0);
    mppt.bflow.lockOff.fetch_and((uint8_t) ~BackflowDriver::LockSelftest);
}

void startPhase(uint32_t now) {
    if (mppt.inBackoff()) return abortRun("protection", now);
    if (const char *r = voltRefusal()) return abortRun(r, now);
    auto &b = mppt.bfTest;
    b.iMin = INFINITY, b.i = b.vin = b.vout = b.d = NAN;
    b.tripped.store(false);
    if (!s.phase) {
        mppt.bflow.lockOff.fetch_or(BackflowDriver::LockSelftest);
        mppt.bflow.enable(false);
    }
    b.mode.store(s.phase ? MpptController::BfConducts : MpptController::BfBlocks);
    s.st = St::BfOff, s.t0 = now;
}

// diode emulation from below D = M up to CCM, at most to a provably forward duty
void startRamp(uint32_t now) {
    const float m = sensors.Vout->ewm.avg.get() / sensors.Vin->ewm.avg.get(), t = converter.periodCounts();
    s.dCap = (uint16_t) std::min<float>(converter.pwmCtrlMax, std::ceil((m * 1.0202f + .01f) * t));
    if (s.phase || s.tog) s.dCap = std::max(s.dCap, s.dLive); // switch on: up to the live point's CCM
    s.d = (uint16_t) std::min<float>(s.dCap, m * .97f * t);
    s.dwell = constrain((uint32_t) (mppt.powerSettleMs() / 4), 20u, 500u);
    mppt.setManualTarget(s.d);
    s.st = St::Ramp, s.t0 = now;
}

// no battery: the switch off, the LS at its minimum (an asynchronous buck: no reverse path, even through a closed
// switch). Then forced PWM up to noBatV() - 3 bumps
void startNoBat(uint32_t now) {
    const float vi = sensors.Vin->ewm.avg.get(), t = converter.periodCounts();
    s.bump = (uint16_t) std::ceil(1.f / vi * t);
    s.dCap = (uint16_t) std::min<float>(converter.pwmCtrlMax, noBatV() / vi * t - 3 * s.bump);
    s.d = converter.pwmCtrlMin + 1;
    s.dwell = constrain((uint32_t) (mppt.powerSettleMs() / 4), 20u, 500u);
    mppt.setManualTarget(s.d);
    s.st = St::Ramp, s.t0 = now, s.t1 = 0;
}

// the duty climbs from the minimum until the output rises 1 V, at most to D Vin = Vout + 3 V (a pack, or an open
// Vout divider with a pack, does not rise). The RT then engages forced PWM, with the switch still off: a working
// Vout sense reading a few volts rules out a pack. Then the duty goes above the output.
void nbProbe(uint32_t now) {
    const float vi = sensors.Vin->ewm.avg.get(), t = converter.periodCounts();
    if (mppt.bfTest.mode.load(std::memory_order_acquire) & MpptController::BfEngage) {
        const float dn = std::ceil((sensors.Vout->med3.get() + .3f) / vi * t);
        if (dn > s.dCap) return abortRun("Vout pumped up", now);
        mppt.setManualTarget(s.d = std::max<uint16_t>(s.d, (uint16_t) dn));
        s.fp = true, s.t0 = now;
        return;
    }
    if (converter.getCtrlOnPwmCnt() != s.d) return;
    if (converter.dutyCtrlEff() * vi < mppt.bfTest.vProof + 2.f) mppt.setManualTarget(++s.d);
    else if (!s.t1) s.t1 = now;
    else if (now - s.t1 > 3000) abortRun("Vout does not follow the duty: a battery?", now);
}

// the RT switches; with a battery the current must stay above 30 % of the last on reading once the switch is on
void bfOn(bool on) {
    auto &b = mppt.bfTest;
    if (on) {
        if (!s.noBat) b.iFloor = std::max(.3f, .3f * s.a);
        mppt.bflow.lockOff.fetch_and((uint8_t) ~BackflowDriver::LockSelftest);
        b.mode.fetch_or(MpptController::BfOn, std::memory_order_release);
        return;
    }
    b.mode.fetch_and((uint8_t) ~MpptController::BfOn);
    // no battery: a blocking switch leaves the output a diode drop below the raised duty
    if (s.noBat) mppt.setManualTarget(s.d = std::min<uint16_t>(converter.pwmCtrlMax, s.d + s.bump));
}

void engage(uint32_t now) {
    const float i = mppt.sensorPhysicalI->ewm.avg.get();
    UART_LOG("selftest bflow %s: forced PWM from D=%.4f, I=%.2f A", name(), converter.dutyCtrlEff(), i);
    s.st = St::Step, s.t0 = now;
    if (!s.tog) return (void) mppt.bfTest.mode.fetch_or(MpptController::BfEngage);
    // toggle: the switch is on already (no battery: off, forced PWM on)
    s.st = St::Toggle, s.k = 0, s.a = i, s.on = s.off = 0, s.dv = INFINITY, s.vin0 = sensors.Vin->ewm.avg.get();
    s.dwell = togDwellMs();
    if (s.noBat) return bfOn(true);
    mppt.bfTest.iFloor = std::max(.3f, .3f * i);
    mppt.bfTest.mode.fetch_or(MpptController::BfEngage | MpptController::BfFloor, std::memory_order_release);
}

void togEnd(char v) {
    const uint8_t n = s.k / 2; // ends on an on reading: n + 1 on, n off
    s.res[0] = {v, false, s.on / (n + 1), sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(),
                converter.dutyCtrlEff(), n ? s.off / n : NAN};
    stopRt();
}

// y = D Vin - Vout: the loop's drop. A blocking switch adds its body diode's drop while off,
// unless the current collapses instead (a stiff source). Each off reading is judged against both on neighbours:
// they must agree (steady source and load, else abort), and the off reading must stand out (else FAIL).
void togStep(uint32_t now) {
    const bool on = !(s.k & 1);
    if (converter.disabled()) return abortRun("protection or a NaN current", now);
    if (now - s.t0 < s.dwell || converter.getCtrlOnPwmCnt() != s.d) return;
    const float vi = sensors.Vin->ewm.avg.get(), x = mppt.sensorPhysicalI->ewm.avg.get(),
            y = converter.dutyCtrlEff() * vi - sensors.Vout->ewm.avg.get();
    if (!std::isfinite(x) || !std::isfinite(y)) return abortRun("NaN reading", now);
    s.t0 = now;
    if (!on) {
        if (!(vi >= .99f * s.vin0)) return abortRun("Vin dropped", now);
        s.xo = x, s.yo = y, s.off += s.noBat ? y : x;
        bfOn(true);
    } else {
        const float a = s.a, ya = s.ya;
        s.on += s.noBat ? y : x, s.ya = y, s.a = x;
        if (s.k) {
            if (!(fabsf(y - ya) < .15f && (s.noBat || fabsf(x - a) <= .15f * std::max(x, a))))
                return abortRun("source or load changed", now);
            const float st = s.yo - std::max(ya, y);
            s.dv = std::min(s.dv, st);
            // the step is Vf less the loop drop a partial current collapse gives back; a closed switch steps 0
            if (!(st >= .1f) && (s.noBat || !(s.xo < .3f * std::min(x, a)))) return togEnd('F');
        }
        if (s.k == 2 * TogN) return togEnd('P');
        bfOn(false);
    }
    ++s.k;
}

// fold the RT-side latches into rec
void syncRec() {
    if (mppt.hsClampUnsafe.load() && !rec.unsafe) rec.unsafe = 1, passRam = false, rec.passUnix = 0;
    rec.hs = mppt.hsFaultState() != 0, rec.bf = mppt.bfFail.load();
}

// urgent skips the 10 s throttle and the failure back-off (one try)
void writeRec(uint64_t now, bool urgent) {
    if (!memcmp(&rec, &stored, sizeof rec) || !flashWriteSafe()) return;
    if (!urgent && (s.busy || now - writeMs < writeGapMs)) return;
    writeMs = now;
    if (recWrite(rec)) {
        if (writeFails) ESP_LOGI("selftest", "state written after %u failures", writeFails);
        stored = rec, writeFails = 0, writeGapMs = 10000;
    } else {
        if (!writeFails) ESP_LOGW("selftest", "state write failed, retrying with back-off");
        if (writeFails < 255) ++writeFails;
        writeGapMs = std::min<uint32_t>(std::max<uint32_t>(writeGapMs, 10000) * 2, 3600000);
    }
}

// converter off: a leaking BF blocks the start and revokes the pass, persisted before anything else
void blocksFail(uint64_t now) {
    mppt.bfFailSet(true);
    passRam = false, rec.passUnix = 0;
    syncRec();
    writeRec(now, true);
}

void runEnd() {
    const char v0 = s.res[0].v, v1 = s.res[1].v;
    const uint64_t now = wallClockMs();
    const bool fail = v0 == 'F' || v1 == 'F', complete = v0 && v0 != 'A' && (s.tog || (v1 && v1 != 'A'));
    if (fail) {
        mppt.bfFailSet(true);
        ESP_LOGE("selftest", "backflow switch FAILED (%s %c, conducts %c): start blocked until `selftest clear`",
                 s.tog ? "toggle" : "blocks", v0 ?: '-', v1 ?: '-');
    }
    const uint32_t t = unixNow();
    if (v0 == 'P')
        passRam = true, passMs = now, rec.passUnix = t, rec.unsafe = 0, rec.tog = s.tog, mppt.hsClampUnsafe.store(false);
    if (v0 == 'F') passRam = false, rec.passUnix = 0;
    if (complete || fail) resRam = true, resMs = now, rec.resUnix = t, rec.v0 = v0, rec.v1 = v1;
    daily.result(now, !complete && !fail);
    syncRec();
    if (fail) writeRec(now, true);
}

bool dailyEligible() {
    if (g_app.opMode.load() != OpMode::Mppt || converter.disabled() || mppt.limiting()) return false;
    // the test needs CCM forward current: the live point in CCM with half the ripple below 2/3 of the current
    const float i = mppt.sensorPhysicalI->ewm.avg.get();
    if (converter.inDCM() || !(i >= .75f * converter.rippleCurrent(sensors.Vin->ewm.avg.get(),
                                                                   sensors.Vout->ewm.avg.get())))
        return false;
    return !noBattery() && !bfTestRefusal();
}
}

#ifdef FUGU_HOST_SIM
bool simStoredBf() { return simStore.bf; }
void simLoadBlob(const uint8_t *b, size_t n) { memcpy(simBlob, b, simBlobN = std::min(n, sizeof simBlob)); }
uint8_t simRecTog() { return rec.tog; }
uint8_t simStoredVer() { return simStore.ver; }
bool simStoredUnsafe() { return simStore.unsafe; }
uint32_t simStoredPass() { return simStore.passUnix; }
#endif

const char *bfTestRefusal() {
    if (s.busy) return "already running";
    watchBat();
    if (converter.boost()) return "buck only";
    if (!mppt.hsIdleCheck()) return "needs board.conf bflow_bat=1 (battery-side backflow switch and current sensor)";
    if (mppt.hsFaultState()) return "hs-short latched";
    if (mppt.bfFail.load()) return "bf-fail latched (selftest clear)";
    if (converter.bsyncOwnsPeriod) return "bsync owns the PWM period";
    if (g_app.psuMode()) return "PSU mode";
    if (adcSampler.isCalibrating() || adcSampler.halted) return "sampler calibrating or halted";
    if (mppt.isSweeping() || mppt.rectCal.cal.running()) return "sweep or rect-cal running";
    if (mppt.hasPendingPsuCommand() || !converter.deadTimeIdle()) return "command pending";
    if (mppt.inBackoff()) return "backoff";
    if (!mppt.sensorPhysicalI->calibrationComplete) return "current sensor not calibrated";
    return voltRefusal();
}

bool bfTestStart() {
    if (bfTestRefusal() || s.busy.exchange(true)) return false;
    abortReq = false;
    s.res[0] = s.res[1] = {};
    s.why = nullptr, s.phase = 0, s.st = St::Enter, s.t0 = 0, s.daily = s.keepMode = s.tog = s.noBat = s.fp = false;
    s.k = 0;
    return true;
}

bool bfTestBusy() { return s.busy; }

void bfTestAbort() {
    if (s.busy) abortReq = true;
}

char bfTestVerdict(uint8_t phase) { return s.res[phase & 1].v; }

void bfTestPrint() {
    for (uint8_t p = 0; p < 2; ++p) {
        const Res &r = s.res[p];
        if (!r.v) continue;
        if (s.tog && r.v == 'A') {
            UART_LOG("selftest bflow toggle: ABORTED");
            continue;
        }
        if (s.tog) {
            UART_LOG("selftest bflow toggle (%s): %s %s on %.2f, off %.2f %c, min step %.2f V Vin=%.2f Vout=%.2f D=%.4f",
                     s.noBat ? "no battery" : "battery", r.v == 'P' ? "PASS" : "FAIL", s.noBat ? "D Vin-Vout" : "I",
                     r.i, r.iMin, s.noBat ? 'V' : 'A', s.dv, r.vin, r.vout, r.d);
            continue;
        }
        UART_LOG("selftest bflow %s: %s I=%.2f A (min %.2f) Vin=%.2f Vout=%.2f D=%.4f dV=%.2f%s", kName[p],
                 r.v == 'P' ? "PASS" : r.v == 'F' ? "FAIL" : "ABORTED", r.i, r.iMin, r.vin, r.vout, r.d,
                 r.vout - r.d * r.vin, r.trip ? " (reverse current)" : "");
    }
    if (s.why) UART_LOG("selftest bflow: %s", s.why);
}

bool bfTestTick(uint32_t now) {
    if (s.st == St::Idle) return false;
    if (now - s.tickMs < 10) return true;
    s.tickMs = now;
    if (abortReq.exchange(false) && s.st < St::Restore) {
        if (s.st == St::Enter) s.keepMode = true; // nothing taken over yet
        if (s.st <= St::Off || s.st == St::Gap) s.why = "aborted", s.st = St::Restore;
        else if (s.st == St::Stop) s.why = s.why ?: "aborted"; // no next phase
        else abortRun("aborted", now);
    }
    if (s.tog && s.st > St::BfOff && s.st < St::Stop && mppt.bfTest.tripped.load(std::memory_order_acquire)) {
        static const char *const w[] = {"forward current lost", "D Vin below Vout", "no battery, but current flows",
                                        "Vin rose: energy from the output"};
        const uint8_t y = mppt.bfTest.why;
        abortRun(y >= 1 && y <= 4 ? w[y - 1] : "RT stop", now);
    }
    if (s.noBat && s.st > St::BfOff && s.st < St::Stop) {
        if (mppt.sensorPhysicalI->med3.get() > .5f) abortRun("no battery, but current flows", now);
        else if (mppt.charger.batSt.haveValidCellVoltage()) abortRun("no battery, but BMS data arrived", now);
    }
    if (s.st > St::Off && s.st < St::Restore) {
        if (g_app.opMode.load() != OpMode::Manual) {
            // the user took over: the RT side drops the exemptions, the new mode stays
            s.keepMode = true;
            abortRun("mode changed", now);
            finish();
            s.st = St::Restore;
        } else if (mppt.hsFaultState()) abortRun("hs-short latched", now);
    }
    auto &b = mppt.bfTest;
    switch (s.st) {
        case St::Enter:
            s.savedMode = g_app.opMode.load(), s.savedManual = mppt.getManualTarget();
            s.dLive = s.savedMode == OpMode::Mppt && !converter.disabled() ? converter.getCtrlOnPwmCnt() : 0;
            s.ticket = mppt.requestPsuManual(0, -1, true, true);
            s.st = St::Off, s.t0 = now;
            break;
        case St::Off: // the take-over landed, the converter is off and the current filters have drained
            if (mppt.isPsuCommandDone(s.ticket) && converter.disabled() &&
                (mppt.inBackoff() || fabsf(mppt.sensorPhysicalI->ewm.avg.get()) < 1.f))
                startPhase(now);
            else if (now - s.t0 > 10000) {
                s.why = "take-over timed out";
                s.st = St::Restore, s.t0 = 0;
            }
            break;
        case St::BfOff: // BF turn-off takes 0.3-1.5 ms
            if (now - s.t0 >= 20 && (s.phase || !mppt.bflow.state())) {
                if (!s.phase) {
                    // the battery current cannot go negative, or there is no battery: toggle the switch instead
                    s.noBat = noBattery(), s.tog = s.noBat || !mppt.batIBipolar();
                    if (s.tog) {
                        if (const char *r = voltRefusal()) return abortRun(r, now), true;
                        const float fs = static_cast<const PhysicalSensor *>(mppt.sensorPhysicalI)->notchFs;
                        b.holdN = (uint8_t) std::min(60.f, 3.f + std::ceil(fs * .01f)); // median + the BF gate's 2 tau
                        b.why = 0;
                        b.vProof = sensors.Vout->ewm.avg.get() + 1.f;
                        b.vinMax = 1.03f * sensors.Vin->ewm.avg.get() + .5f; // no battery: nothing to pull Vin above the open circuit
                        b.mode.store(s.noBat ? MpptController::BfToggle | MpptController::BfNoBat
                                             : MpptController::BfToggle);
                        nbOut |= s.noBat;
                        if (!s.noBat) bfOn(true);
                    }
                }
                if (s.noBat) startNoBat(now);
                else startRamp(now);
            } else if (now - s.t0 > 1000) abortRun("BF still on", now);
            break;
        case St::Ramp:
            if (converter.getCtrlOnPwmCnt() == s.d) s.st = St::Seek, s.t0 = now;
            else if (now - s.t0 > 5000) abortRun(converter.disabled() ? "start refused" : "ramp timed out", now);
            break;
        case St::Seek:
            if (converter.disabled()) abortRun("protection", now);
            else if (s.noBat && !s.fp) nbProbe(now);
            else if (now - s.t0 >= s.dwell && converter.getCtrlOnPwmCnt() == s.d) {
                s.t0 = now;
                const float i = mppt.sensorPhysicalI->ewm.avg.get();
                const uint16_t up = std::min<uint16_t>(s.dCap, s.d + std::max(1, (int) (converter.periodCounts() / 512)));
                if (s.noBat) { // ramp the open output up
                    if (s.d >= s.dCap) s.st = St::Settle;
                    else mppt.setManualTarget(s.d = up);
                    break;
                }
                // CCM forward current: D is above the zero-current duty, forced PWM pulls no reverse current. The
                // BF body diode carries it in phase blocks, so that phase engages at once and does not climb near the
                // 6 A highI-bflowOpen trip.
                const bool ccm = !converter.inDCM();
                if (s.tog ? ccm && i >= 1.f : (ccm && i >= .3f) || (!s.phase && (s.d >= s.dCap || i >= 4.5f))) {
                    if (s.phase || s.tog) s.st = St::Settle;
                    else engage(now);
                } else if (s.d < s.dCap) mppt.setManualTarget(++s.d);
                else if (s.tog && ccm && i >= .5f) s.st = St::Settle;
                else abortRun("no CCM forward current at the duty cap", now);
            }
            break;
        case St::Settle: // gate pump of the BF, ADC filters
            if (converter.disabled()) abortRun("protection", now);
            else if (now - s.t0 < 150) break;
            else if (s.noBat ? !s.fp : converter.inDCM()) abortRun(s.noBat ? "Vout not proven" : "DCM at engage", now);
            else engage(now);
            break;
        case St::Toggle:
            togStep(now);
            break;
        case St::Step: {
            Res &r = s.res[s.phase];
            if (b.tripped.load(std::memory_order_acquire)) {
                r = {s.phase ? 'P' : 'F', true, b.i, b.vin, b.vout, b.d, b.iMin};
                stopRt();
                break;
            }
            if (converter.disabled()) {
                abortRun("protection or a NaN current", now);
                break;
            }
            if (now - s.t0 < s.dwell) break;
            s.t0 = now;
            const float vi = sensors.Vin->ewm.avg.get(), vo = sensors.Vout->ewm.avg.get(), d = converter.dutyCtrlEff();
            if (d * vi <= vo - 1.f || s.d <= converter.pwmCtrlMin + 2) {
                const bool ok = !s.phase && b.iMin >= -.2f;
                r = {ok ? 'P' : 'F', false, mppt.sensorPhysicalI->ewm.avg.get(), vi, vo, d, b.iMin};
                if (!(vi > vo + 1.f)) { // the ramp ended on a collapsed input, not on the duty
                    r.v = 0;
                    abortRun("Vin collapsed", now);
                    break;
                }
                stopRt();
                break;
            }
            // body-diode current in phase blocks: leave it faster
            s.d -= !s.phase && mppt.sensorPhysicalI->ewm.avg.get() > 2.f ? 2 : 1;
            mppt.setManualTarget(s.d);
            break;
        }
        case St::Stop:
            if (!s.t0) s.t0 = now;
            if (converter.disabled()) {
                mppt.setManualTarget(0);
                b.mode.store(0);
                if (!s.phase) mppt.bflow.lockOff.fetch_and((uint8_t) ~BackflowDriver::LockSelftest);
                if (!s.phase && s.res[0].v == 'F') blocksFail(wallClockMs());
                const bool next = !s.why && !s.tog && s.phase == 0 && s.res[0].v == 'P'; // no conducts phase on a leaking BF
                s.st = next ? St::Gap : St::Restore, s.t0 = now;
            } else if (now - s.t0 > 2000) {
                s.why = "converter did not stop";
                s.st = St::Restore, s.t0 = now;
            }
            break;
        case St::Gap: // off long enough for the HS-short check before the next start
            if (now - s.t0 >= 1500) s.phase = 1, startPhase(now);
            break;
        case St::Restore:
            finish();
            // backoffs and the charger's latches stay as they are
            s.ticket = s.keepMode ? 0
                       : s.savedMode == OpMode::Manual ? mppt.requestPsuManual(s.savedManual, -1, true)
                                                       : mppt.requestPsuOff(true);
            s.st = St::Done, s.t0 = now;
            break;
        case St::Done:
            if (s.ticket && !mppt.isPsuCommandDone(s.ticket) && now - s.t0 < 2000) break;
            if (s.ticket && !mppt.isPsuCommandDone(s.ticket)) UART_LOG("selftest bflow: mode restore timed out");
            runEnd();
            bfTestPrint();
            s.st = St::Idle;
            s.busy = false;
            return false;
        default:
            break;
    }
    return true;
}

uint32_t bfPassAgeS() { return ageS(passRam, passMs, rec.passUnix); }

void selftestLoad() {
    const int rd = recRead(rec);
    stored = rec;
    if (rd == 2) stored.ver = 0; // rewrite in the current layout
    passRam = resRam = false;
    mppt.hsClampUnsafe.store(rec.unsafe);
    if (rec.hs && converter.boost()) {
        rec.hs = 0; // the check does not run on a boost: drop it, persisted on the next write
        ESP_LOGW("selftest", "HS short latched before the reboot ignored: boost");
    } else if (rec.hs) {
        mppt.hsRestore();
        ESP_LOGE("selftest", "HS short latched before the reboot: start blocked, gates off (selftest clear)");
    }
    if (rec.bf) {
        mppt.bfFailSet(true);
        ESP_LOGE("selftest", "backflow switch failed before the reboot: start blocked (selftest clear)");
    }
}

void selftestClearBf() { mppt.bfFailSet(false); }

void selftestUrgent(uint64_t now) {
    const uint8_t b = mppt.hsClampUnsafe.load() | (mppt.hsFaultState() != 0) << 1 | mppt.bfFail.load() << 2;
    if (b == seen) return;
    const bool rise = b & ~seen;
    seen = b;
    if (!rise) return;
    syncRec();
    writeRec(now, true);
}

void selftestLf(uint64_t now, bool maintenance) {
    if (!s.busy) watchBat();
    if (const uint32_t t = unixNow()) { // the clock arrived after a result
        if (passRam && !rec.passUnix) rec.passUnix = t - (uint32_t) ((now - passMs) / 1000);
        if (resRam && !rec.resUnix) rec.resUnix = t - (uint32_t) ((now - resMs) / 1000);
    }
    syncRec();
    mppt.bfRecent.store(bfPassAgeS() <= RecentS && !rec.tog, std::memory_order_relaxed);
    writeRec(now, false);

    if (s.busy || !mppt.hsIdleCheck()) return;
    const uint32_t ra = ageS(resRam, resMs, rec.resUnix);
    if (daily.poll(now, ra >= BfDaily::DueMs / 1000, !maintenance && dailyEligible())) {
        if (const char *r = bfTestRefusal()) {
            UART_LOG("selftest bflow: daily run refused: %s", r);
            daily.refused(now);
        } else if (bfTestStart()) {
            s.daily = true;
            UART_LOG("selftest bflow: daily run");
        }
    }
}

void selftestPrintState() {
    const uint32_t pa = bfPassAgeS(), ra = ageS(resRam, resMs, rec.resUnix);
    char last[24] = "never", pass[24] = "never";
    if (ra != UINT32_MAX) snprintf(last, sizeof last, "%.1f h ago", (float) ra / 3600.f);
    if (pa != UINT32_MAX) snprintf(pass, sizeof pass, "%.1f h ago", (float) pa / 3600.f);
    if (!mppt.hsIdleCheck()) {
        UART_LOG("bflow: not checked (board.conf bflow_bat=0)");
        return;
    }
    UART_LOG("bflow: %s; last run %c/%c %s; blocks pass %s (%s%s)%s%s",
             mppt.bfFail.load() ? "FAIL LATCHED" : "ok", rec.v0 ?: '-', rec.v1 ?: '-', last, pass,
             pa <= RecentS ? "verified" : "unverified", rec.tog && pa <= RecentS ? ", toggle: no clamp" : "",
             rec.unsafe ? "; clamp met battery current" : "",
             s.busy ? "; running" : "");
}
