#include "../mppt.h"
#include "bflow_test.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <ctime>
#ifdef FUGU_HOST_SIM
#include <mutex>
#endif

#include "../logging.h"
#include "../app_state.h"

#ifndef FUGU_HOST_SIM
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
bool flashWriteSafe();
#endif

extern SynchronousConverter converter;
extern MpptController mppt;
extern VIinVout<const Sensor *> sensors;
extern ADC_Sampler adcSampler;

namespace {
enum class St : uint8_t { Idle, Enter, Off, BfOff, Ramp, Seek, Settle, Step, Stop, Gap, Restore, Done };

struct Res {
    char v = 0; // 0 not run, P pass, F fail, A aborted
    bool trip;
    float i, vin, vout, d, iMin;
};

struct {
    std::atomic<bool> busy{false};
    St st = St::Idle;
    uint8_t phase = 0; // 0 blocks, 1 conducts
    bool daily = false, keepMode = false;
    uint32_t ticket = 0, t0 = 0, dwell = 0;
    OpMode savedMode = OpMode::Mppt;
    uint16_t savedManual = 0, d = 0, dCap = 0;
    const char *why = nullptr;
    Res res[2];
} s;

// persisted (NVS "selftest"); unix times 0 = unknown
struct Rec {
    uint8_t ver = 2, hs = 0, bf = 0, unsafe = 0; // unsafe: a clamp met battery current, no blocks pass since
    char v0 = 0, v1 = 0;
    uint32_t passUnix = 0, resUnix = 0; // blocks pass, last run with both verdicts
};
static_assert(sizeof(Rec) == 16, "");

// rec, the RAM ages and the scheduler: core 0 (selftestLf, selftestUrgent, the test task), under Lock
#ifdef FUGU_HOST_SIM
std::mutex mx;
struct Lock {
    std::lock_guard<std::mutex> g{mx};
};
#else
// static: a std::mutex takes ~100 B of heap at its first lock (boot) and keeps it
SemaphoreHandle_t mxHandle() {
    static StaticSemaphore_t buf;
    static const SemaphoreHandle_t h = xSemaphoreCreateMutexStatic(&buf);
    return h;
}
struct Lock {
    Lock() { xSemaphoreTake(mxHandle(), portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(mxHandle()); }
    Lock(const Lock &) = delete;
};
#endif
Rec rec, stored;
uint64_t passMs = 0, resMs = 0, writeMs = 0; // uptime [ms] of this boot's events
uint32_t writeGapMs = 0;
bool passRam = false, resRam = false;
uint8_t writeFails = 0;
BfDaily daily;
std::atomic<bool> abortReq{false};
uint8_t seen = 0; // selftestUrgent: latch bits last looked at

const char *const kName[2] = {"blocks", "conducts"};
constexpr uint32_t RecentS = 36u * 3600;

uint32_t unixNow() {
    const time_t t = time(nullptr);
    return t > 1000000000 ? (uint32_t) t : 0;
}

uint32_t ageS(bool ram, uint64_t ms, uint32_t unix) {
    if (ram) return (uint32_t) std::min<uint64_t>((wallClockMs() - ms) / 1000, UINT32_MAX - 1);
    const uint32_t t = unixNow();
    return unix && t >= unix ? t - unix : UINT32_MAX;
}

#ifndef FUGU_HOST_SIM
// 0 no record, 1 read, 2 converted (rewrite it). Another version or size keeps the hs/bf bytes (offsets 1 and 2 since version 1) and drops the
// rest (the pass time: unverified); an unreadable record counts as a latched HS short.
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
    if (e == ESP_OK && n == sizeof r && b[0] == r.ver) return memcpy(&r, b, sizeof r), 1;
    if (e == ESP_OK && n >= 3) {
        r.hs = b[1] != 0, r.bf = b[2] != 0;
        ESP_LOGE("selftest", "state record version %u (%u B): kept hs %u bf %u, pass time dropped", b[0], (unsigned) n,
                 r.hs, r.bf);
    } else {
        r.hs = 1;
        ESP_LOGE("selftest", "state record unreadable (%s, %u B): treated as a latched HS short", esp_err_to_name(e),
                 (unsigned) n);
    }
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
int recRead(Rec &r) { return r = simStore, r.ver == 2; }
bool recWrite(const Rec &r) { return simWriteFail ? (--simWriteFail, false) : (simStore = r, true); }
bool flashWriteSafe() { return true; }
#endif

const char *voltRefusal() {
    const float vi = sensors.Vin->ewm.avg.get(), vo = sensors.Vout->ewm.avg.get();
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
    s.d = (uint16_t) std::min<float>(s.dCap, m * .97f * t);
    s.dwell = constrain((uint32_t) (mppt.powerSettleMs() / 4), 20u, 500u);
    mppt.setManualTarget(s.d);
    s.st = St::Ramp, s.t0 = now;
}

void engage(uint32_t now) {
    UART_LOG("selftest bflow %s: forced PWM from D=%.4f, I=%.2f A", kName[s.phase], converter.dutyCtrlEff(),
             mppt.sensorPhysicalI->ewm.avg.get());
    mppt.bfTest.mode.fetch_or(MpptController::BfEngage);
    s.st = St::Step, s.t0 = now;
}

// under Lock: fold the RT-side latches into rec
void syncRec() {
    if (mppt.hsClampUnsafe.load() && !rec.unsafe) rec.unsafe = 1, passRam = false, rec.passUnix = 0;
    rec.hs = mppt.hsFaultState() != 0, rec.bf = mppt.bfFail.load();
}

// under Lock: urgent skips the 10 s throttle and the failure back-off (one try)
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

// test task, converter off: a leaking BF blocks the start and revokes the pass, persisted before anything else
void blocksFail(uint64_t now) {
    mppt.bfFailSet(true);
    Lock l;
    passRam = false, rec.passUnix = 0;
    syncRec();
    writeRec(now, true);
}

// test task: the result
void runEnd() {
    const char v0 = s.res[0].v, v1 = s.res[1].v;
    const uint64_t now = wallClockMs();
    const bool fail = v0 == 'F' || v1 == 'F', complete = v0 && v1 && v0 != 'A' && v1 != 'A';
    if (fail) {
        mppt.bfFailSet(true);
        ESP_LOGE("selftest", "backflow switch FAILED (blocks %c, conducts %c): start blocked until `selftest clear`",
                 v0 ?: '-', v1 ?: '-');
    }
    Lock l;
    const uint32_t t = unixNow();
    if (v0 == 'P') passRam = true, passMs = now, rec.passUnix = t, rec.unsafe = 0, mppt.hsClampUnsafe.store(false);
    if (v0 == 'F') passRam = false, rec.passUnix = 0;
    if (complete || fail) resRam = true, resMs = now, rec.resUnix = t, rec.v0 = v0, rec.v1 = v1;
    daily.result(now, !complete && !fail);
    syncRec();
    if (fail) writeRec(now, true);
}

bool dailyEligible() {
    if (g_app.opMode.load() != OpMode::Mppt || converter.disabled() || mppt.limiting()) return false;
    if (!(mppt.sensorPhysicalI->ewm.avg.get() >= .5f)) return false;
    return !bfTestRefusal();
}
}

#ifdef FUGU_HOST_SIM
bool simStoredBf() { return simStore.bf; }
bool simStoredUnsafe() { return simStore.unsafe; }
uint32_t simStoredPass() { return simStore.passUnix; }
#endif

const char *bfTestRefusal() {
    if (s.busy) return "already running";
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

bool bfTestArm() {
    if (bfTestRefusal() || s.busy.exchange(true)) return false;
    abortReq = false;
    s.res[0] = s.res[1] = {};
    s.why = nullptr, s.phase = 0, s.st = St::Enter, s.t0 = 0, s.daily = s.keepMode = false;
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
        UART_LOG("selftest bflow %s: %s I=%.2f A (min %.2f) Vin=%.2f Vout=%.2f D=%.4f dV=%.2f%s", kName[p],
                 r.v == 'P' ? "PASS" : r.v == 'F' ? "FAIL" : "ABORTED", r.i, r.iMin, r.vin, r.vout, r.d,
                 r.vout - r.d * r.vin, r.trip ? " (reverse current)" : "");
    }
    if (s.why) UART_LOG("selftest bflow: %s", s.why);
}

bool bfTestTick(uint32_t now) {
    if (s.st == St::Idle) return false;
    if (abortReq.exchange(false) && s.st < St::Restore) {
        if (s.st == St::Enter) s.keepMode = true; // nothing taken over yet
        if (s.st <= St::Off || s.st == St::Gap) s.why = "aborted", s.st = St::Restore;
        else if (s.st == St::Stop) s.why = s.why ?: "aborted"; // no next phase
        else abortRun("aborted", now);
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
            if (now - s.t0 >= 20 && (s.phase || !mppt.bflow.state())) startRamp(now);
            else if (now - s.t0 > 1000) abortRun("BF still on", now);
            break;
        case St::Ramp:
            if (converter.getCtrlOnPwmCnt() == s.d) s.st = St::Seek, s.t0 = now;
            else if (now - s.t0 > 5000) abortRun(converter.disabled() ? "start refused" : "ramp timed out", now);
            break;
        case St::Seek:
            if (converter.disabled()) abortRun("protection", now);
            else if (now - s.t0 >= s.dwell && converter.getCtrlOnPwmCnt() == s.d) {
                s.t0 = now;
                const float i = mppt.sensorPhysicalI->ewm.avg.get();
                // CCM forward current: D is above the zero-current duty. The BF body diode carries it in phase
                // blocks, so that phase engages at once and does not climb near the 6 A highI-bflowOpen trip.
                if ((!converter.inDCM() && i >= .3f) || s.d >= s.dCap || (!s.phase && i >= 4.5f)) {
                    if (s.phase) s.st = St::Settle;
                    else engage(now);
                } else mppt.setManualTarget(++s.d);
            }
            break;
        case St::Settle: // gate pump of the BF, ADC filters
            if (converter.disabled()) abortRun("protection", now);
            else if (now - s.t0 >= 150) engage(now);
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
                const bool next = !s.why && s.phase == 0 && s.res[0].v == 'P'; // no conducts phase on a leaking BF
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

#ifndef FUGU_HOST_SIM
static void bfTestTask(void *) {
    while (bfTestTick((uint32_t) wallClockMs())) vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelete(nullptr);
}

bool bfTestStart() {
    if (!bfTestArm()) return false;
    if (xTaskCreatePinnedToCore(bfTestTask, "bf-test", 6144, nullptr, 1, nullptr, NON_RT_CORE) != pdPASS) {
        s.st = St::Idle;
        s.busy = false;
        return false;
    }
    return true;
}
#endif

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
    Lock l;
    syncRec();
    writeRec(now, true);
}

void selftestLf(uint64_t now, bool maintenance) {
    Lock l;
    if (const uint32_t t = unixNow()) { // the clock arrived after a result
        if (passRam && !rec.passUnix) rec.passUnix = t - (uint32_t) ((now - passMs) / 1000);
        if (resRam && !rec.resUnix) rec.resUnix = t - (uint32_t) ((now - resMs) / 1000);
    }
    syncRec();
    mppt.bfRecent.store(bfPassAgeS() <= RecentS, std::memory_order_relaxed);
    writeRec(now, false);

    if (s.busy || !mppt.hsIdleCheck()) return;
    const uint32_t ra = ageS(resRam, resMs, rec.resUnix);
    if (daily.poll(now, ra >= BfDaily::DueMs / 1000, !maintenance && dailyEligible())) {
        if (bfTestArm()) {
            s.daily = true;
            UART_LOG("selftest bflow: daily run");
#ifndef FUGU_HOST_SIM
            if (xTaskCreatePinnedToCore(bfTestTask, "bf-test", 6144, nullptr, 1, nullptr, NON_RT_CORE) != pdPASS) {
                s.st = St::Idle;
                s.busy = false;
                daily.refused(now);
            }
#endif
        } else daily.refused(now);
    }
}

void selftestPrintState() {
    Lock l;
    const uint32_t pa = bfPassAgeS(), ra = ageS(resRam, resMs, rec.resUnix);
    char last[24] = "never", pass[24] = "never";
    if (ra != UINT32_MAX) snprintf(last, sizeof last, "%.1f h ago", (float) ra / 3600.f);
    if (pa != UINT32_MAX) snprintf(pass, sizeof pass, "%.1f h ago", (float) pa / 3600.f);
    if (!mppt.hsIdleCheck()) {
        UART_LOG("bflow: not checked (board.conf bflow_bat=0)");
        return;
    }
    UART_LOG("bflow: %s; last run %c/%c %s; blocks pass %s (%s)%s%s",
             mppt.bfFail.load() ? "FAIL LATCHED" : "ok", rec.v0 ?: '-', rec.v1 ?: '-', last, pass,
             pa <= RecentS ? "verified" : "unverified", rec.unsafe ? "; clamp met battery current" : "",
             s.busy ? "; running" : "");
}
