#pragma once

#include <cstdint>

// `selftest bflow`: checks the battery-side backflow switch of a buck. Forced PWM (complementary LS) ramps the
// duty down from just above Vout/Vin until D*Vin is 1 V below Vout:
//  - blocks:   BF held off, the current must stay >= -0.2 A (trip below -0.3 A)
//  - conducts: BF on, a reverse current trip below -0.5 A must happen
// Converter off and the previous mode restored at the end; a blocks FAIL skips the conducts phase. A FAIL latches bf-fail (start blocked) until
// `selftest clear`; the state persists across reboots, as does a latched HS short.

// nullptr: may start, else the reason
const char *bfTestRefusal();
// arms the step machine; false if refused or running
bool bfTestStart();
bool bfTestBusy();
// stops a running test: converter off, previous mode back, no verdict
void bfTestAbort();
// loopNetwork_task, every pass: one step of the machine at most every 10 ms; false once done
bool bfTestTick(uint32_t nowMs);
void bfTestPrint();
// last verdict of phase 0 (blocks) / 1 (conducts): P pass, F fail, A aborted, 0 not run
char bfTestVerdict(uint8_t phase);

// persisted HS latch / bf-fail / pass times: read once before the RT task starts
void selftestLoad();
// core 0, about once a second: clock fix-up, persistence, the daily run. maintenance: OTA, measure-coil, ...
void selftestLf(uint64_t nowMs, bool maintenance);
// core 0, every loop pass: writes a new HS latch, bf-fail or clamp-unsafe flag at once
void selftestUrgent(uint64_t nowMs);
// clears a latched bf-fail (the HS latch is cleared by MpptController::hsClear)
void selftestClearBf();
void selftestPrintState();
// [s] since the last blocks pass, UINT32_MAX if unknown
uint32_t bfPassAgeS();

// When the daily run may start: due and eligible for HoldMs; an aborted run retries after RetryMs, at most
// MaxAborts times, then waits DueMs. A refused start waits RefusedMs.
struct BfDaily {
    static constexpr uint32_t DueMs = 20u * 3600 * 1000, HoldMs = 60000, RetryMs = 30u * 60000,
            RefusedMs = 5u * 60000;
    static constexpr uint8_t MaxAborts = 4;
    uint64_t since = 0, nextMs = 0;
    uint8_t aborts = 0;
    bool ok = false, wait = false;

    bool poll(uint64_t now, bool due, bool eligible) {
        if (wait && now < nextMs) return ok = false;
        wait = false;
        if (!due || !eligible) return ok = false;
        if (!ok) ok = true, since = now;
        if (now - since < HoldMs) return false;
        ok = false; // fires once
        return true;
    }
    void defer(uint64_t now, uint32_t ms) { wait = true, ok = false, nextMs = now + ms; }
    void refused(uint64_t now) { defer(now, RefusedMs); }
    void result(uint64_t now, bool aborted) {
        ok = false;
        if (!aborted) {
            aborts = 0;
            return;
        }
        if (++aborts >= MaxAborts) aborts = 0, defer(now, DueMs);
        else defer(now, RetryMs);
    }
};

#ifdef FUGU_HOST_SIM
extern uint8_t simWriteFail; // fail the next NVS writes
bool simStoredBf();
bool simStoredUnsafe();
uint32_t simStoredPass();
#endif
