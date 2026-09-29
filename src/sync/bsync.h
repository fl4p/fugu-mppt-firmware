#pragma once

// BeaconSyncService ("bsync"): frequency/phase-locks the MCPWM switching clock of multiple
// converters to a shared timebase recovered from sniffed 802.11 beacons. Receive-only: the
// device never associates or transmits — both boards timestamp the *same* beacon frames of one
// AP (common view), so all constant RX-path delays cancel between identical chips.
// The local MCPWM period is dithered between {P, P+1} ticks (first-order sigma-delta, latching
// on TEZ) to servo the switching phase onto a grid of period P+1/2 ticks in AP time, giving
// zero average frequency drift between devices and ~µs bounded relative phase.
// See website/docs/development/sync/beacon-sync.md.

#include <stdexcept> // both before buck.h: pwm/ledc.h expands ESP_ERROR_CHECK_THROW
#include "util.h"
#include "buck.h"

#if defined(WITH_BSYNC) && defined(HAVE_MCPWM)
#define HAVE_BSYNC 1

#include <esp_timer.h>
#include <esp_wifi.h>
#include "service.h"

class BeaconSyncService : public Service {
public:
    BeaconSyncService() : Service("bsync", "/littlefs/conf/bsync.conf",
                                  /*requiresNetwork*/ false, /*enabledDefault*/ false) {}

    std::string statusDetail() const override;

protected:
    bool onStart() override;
    void onStop() override;
    void onTick() override;

private:
    static void rxCb(void *buf, wifi_promiscuous_pkt_type_t type);
    static void ditherCb(void *arg);

    bool radioUp();          // (re)enable sniffer; false if the radio can't be brought up
    void servo(int64_t nowUs);

    // conf
    uint8_t bssid_[6]{};
    uint8_t channel_ = 1;
    float phaseUs_ = 0;      // per-device target phase offset on the shared grid
    float kp_ = 5e-6f;       // period-ticks per phase-error-tick (bw-scaled in onStart)
    float ki_ = 2.5e-7f;     // 1/s (bw-scaled)
    float bw_ = 1.0f;        // one-knob loop-bandwidth scale: lower = less phase breathing, slower lock
    float alphaA_ = 0.3f;    // alpha-beta gains, bw-scaled (A~bw, B~bw² keeps damping)
    float alphaB_ = 0.05f;
    bool hwOnly_ = false;    // accept only full-IE (hw TBTT) beacons, drop short injected frames

    // driver
    MCPWM_SyncLeg *leg_ = nullptr;
    uint16_t nomPeriod_ = 0;
    uint32_t ticksPerUs_ = 0;

    // beacon offset estimate (alpha-beta: offset o at local esp_timer time t, drift rate r).
    // The 32-bit hw rx stamp (its own µs clock, same crystal as esp_timer) is bridged to the
    // esp_timer domain via dEst_, a max-filter of (rxExt - espAtCb): callback latency is always
    // positive, so the max converges to (true offset - latency floor); the floor is identical
    // firmware on identical chips and cancels chip-to-chip.
    // Written by rxCb (wifi task), read by servo (network loop) — both core 0, mux-guarded.
    struct Est {
        double o = 0;        // localEspTimer - apTSF at time t [µs]
        double r = 0;        // d(o)/dt, local-vs-AP crystal drift [µs/µs]
        int64_t t = 0;       // local esp_timer of last accepted beacon [µs]
        uint32_t nBeacons = 0;
    };
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    Est est_{};
    bool estInit_ = false;
    uint8_t rejStreak_ = 0;
    uint32_t nRejected_ = 0, nClockDomain_ = 0, nTsfDead_ = 0;
    // reject-streak revalidation: a streak collects REVAL_N raw offsets and rebases on their
    // MEDIAN only if it truly disagrees — a transient reject burst (callback-latency spike,
    // bad-stamp cluster) then causes NO phase step. Hard re-seed stays reserved for epoch jumps.
    static constexpr int REVAL_N = 15;
    double revalBuf_[REVAL_N]{};
    uint8_t revalCnt_ = 0;
    bool revaling_ = false;
    uint32_t nReval_ = 0, nRevalStep_ = 0;
    // rx-clock bridge state (rxCb only)
    int64_t rxExt_ = 0;      // 64-bit extension of the 32-bit hw rx stamp
    uint32_t rxLast_ = 0;
    int64_t espLast_ = 0;    // esp_timer at last beacon; gaps > ~42 min lose 32-bit wraps -> restart
    bool rxInit_ = false;
    int64_t dEst_ = 0;       // max of (rxExt_ - espAtCb) [µs]
    bool dInit_ = false;

    // servo state
    double iAcc_ = 0;
    // feedforward uses a ~30s-smoothed drift rate: raw r carries per-frame estimator noise that
    // would otherwise be applied directly as frequency modulation (u = 0.5 + P·r + ...), slewing
    // the phase ~±1µs/s at 60 beacons/s (observed as white ±2µs shot noise on the scope)
    double rSmooth_ = 0;
    bool rSmoothInit_ = false;
    int64_t lastServoUs_ = 0, lastRadioRetryUs_ = 0;
    float lastErrUs_ = NAN;
    enum class Lock : uint8_t { Acquiring, Locked, Coasting } lock_ = Lock::Acquiring;

    // actuator (esp_timer task, 1 kHz)
    volatile float uCmd_ = 0.5f; // mean period offset in ticks, [0,1] over nomPeriod_
    esp_timer_handle_t ditherTimer_ = nullptr;
    float sdCarry_ = 0;
    int sdOut_ = -1;

    bool radioStartedByUs_ = false;
    wifi_ps_type_t prevPs_ = WIFI_PS_MIN_MODEM;

    static BeaconSyncService *inst_;
};

extern BeaconSyncService bsyncService;

#endif // WITH_NETW && HAVE_MCPWM
