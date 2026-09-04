#include "sync/bsync.h"

#ifdef HAVE_BSYNC

#include <WiFi.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "app_state.h"

extern SynchronousConverter converter;

BeaconSyncService bsyncService;
BeaconSyncService *BeaconSyncService::inst_ = nullptr;

// "aa:bb:cc:dd:ee:ff" (or '-') -> 6 bytes. No sscanf: newlib-nano has no %hhx.
static bool parseBssid(const std::string &s, uint8_t out[6]) {
    if (s.size() != 17) return false;
    for (int i = 0; i < 6; ++i) {
        char *end;
        long v = strtol(s.c_str() + i * 3, &end, 16);
        if (end != s.c_str() + i * 3 + 2 || v < 0) return false;
        if (i < 5 && s[i * 3 + 2] != ':' && s[i * 3 + 2] != '-') return false;
        out[i] = (uint8_t) v;
    }
    return true;
}

// Runs in the wifi task on every sniffed mgmt frame. Keep it lean: no logging (3 KB stack,
// see [[project_bootlog_hook_wifi_stack_overflow]]), counters only.
void BeaconSyncService::rxCb(void *buf, wifi_promiscuous_pkt_type_t type) {
    auto *s = inst_;
    if (!s || type != WIFI_PKT_MGMT) return;
    auto *p = (wifi_promiscuous_pkt_t *) buf;
    if (p->rx_ctrl.sig_len < 36) return;      // 24B header + 12B fixed beacon body
    const uint8_t *pl = p->payload;
    if (pl[0] != 0x80) return;                // beacon subtype only
    if (memcmp(pl + 16, s->bssid_, 6) != 0) return;
    // injected skeleton frames are 47B; real softAP beacons carry the full IE set (>80B)
    if (s->hwOnly_ && p->rx_ctrl.sig_len < 64) return;

    uint64_t apTsf;                           // hw-inserted AP timestamp, first field of the body
    memcpy(&apTsf, pl + 24, 8);

    // extend the 32-bit hw rx stamp (own µs clock, hw-latched at frame arrival) to 64 bit.
    // A beacon gap long enough to lose whole 32-bit wraps (71.6 min) can't be delta-extended;
    // restart the bridge + estimator instead of carrying a silently wrap-biased rxExt_.
    uint32_t rx32 = p->rx_ctrl.timestamp;
    int64_t espNow = esp_timer_get_time();
    portENTER_CRITICAL(&s->mux_);
    if (!s->rxInit_ || espNow - s->espLast_ > 2'500'000'000LL) {
        s->rxExt_ = rx32;
        s->rxInit_ = true;
        s->dInit_ = false;
        s->estInit_ = false;
        s->revaling_ = false;
    } else {
        s->rxExt_ += (uint32_t) (rx32 - s->rxLast_);
    }
    s->rxLast_ = rx32;
    s->espLast_ = espNow;

    // bridge rx clock -> esp_timer: both run off the same crystal, so d is constant per wifi
    // session; max-filter converges to (true offset - callback-latency floor). Ratchet only
    // during acquisition: after convergence a lucky low-latency callback would rebase the
    // bridge and step the phase by µs (observed +1.85µs on the scope) — freeze it instead,
    // keeping only the epoch-jump re-seeds. The frozen floor residual is a small constant
    // per-device bias.
    int64_t dMeas = s->rxExt_ - espNow;
    bool acquiring = !s->estInit_ || s->est_.nBeacons < 300;
    if (!s->dInit_ || (dMeas > s->dEst_ && acquiring)) {
        if (s->dInit_ && dMeas - s->dEst_ > 1'000'000) { // rx-clock epoch jumped (wifi restart)
            s->nClockDomain_++;
        }
        s->dEst_ = dMeas;
        s->dInit_ = true;
    } else if (s->dEst_ - dMeas > 1'000'000 || dMeas - s->dEst_ > 1'000'000) {
        s->nClockDomain_++;                              // epoch jumped: re-seed bridge
        s->dEst_ = dMeas;
    }

    int64_t w = s->rxExt_ - s->dEst_;         // frame arrival in the esp_timer domain, µs
    double meas = (double) (w - (int64_t) apTsf);

    if (!s->estInit_) {
        s->est_ = {meas, 0.0, w, 1};
        s->estInit_ = true;
        s->rejStreak_ = 0;
    } else {
        double dt = (double) (w - s->est_.t);
        if (dt < 0) {
            // local TSF went backwards = wifi driver restart reset the clock domain; the
            // rejStreak_ path can't reach this (dt stays negative for hours) — re-seed now
            s->est_ = {meas, 0.0, w, 1};
            s->rejStreak_ = 0;
            s->revaling_ = false;
        } else if (dt > 1000.0) {             // ignore duplicate/burst frames
            double pred = s->est_.o + s->est_.r * dt;
            if (s->revaling_) {
                // reject-streak revalidation: collect raw offsets, judge by their median.
                // est stays frozen meanwhile (servo keeps predicting via r).
                s->revalBuf_[s->revalCnt_++] = meas;
                if (s->revalCnt_ >= REVAL_N) {
                    double tmp[REVAL_N];
                    memcpy(tmp, s->revalBuf_, sizeof tmp);
                    std::sort(tmp, tmp + REVAL_N);
                    double med = tmp[REVAL_N / 2];
                    if (std::abs(med - pred) > 500.0) {
                        pred = med;           // genuine offset step: robust rebase, keep r
                        s->nRevalStep_++;
                    }                          // else: transient burst — resume with NO step
                    s->est_.o = pred;          // o re-anchored at time w in both cases
                    s->est_.t = w;
                    // nBeacons preserved: the dEst_ ratchet stays frozen through revalidation
                    s->revaling_ = false;
                    s->rejStreak_ = 0;
                }
            } else {
                double resid = meas - pred;
                if (std::abs(resid) > 500.0) {
                    s->nRejected_++;
                    if (++s->rejStreak_ >= 8) { // persistent disagreement: revalidate, don't re-seed
                        s->revaling_ = true;
                        s->revalCnt_ = 0;
                        s->nReval_++;
                    }
                } else {
                    s->rejStreak_ = 0;
                    double A = s->alphaA_, B = s->alphaB_; // alpha-beta, ~critically damped at bw=1 / 10 Hz beacons
                    s->est_.o = pred + A * resid;
                    s->est_.r = std::clamp(s->est_.r + B * resid / dt, -200e-6, 200e-6);
                    s->est_.t = w;
                    s->est_.nBeacons++;
                }
            }
        }
    }
    portEXIT_CRITICAL(&s->mux_);
}

// 1 kHz first-order sigma-delta: period alternates {P, P+1} so the mean period tracks
// nomPeriod_ + uCmd_. Never below nominal: a comparator at P-1 would miss its event in a
// shrunken period and leave the LS gate high a full cycle.
void BeaconSyncService::ditherCb(void *arg) {
    auto *s = (BeaconSyncService *) arg;
    s->sdCarry_ += s->uCmd_;
    int out = (int) s->sdCarry_;
    s->sdCarry_ -= (float) out;
    out = std::clamp(out, 0, 1);
    if (out != s->sdOut_) {
        s->leg_->setPeriodTicks((uint16_t) (s->nomPeriod_ + out));
        s->sdOut_ = out;
    }
}

bool BeaconSyncService::radioUp() {
    // bring the driver up unassociated (RX-only; a sniffer never transmits)
    if (!(WiFi.getMode() & WIFI_MODE_STA)) {
        if (!WiFi.mode(WIFI_STA)) {
            ESP_LOGE(name(), "can't start wifi driver");
            return false;
        }
        radioStartedByUs_ = true;
    }
    esp_wifi_set_ps(WIFI_PS_NONE); // beacon RX + µs stamp precision need the RF always on (runs warmer)
    if (!WiFi.isConnected()) {
        esp_err_t err = esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
        if (err != ESP_OK) {
            ESP_LOGE(name(), "set_channel(%u): %s", channel_, esp_err_to_name(err));
            return false;
        }
    } else if (WiFi.channel() != channel_) {
        // associated: channel follows the AP, the conf value can't be applied
        ESP_LOGW(name(), "associated on ch%d, conf channel %u ignored - sync AP must share the STA channel",
                 WiFi.channel(), channel_);
    }
    wifi_promiscuous_filter_t f = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&f);
    esp_wifi_set_promiscuous_rx_cb(&rxCb);
    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        ESP_LOGE(name(), "promiscuous on: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool BeaconSyncService::onStart() {
    // conf first: these early failure returns must leave no side effects behind
    ConfFile c{"/littlefs/conf/bsync.conf", true};
    auto bs = c.getString("bssid", "");
    if (!parseBssid(bs, bssid_)) {
        ESP_LOGE(name(), "bssid missing/malformed ('%s'); set-config bsync.conf bssid aa:bb:cc:dd:ee:ff",
                 bs.c_str());
        return false;
    }
    channel_ = c.getByte("channel", 1);
    if (channel_ < 1 || channel_ > 14) {
        ESP_LOGE(name(), "channel %u out of range", channel_);
        return false;
    }
    phaseUs_ = c.getFloat("phase_us", 0.f);
    // bw scales every loop bandwidth together, damping-preserving (first-order gains ~bw,
    // integral/rate gains ~bw²). <1 trades lock/reacquire speed for less phase breathing:
    // the crystals only drift at ppb/s, so the loop can average much harder than default.
    bw_ = std::clamp(c.getFloat("bw", 1.0f), 0.05f, 4.0f);
    kp_ = c.getFloat("kp", 5e-6f) * bw_;
    ki_ = c.getFloat("ki", 2.5e-7f) * bw_ * bw_;
    alphaA_ = std::min(0.3f * bw_, 0.9f);
    alphaB_ = 0.05f * bw_ * bw_;
    hwOnly_ = c.getByte("hw_only", 0) != 0;

    leg_ = converter.mcpwmLeg();
    if (!leg_ || !leg_->periodTicks) {
        leg_ = nullptr;
        ESP_LOGE(name(), "needs the mcpwm gate driver (converter.conf pwm_driver=mcpwm)");
        return false;
    }
    if (converter.wsyncFollower) {
        leg_ = nullptr;
        ESP_LOGE(name(), "wired-sync follower: the sync wire owns the period, bsync must stay off");
        return false;
    }
    // A `pwm-freq` change is in flight (posted, not yet applied by the RT core). Latching
    // nomPeriod_ now would cache the period that is about to be replaced, and the servo would then
    // dither around a stale nominal. requestPwmFrequency() refuses once bsyncOwnsPeriod is set, so
    // this is the other half of that mutual exclusion.
    if (!converter.pwmFreqIdle()) {
        leg_ = nullptr;
        ESP_LOGE(name(), "a switching-frequency change is pending, retry once it has landed");
        return false;
    }
    if (leg_->resolutionHz % 1000000u != 0) {
        // the phase grid math assumes an integral tick-per-us rate (true for 160 MHz)
        ESP_LOGE(name(), "tick rate %lu not integral per us", (unsigned long) leg_->resolutionHz);
        leg_ = nullptr;
        return false;
    }
    nomPeriod_ = leg_->periodTicks;
    ticksPerUs_ = leg_->resolutionHz / 1000000u;

    portENTER_CRITICAL(&mux_);
    estInit_ = false;
    est_ = {};
    rejStreak_ = 0;
    rxExt_ = 0;
    rxLast_ = 0;
    espLast_ = 0;
    rxInit_ = false;
    dEst_ = 0;
    dInit_ = false;
    revaling_ = false;
    revalCnt_ = 0;
    portEXIT_CRITICAL(&mux_);
    nRejected_ = nClockDomain_ = nTsfDead_ = 0;
    nReval_ = nRevalStep_ = 0;
    rSmooth_ = 0;
    rSmoothInit_ = false;
    iAcc_ = 0;
    uCmd_ = 0.5f;
    sdCarry_ = 0;
    sdOut_ = -1;
    lastErrUs_ = NAN;
    lock_ = Lock::Acquiring;
    lastServoUs_ = lastRadioRetryUs_ = 0;

    inst_ = this;
    if (!radioUp()) {
        onStop(); // undo partial radio state (PS_NONE / STA mode) applied before the failure
        return false;
    }

    esp_timer_create_args_t ta = {
        .callback = &ditherCb,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "bsync_sd",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&ta, &ditherTimer_) != ESP_OK ||
        esp_timer_start_periodic(ditherTimer_, 1000) != ESP_OK) {
        ESP_LOGE(name(), "dither timer create/start failed");
        onStop();
        return false;
    }
    // Claim the period: requestPwmFrequency() refuses while this is set. Last thing before the
    // success return, so no failure path can leave it claimed.
    converter.bsyncOwnsPeriod = true;
    ESP_LOGI(name(), "sniffing %s ch=%u grid=%u+0.5 ticks @%lu Hz phase=%+.1f us bw=%.2f kp=%.2g ki=%.2g A=%.3f%s",
             bs.c_str(), channel_, nomPeriod_, (unsigned long) leg_->resolutionHz, phaseUs_, bw_, kp_, ki_, alphaA_,
             hwOnly_ ? " hw_only" : "");
    return true;
}

void BeaconSyncService::onStop() {
    converter.bsyncOwnsPeriod = false;
    if (ditherTimer_) {
        esp_timer_stop(ditherTimer_);
        esp_timer_delete(ditherTimer_);
        ditherTimer_ = nullptr;
    }
    if (leg_) leg_->setPeriodTicks(nomPeriod_);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    inst_ = nullptr;
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (radioStartedByUs_ && !WiFi.isConnected()) WiFi.mode(WIFI_OFF);
    radioStartedByUs_ = false;
    lock_ = Lock::Acquiring;
}

void BeaconSyncService::servo(int64_t nowUs) {
    // pair-read local clock + timer count, skew-bounded (esp_timer µs, same domain as the
    // bridged beacon stamps — no wifi API in the loop, works unassociated)
    int64_t w = 0;
    uint32_t cnt = 0;
    int64_t skew = INT64_MAX;
    for (int i = 0; i < 3; ++i) {
        int64_t w1 = esp_timer_get_time();
        uint32_t cc = leg_->count();
        int64_t w2 = esp_timer_get_time();
        if (w2 < w1) continue;
        if (w2 - w1 < skew) {
            skew = w2 - w1;
            w = (w1 + w2) / 2;
            cnt = cc;
        }
        if (skew <= 4) break;
    }

    Est e;
    bool init;
    portENTER_CRITICAL(&mux_);
    e = est_;
    init = estInit_;
    portEXIT_CRITICAL(&mux_);

    int64_t age = w - e.t;
    bool fresh = init && e.nBeacons >= 5 && age >= 0 && age < 3'000'000;
    // smooth the drift feedforward (~30s EWMA at 1Hz servo ticks): raw e.r is per-frame noisy
    // and must not modulate the frequency directly
    if (init && e.nBeacons >= 5) {
        if (!rSmoothInit_) {
            rSmooth_ = e.r;
            rSmoothInit_ = true;
        } else {
            rSmooth_ += (e.r - rSmooth_) / 30.0;
        }
    }
    if (!fresh) {
        // coast on the learned frequency trim (smoothed drift feedforward + I residual); no
        // phase corrections without a timebase
        lock_ = (init && e.nBeacons >= 5) ? Lock::Coasting : Lock::Acquiring;
        uCmd_ = (float) std::clamp(0.5 + (double) nomPeriod_ * rSmooth_ + iAcc_, 0.0, 1.0);
        return;
    }

    double oPred = e.o + e.r * (double) age;
    double b = (double) w - oPred - (double) phaseUs_; // shared (AP-clock) time, µs

    // target phase on the P+0.5-tick grid, in half-ticks (G = 2P+1 keeps the modulus integral;
    // the integer/fraction split keeps doubles exact for multi-year AP uptimes)
    int64_t G = 2 * (int64_t) nomPeriod_ + 1;
    int64_t htpu = 2 * (int64_t) ticksPerUs_;
    auto bi = (int64_t) b;
    double bf = b - (double) bi;
    int64_t ti = ((bi % G) * htpu % G + G) % G;
    double target = (double) ti + bf * (double) htpu;

    double e2 = 2.0 * (double) cnt - target;           // phase error, half-ticks
    e2 = std::fmod(e2, (double) G);
    if (e2 > (double) G / 2) e2 -= (double) G;
    else if (e2 < (double) -G / 2) e2 += (double) G;
    double eTicks = e2 * 0.5;
    lastErrUs_ = (float) (eTicks / (double) ticksPerUs_);

    double dt = lastServoUs_ ? std::clamp((double) (nowUs - lastServoUs_) * 1e-6, 0.2, 5.0) : 1.0;
    iAcc_ = std::clamp(iAcc_ + (double) ki_ * eTicks * dt, -0.45, 0.45);
    // frequency feedforward from the measured drift: the P-term's pull-in range is only a few
    // ppm and the wrapped phase error integrates to ~zero, so the crystal offset (tens of ppm)
    // must come from r (smoothed), not the PI
    uCmd_ = (float) std::clamp(0.5 + (double) nomPeriod_ * rSmooth_ + (double) kp_ * eTicks + iAcc_, 0.0, 1.0);
    lock_ = std::abs(lastErrUs_) < 5.0f ? Lock::Locked : Lock::Acquiring;
}

void BeaconSyncService::onTick() {
    int64_t now = esp_timer_get_time();
    if (now - lastServoUs_ < 1'000'000) return;

    // self-heal: `wifi off` / reconnect churn can stop the driver or clear promiscuous under us.
    // Deliberately overrides `wifi off`: sniffing is RX-only (no association, no TX), which is
    // the whole point of beacon sync during radio-quiet measurements. Full radio silence needs
    // `svc off bsync`. Say so loudly instead of silently re-arming the radio.
    // association (WiFi.begin) resets power-save to MIN_MODEM, which sleeps through most
    // beacons and degrades rx-stamp precision; keep forcing it off while we run
    wifi_ps_type_t ps;
    if (esp_wifi_get_ps(&ps) == ESP_OK && ps != WIFI_PS_NONE) esp_wifi_set_ps(WIFI_PS_NONE);

    bool promisc = false;
    esp_wifi_get_promiscuous(&promisc);
    if ((WiFi.getMode() == WIFI_MODE_NULL || !promisc) && now - lastRadioRetryUs_ > 10'000'000) {
        lastRadioRetryUs_ = now;
        ESP_LOGW(name(), "%s", g_app.disableWifi
                     ? "wifi is off; keeping RX-only sniffer radio up (`svc off bsync` for full radio silence)"
                     : "radio/promiscuous down, re-enabling sniffer");
        radioUp();
    }

    servo(now);
    lastServoUs_ = now;
}

std::string BeaconSyncService::statusDetail() const {
    Est e;
    bool init;
    int64_t d;
    portENTER_CRITICAL(&mux_);
    e = est_;
    init = estInit_;
    d = dEst_;
    portEXIT_CRITICAL(&mux_);
    float age = init ? (float) (esp_timer_get_time() - e.t) * 1e-6f : NAN;
    const char *st = lock_ == Lock::Locked ? "locked" : lock_ == Lock::Coasting ? "coasting" : "acquiring";
    char buf[144];
    snprintf(buf, sizeof buf,
             "%s e=%+.1fµs u=%.3f drift=%+.1fppm beacons=%u age=%.1fs rej=%u dom=%u rv=%u/%u d=%.0fµs",
             st, lastErrUs_, (double) uCmd_, e.r * 1e6, (unsigned) e.nBeacons, age,
             (unsigned) nRejected_, (unsigned) nClockDomain_, (unsigned) nReval_, (unsigned) nRevalStep_,
             (double) d);
    return buf;
}

#endif // HAVE_BSYNC
