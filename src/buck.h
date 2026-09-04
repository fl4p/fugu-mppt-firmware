#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <atomic>


// Gate-driver back-ends. The MCPWM driver and a "legacy" LEDC-style driver (LEDC on real
// hardware, or the VConv simulation plant / Mock) can both be compiled in. When both are
// present the active one is picked at runtime from converter.conf::pwm_driver (default ledc);
// when only one is compiled the dispatch below folds to a direct call. See the drv*() helpers.
#ifndef MOCK

#include <Arduino.h>
//#include "pinconfig.h"
#if WITH_VCONV
#include "pwm/vconv.h"
using LegacyPwm = PWM_VConv;
#define HAVE_LEGACY 1
#elif WITH_LEDC
#include "pwm/ledc.h"
using LegacyPwm = PWM_ESP32_ledc;
#define HAVE_LEGACY 1
#endif
#if WITH_MCPWM
#include "pwm/mcpwm.h"
#define HAVE_MCPWM 1
#if WITH_WSYNC
#include <driver/pulse_cnt.h>
#include "pwm/wsync_usb.h"
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
#include <driver/usb_serial_jtag.h>
#endif
#endif
#endif

#else // MOCK

#include "pwm/mock.h"
using LegacyPwm = PWM_Mock;
#define HAVE_LEGACY 1

#endif

#if !defined(HAVE_MCPWM) && !defined(HAVE_LEGACY)
#error "no gate driver compiled: enable FUGU_WITH_LEDC and/or FUGU_WITH_MCPWM (or MOCK / WITH_VCONV)"
#endif

#include "conf.h"
#include "util.h"
#include "logging.h"
#include "pwm/mcpwm_timing.h"   // rectOffset{Counts,Ns} helpers (no hardware deps)

/**
 * Synchronous buck or boost converter
 */
class SynchronousConverter {
    static constexpr uint8_t pwmCh_Ctrl = 0; // Ctrl FET; buck: IN, HI (HS signal)
    static constexpr uint8_t pwmCh_Rect = 1; // Rect FET; buck: EN or LO, depending on `pwmEnLogic`

    /**
     * instead of the %µi-curve we simply use a constant drop factor
     * this appears to be sufficient for CCM/DCM decision
     * notice that the more accurate we model coil current, the higher the efficiency in DCM and near-DCM condition
     */
    static constexpr float InductivityDcBias = 0.95f;

    // --- diode-emulation tuning (see doc/Diode Emulation.md) ---
    // DCM is entered when half the ripple exceeds the dc current (ΔI/2 > Io, i.e. ir > 2·Io).
    // Hysteresis: once in DCM, stay there until ir drops below 1.8·Io to avoid mode chatter.
    static constexpr float DcmEnterRippleRatio = 2.0f;
    static constexpr float DcmExitRippleRatio = 1.8f;
    // Below this dc current the converter is treated as DCM regardless of the ripple comparison.
    static constexpr float DcmForceCurrent = 0.1f; // A
    // In DCM, fully disable sync rectification below these: the body diode handles the small
    // discharge and this avoids reverse current at near-zero load / output.
    static constexpr float SyncRectOffCurrent = 0.01f; // A
    static constexpr float SyncRectOffVoltage = 1.0f; // V
    // Minimum side voltage for the M=vl/vh (buck) / vh/vl (boost) ratio to be trustworthy.
    static constexpr float MinRatioVoltage = 0.1f; // V
    // M clamp: floor keeps rectCtrlRatio() finite (no div-by-zero); UnityRatioMargin keeps M off
    // 1.0 so rectCtrlRatio() stays finite and positive on both sides.
    static constexpr float MinVoltageRatio = 1e-2f;
    static constexpr float MaxBoostRatio = 10.f;
    static constexpr float UnityRatioMargin = 1e-2f;


#if HAVE_MCPWM
    MCPWM_SyncLeg mcpwmDrv;
    MCPWM_FaultBrake faultBrake;
#endif
#if HAVE_LEGACY
    LegacyPwm legacyDrv;
#endif
#if defined(HAVE_MCPWM) && defined(HAVE_LEGACY)
    bool useMcpwm = false; // converter.conf::pwm_driver, default ledc (legacy); set in init()
#elif defined(HAVE_MCPWM)
    static constexpr bool useMcpwm = true;
#else
    static constexpr bool useMcpwm = false;
#endif
    uint16_t driverPwmMax = 0; // pwmMax of the active driver, cached after drvInit()
    const char *driverName = "?";

    bool pwmEnLogic = false; // whether the driver has a IN/SD input logic (instead of HI/LO)
    uint8_t pinSd = 255;
    bool dcmHysteresis = false;
    bool syncRectEnabled = false;

    uint16_t pwmCtrl = 0; // buck: HS
    uint16_t pwmRect = 0; // buck: LS
    uint16_t pwmRectMax = 0;
    int32_t manualRect = -1; // >=0: hold LS at this count (bench), -1: auto diode emulation
    float pwmRectRatioDCM = 0; // t_onRect/t_onCtrl when in DCM
    float rectDitherErr = 0; // carried LS-count rounding remainder (DCM error-feedback dither)
    bool rectDither = true; // false: plain round() (fallback)
    int16_t rectOnOffset = 0; // DCM LS-count dead-time/gate-delay offset (counts; >0 = LS off later, toward zero crossing)

    float outInVoltageRatio = 0; // M
    float directionFloatBuffer = 0.0f; // fractional perturbation buffer

    bool isBoost = false;
    bool forcedPwm = false;

    float fL = NAN; // fsw * L
    float coilL0 = 0; // retained from coil.conf so logConfig() can re-emit the config line
    uint16_t rectRefreshTicks = 0; // bootstrap-refresh pulse in timer ticks; pwmRectMin minus dead-time

    // Runtime dead-time change (console `dt`), posted from core 0 and applied on the RT core so
    // pwmMax and the gate comparators never move under the control path. One word — [31:22] seq,
    // [21:11] hl, [10:0] lh — so the triple is a coherent snapshot; ack==req means idle. seq
    // (never 0 once posted) rather than a sentinel tick count, so "no request" is not a valid
    // dead-time. 11 bits per value is enough: both are producer-clamped to periodTicks/32 <= 2047.
    //
    // This is a latest-value MAILBOX, not a queue, and it assumes ONE producer: a second post
    // overwrites an unapplied request instead of being rejected or queued. That holds today because
    // every console transport (UART, USB-CDC, telnet, MQTT, BLE) dispatches on core 0 through the
    // same handleCommand(), and `dt` waits for the ack before returning. A second producer, or a
    // post issued after the CLI's 1 s ack timeout gave up, would silently drop the older request.
    std::atomic<uint32_t> dtReqWord{0}, dtAckWord{0};
    static constexpr uint32_t kDtTickMask = 0x7ffu;   // 11 bits, >= 65535/32
    static constexpr uint32_t kDtSeqMask  = 0x3ffu;   // 10 bits

    // Runtime switching-frequency change (console `pwm-freq`), same mailbox contract as the
    // dead-time above: ONE core-0 producer, applied on the RT core so periodTicks/pwmMax and the
    // gate comparators never move under the control path. Latest-value mailbox, not a queue.
    // [31:20] seq (never 0 once posted), [19:0] the requested frequency in Hz — the producer bounds
    // it under 500 kHz, so it fits.
    //
    // The NOMINAL Hz is what travels, not the period: pwmFrequency and fL (the ripple basis, and so
    // the CCM/DCM decision) are written from it, and the period is a pure function of it at a fixed
    // prescaler. Carrying the period and the Hz in separate words would let a second request posted
    // after the CLI's 1 s ack timeout give up pair request N's period with request N+1's Hz — the
    // exact tear the dead-time mailbox packs its triple into one word to avoid.
    std::atomic<uint32_t> freqReqWord{0}, freqAckWord{0};
    static constexpr uint32_t kFreqHzMask = 0xfffffu;   // 20 bits, >= 500000
    static constexpr uint32_t kFreqSeqMask = 0xfffu;    // 12 bits

    // ---- gate-driver dispatch ----------------------------------------------------------------
    // One place per distinct driver operation. With both drivers compiled these branch on
    // useMcpwm at runtime; with a single driver the unreachable arm is #if'd out and the call
    // folds to the same code as before. MCPWM commits both comparators on TEZ (glitch-free,
    // order-independent); the LEDC path keeps its two-write ordering dance.
    void drvInit(uint8_t pinCtrl, uint8_t pinRect, const ConfFile &boardConf,
                 const std::string &syncRole, float syncPhaseNs) {
#if !WITH_WSYNC
        (void) syncRole; (void) syncPhaseNs;
#endif
#if HAVE_MCPWM
        if (useMcpwm) {
            // bestTiming picks the max period_ticks the hw can give at pwmFrequency; rect_offset is
            // stored as ns and converted to counts later, so it survives the resolution change.
            uint32_t resolutionHz = bestTiming(pwmFrequency).resolution_hz;
            // InEn drivers do dead-time in the gate-driver chip, so the MCPWM dt submodule stays off.
            // One value per transition: hl (ctrl-off -> rect-on) is the hardware RED, lh (rect-off ->
            // ctrl-on at the wrap) is the pwmMax reservation. Both default to the common key.
            // The common key is validated ONCE here, so a bad per-edge key can fall back to a value
            // already known to be sane rather than to zero.
            const float dtCommonRaw = boardConf.getFloat("pwm_deadtime_ns", 0.f);
            const bool commonOk = (dtCommonRaw >= 0.f && dtCommonRaw <= 1e6f); // also rejects NaN
            if (!commonOk)
                ESP_LOGE("converter", "pwm_deadtime_ns=%.1f is out of range, using 0", dtCommonRaw);
            const float dtCommonNs = commonOk ? dtCommonRaw : 0.f;
            // Same ceiling the runtime `dt` command enforces; a dead-band worth more than a
            // thirty-second of the period is a misconfiguration, not a tuning choice. A wired-sync
            // follower runs a period shortened by wsyncLeadTicks, and the runtime guard measures
            // against THAT period -- take the shorter one here too, or a value accepted at boot can
            // be one count above what `dt` will re-accept later.
            uint32_t ceilPeriod = bestTiming(pwmFrequency).period_ticks;
#if WITH_WSYNC
            if (ceilPeriod > MCPWM_SyncLeg::wsyncLeadTicks) ceilPeriod -= MCPWM_SyncLeg::wsyncLeadTicks;
#endif
            const uint32_t dtCeil = ceilPeriod / 32;
            // `hlEdge`: the realized gap is one tick short (the HS generator carries the 1-tick FED
            // that claims the dead-time path), so 1 tick is not a small gap but none at all. The lh
            // band is periodTicks - cmpLS and carries no such fixed offset.
            auto parseDt = [&](const char *key, bool hlEdge) -> uint32_t {
                const float raw = boardConf.getFloat(key, dtCommonNs);
                // Bound before the conversion: NaN or an absurd value would make lround UB, and a
                // tick count above the period underflows pwmMax = periodTicks - lh to ~65000, which
                // destroys every duty calculation downstream. 1 ms is already many periods.
                //
                // Falling back to 0 here would fail OPEN: on a HiLi board `hl=0` bypasses the
                // dead-time module entirely, so one typo'd key would boot the half-bridge with HS
                // going low and LS going high on the same comparator event. Fall back to the
                // validated common value instead -- an explicit 0 stays legal, a malformed value
                // does not silently become one.
                const bool ok = (raw >= 0.f && raw <= 1e6f);
                const float ns = ok ? raw : dtCommonNs;
                if (!ok)
                    ESP_LOGE("converter", "%s=%.1f is out of range, falling back to pwm_deadtime_ns=%.1f",
                             key, raw, dtCommonNs);
                if (pwmEnLogic) return 0u;
                auto ticks = (uint32_t) std::lround(ns * 1e-9f * (float) resolutionHz);
                if (ticks > dtCeil) {
                    ESP_LOGE("converter", "%s=%.1f is %u ticks, clamping to %u", key,
                             ns, (unsigned) ticks, (unsigned) dtCeil);
                    ticks = dtCeil;
                }
                if (hlEdge && ticks == 1) {
                    ticks = 2;
                    ESP_LOGW("converter", "%s rounds to 1 tick (= no gap), using 2", key);
                } else if (ticks == 0 && ns > 0.f) {
                    // Asked for a dead-time, quantized away to nothing. Silence here would leave the
                    // module bypassed for good, and `dt` refuses to arm it at runtime.
                    ESP_LOGW("converter", "%s=%.1f is under one tick, no dead-band", key, ns);
                }
                // Not an `else`: the 1 -> 2 bump above lands at a 1-tick realized gap, which is
                // exactly the case most in need of this warning.
                if (ticks > 0) {
                    const float gapNs = (float) (ticks - (hlEdge ? 1u : 0u)) * 1e9f / (float) resolutionHz;
                    if (gapNs < 50.f)
                        ESP_LOGW("converter", "%s=%.1f gives only %.0f ns realized gap, below the "
                                 "%.0f ns the runtime `dt` command would accept", key, ns, gapNs, 50.f);
                }
                return ticks;
            };
            const uint32_t dtHlTicks = parseDt("pwm_deadtime_hl_ns", true);
            const uint32_t dtLhTicks = parseDt("pwm_deadtime_lh_ns", false);
            bool syncFollower = false;
            uint8_t faultPin = boardConf.getByte("pwm_fault_pin", 255);
#if WITH_WSYNC
            // Role is decided BEFORE mcpwmDrv.init(): syncFollower is not a late detail, it
            // configures the leg (2-tick lead, and period/dead-time/comparator updates on sync).
            std::string role = syncRole;
            syncFollower = role == "follower";
            uint8_t syncPin = 255;
            if (role != "none") {
                syncPin = boardConf.getByte("pwm_sync_pin", 255);
                // Every pin/role guard runs while USB is still alive: a throw after the pad is
                // handed over would strand a header-less board with no console at all.
                assert_throw(syncPin != 255, "pwm_sync_pin missing in board.conf");
                assert_throw(syncFollower ? GPIO_IS_VALID_GPIO(syncPin) : GPIO_IS_VALID_OUTPUT_GPIO(syncPin),
                             "pwm_sync_pin invalid");
                assert_throw(syncPin != pinCtrl && syncPin != pinRect && syncPin != pinSd
                             && syncPin != faultPin, "pwm_sync_pin collides");
            }
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
            // Sync on a USB pad (boards with no GPIO header): the pad serves the USB-Serial-JTAG
            // PHY or the GPIO matrix, never both, so pick here and fall back to USB -- the
            // recoverable state -- unless a leader is positively qualified.
            if (syncFollower && wsyncPinIsUsb(syncPin)) {
                // `wsync arm` overrides only the host pre-check, never the qualification below:
                // arming a follower with no leader is the hazard this whole path exists to avoid.
                if (!wsyncArmRequest && usb_serial_jtag_is_connected()) {
                    // SOF activity, i.e. a host is really there. Never take the pad from it.
                    wsyncMode = WsyncMode::usb_host_active;
                } else {
                    if (wsyncArmRequest)
                        ESP_LOGI("converter", "%s", "wsync arm: probing the pad despite USB");
                    wsyncUsbPadEnable(false);
                    auto pr = wsyncQualifyLine(syncPin, (float) pwmFrequency);
                    ESP_LOGI("converter", "wsync usb-pin probe: %d edges, %.2f kHz, %s",
                             pr.edges, pr.rateHz * 1e-3f, pr.qualified ? "qualified" : "rejected");
                    if (pr.qualified) {
                        wsyncMode = WsyncMode::armed_follower;
                    } else {
                        wsyncUsbPadEnable(true);
                        wsyncMode = pr.edges ? WsyncMode::probe_bad_rate : WsyncMode::probe_no_edges;
                    }
                }
                if (wsyncMode != WsyncMode::armed_follower) {
                    // Downgrade so the sync block below self-skips: wsyncFollower stays false,
                    // bsync is free to run, and `wsync` reports why via wsyncMode.
                    role = "none";
                    syncFollower = false;
                    ESP_LOGW("converter", "wired sync disabled, %s", wsyncModeStr(wsyncMode));
                }
            } else if (role != "none") {
                wsyncMode = syncFollower ? WsyncMode::armed_follower : WsyncMode::leader;
            }
#else
            if (role != "none") wsyncMode = syncFollower ? WsyncMode::armed_follower : WsyncMode::leader;
#endif
#endif
            mcpwmDrv.init(0, pwmFrequency, pinCtrl, pinRect, dtHlTicks, dtLhTicks, pwmEnLogic, 0,
                          syncFollower);
            if (faultPin != 255) {
                faultBrake.initGpio(0, faultPin, boardConf.getByte("pwm_fault_active_high", 0));
                faultBrake.bindLeg(mcpwmDrv.oper(), mcpwmDrv.genHS(), mcpwmDrv.genLS());
            }
#if WITH_WSYNC
            // Wired inter-chip clock sync, armed before start() so the timer never free-runs with
            // the sync half-configured. Leader: TEZ-locked pulse on pwm_sync_pin, sync_phase_deg
            // shifts the pulse (= follower period start) for interleaving. Follower: pulse edge
            // = period boundary, no shift.
            // syncPin and its guards were validated above, before the USB pad decision.
            if (role != "none") {
                float tickHz = (float) mcpwmDrv.resolutionHz;
                // pad edge counter for both roles (`wsync` cmd): follower = wire delivery check,
                // leader = self-check of its own pulse. Runs BEFORE the role init: enabling the
                // pad's input/output here routes the matrix to plain GPIO, and initSyncOut must
                // claim the output matrix LAST or the pulse never reaches the pad.
                pcnt_unit_config_t upc = {.low_limit = -32768, .high_limit = wsyncCountMax,
                                          .intr_priority = 0, .flags = {}};
                // accumulate across the 16-bit wrap: the hw counter resets to 0 at high_limit,
                // so without this a noise-multiplied edge rate (>3.4x nominal at 39 kHz) wraps
                // and reads back as a plausible healthy rate. The accumulator is maintained in
                // the limit-event ISR, which needs the high limit registered as a watch point;
                // pcnt_unit_clear_count() resets it along with the counter.
                upc.flags.accum_count = 1;
                ESP_ERROR_CHECK(pcnt_new_unit(&upc, &wsyncPcnt_));
                ESP_ERROR_CHECK(pcnt_unit_add_watch_point(wsyncPcnt_, wsyncCountMax));
                pcnt_chan_config_t cpc = {.edge_gpio_num = syncPin, .level_gpio_num = -1,
                                          .flags = {.invert_edge_input = 0, .invert_level_input = 0,
                                                    .virt_edge_io_level = 0, .virt_level_io_level = 1,
                                                    .io_loop_back = 0}};
                pcnt_channel_handle_t pch;
                ESP_ERROR_CHECK(pcnt_new_channel(wsyncPcnt_, &cpc, &pch));
                ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                             PCNT_CHANNEL_EDGE_ACTION_HOLD));
                // level input unused: KEEP on both levels, else the (virtual) level line gates counting
                ESP_ERROR_CHECK(pcnt_channel_set_level_action(pch, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                              PCNT_CHANNEL_LEVEL_ACTION_KEEP));
                ESP_ERROR_CHECK(pcnt_unit_enable(wsyncPcnt_));
                ESP_ERROR_CHECK(pcnt_unit_clear_count(wsyncPcnt_));
                ESP_ERROR_CHECK(pcnt_unit_start(wsyncPcnt_));
                if (!syncFollower)
                    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t) syncPin, GPIO_MODE_INPUT_OUTPUT));
                if (syncFollower) {
                    // follower reload is fixed at count 0 (see initSyncIn); phase shifts live on the leader
                    if (syncPhaseNs != 0)
                        ESP_LOGW("converter", "%s", "sync_phase_* is leader-only, ignored on follower");
                    // armed here, before start(): both gates are still idle, so the one
                    // unavoidable arbitrary-phase jump (first lock) happens with nothing switching
                    mcpwmDrv.initSyncFilter(syncPin);
                    mcpwmDrv.initSyncIn(syncPin, pwmEnLogic);
                    wsyncFollower = true;
                } else {
                    int32_t ph = (int32_t) (std::lround(syncPhaseNs * 1e-9f * tickHz) % mcpwmDrv.periodTicks);
                    if (ph < 0) ph += mcpwmDrv.periodTicks;
                    // ~1 us pulse: long vs gate edges and scope-friendly, short vs the receiver
                    // RC bias droop (~8 us)
                    auto pulseTicks = (uint16_t) std::max(1l, std::lround(1e-6f * tickHz));
                    mcpwmDrv.initSyncOut(syncPin, pulseTicks, (uint16_t) ph);
                    ESP_LOGI("converter", "wired sync leader pulse offset %ld ticks", (long) ph);
                }
                ESP_LOGI("converter", "wired sync %s pin=%u", role.c_str(), syncPin);
            }
#endif
            mcpwmDrv.start();
            // Boot latched-low (consistent with disabled(), pwmCtrl==0); first protected
            // pwmPerturb(+) clears it. Without this, InEn boards (no pwm_sd) would switch early.
            mcpwmDrv.forceShutdown();
            driverPwmMax = mcpwmDrv.pwmMax;
            driverName = mcpwmDrv.name;
            return;
        }
#endif
#if HAVE_LEGACY
        legacyDrv.init_pwm(pwmCh_Ctrl, pinCtrl, pwmFrequency);
        legacyDrv.init_pwm(pwmCh_Rect, pinRect, pwmFrequency);
        driverPwmMax = legacyDrv.pwmMax;
        driverName = legacyDrv.name;
#endif
    }

    // Per-tick HS+LS commit. `enabling`: converter just left the disabled state. `largerDecrease`
    // / `direction`: LEDC two-write ordering hints (ignored by MCPWM).
    void drvCommit(bool enabling, bool largerDecrease, int direction) {
#if HAVE_MCPWM
        if (useMcpwm) {
            mcpwmDrv.setHsOff(pwmCtrl);
            mcpwmDrv.setLsOff(pwmCtrl + pwmRect);
            if (enabling) mcpwmDrv.clearForce(); // release the boot/disable force latch
            (void) largerDecrease; (void) direction;
            return;
        }
#endif
#if HAVE_LEGACY
        if (largerDecrease) {
            if (pwmEnLogic) {
                legacyDrv.update_pwm(pwmCh_Rect, 0);
                legacyDrv.update_pwm(pwmCh_Ctrl, pwmCtrl);
                legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            } else {
                if (pinSd != 255) digitalWrite(pinSd, 1);
                else legacyDrv.update_pwm(pwmCh_Rect, 0);
                legacyDrv.update_pwm(pwmCh_Ctrl, pwmCtrl);
                legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
                if (pinSd != 255) digitalWrite(pinSd, 0);
            }
        } else if (direction < 0) {
            // update EN/LS before IN/HS on a decrease
            if (pwmEnLogic) legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            else legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
            legacyDrv.update_pwm(pwmCh_Ctrl, pwmCtrl);
        } else {
            // update IN/HS before EN/LS on an increase
            legacyDrv.update_pwm(pwmCh_Ctrl, pwmCtrl);
            if (pwmEnLogic) legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            else legacyDrv.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
        }
        (void) enabling;
#endif
    }

    // Commit only the LS off-count (instant clamp when an LS limit decreases).
    void drvCommitLsOff(uint16_t ctrl, uint16_t rect) {
#if HAVE_MCPWM
        if (useMcpwm) { mcpwmDrv.setLsOff(ctrl + rect); return; }
#endif
#if HAVE_LEGACY
        if (pwmEnLogic) legacyDrv.update_pwm(pwmCh_Rect, ctrl + rect);
        else legacyDrv.update_pwm(pwmCh_Rect, ctrl, rect);
#endif
    }

    // Force both gates low / park duty at 0.
    void drvShutdown() {
#if HAVE_MCPWM
        if (useMcpwm) {
            mcpwmDrv.forceShutdown();
            // Park comparators at 0 too: when later un-forced, the latched pre-disable duty
            // cannot resume for one period before pwmPerturb rewrites them.
            mcpwmDrv.setHsOff(0);
            mcpwmDrv.setLsOff(0);
            return;
        }
#endif
#if HAVE_LEGACY
        legacyDrv.stop(pwmCh_Ctrl, 0);
        legacyDrv.stop(pwmCh_Rect, 0);
#endif
    }

    // Bench diagnostic: hold LS high. LEDC-only.
    void drvShortLs() {
#if HAVE_MCPWM
        if (useMcpwm) {
            ESP_LOGW("buck", "shortLs() is a LEDC-only diagnostic; ignored for the MCPWM driver");
            return;
        }
#endif
#if HAVE_LEGACY
        {
            auto pwmChLS = boost() ? pwmCh_Ctrl : pwmCh_Rect;
            auto pwmChHS = !boost() ? pwmCh_Ctrl : pwmCh_Rect;
            legacyDrv.stop(pwmChLS, 1);
            legacyDrv.stop(pwmChHS, 0);
        }
#endif
    }

    [[nodiscard]] uint16_t drvDtHlTicks() const {
#if HAVE_MCPWM
        if (useMcpwm) return mcpwmDrv.getDtHlTicks();
#endif
        return 0;
    }

    [[nodiscard]] uint16_t drvDtLhTicks() const {
#if HAVE_MCPWM
        if (useMcpwm) return mcpwmDrv.getDtLhTicks();
#endif
        return 0;
    }

    // Dead-time tick rate. 0 means the active driver has no dead-time submodule (LEDC/mock),
    // which is also the "cannot convert ns<->ticks" answer — callers must treat it as unsupported
    // rather than silently dividing by it.
    [[nodiscard]] uint32_t drvDtResolutionHz() const {
#if HAVE_MCPWM
        if (useMcpwm) return mcpwmDrv.resolutionHz;
#endif
        return 0;
    }

    // Timer period in ticks, the ceiling a dead-time must stay below. 0 = no MCPWM leg.
    [[nodiscard]] uint16_t drvPeriodTicks() const {
#if HAVE_MCPWM
        if (useMcpwm) return mcpwmDrv.periodTicks;
#endif
        return 0;
    }

public:
    SynchronousConverter() = default;

    SynchronousConverter(const SynchronousConverter &) = delete;

    SynchronousConverter &operator=(SynchronousConverter const &) = delete;


    [[nodiscard]] bool boost() const { return isBoost; }

    [[nodiscard]] bool forcedPwm_() const { return forcedPwm; }

    void forcedPwm_(bool forced) { forcedPwm = forced; }

    [[nodiscard]] bool syncRectEnabled_() const { return syncRectEnabled; }

    [[nodiscard]] const uint16_t &pwmMaxDriver() const { return driverPwmMax; };

#if HAVE_MCPWM
    // Raw leg access for the beacon-sync servo (period trim + live count); null when LEDC active.
    MCPWM_SyncLeg *mcpwmLeg() { return useMcpwm ? &mcpwmDrv : nullptr; }
#endif
    // true when this converter is a wired-sync follower (sync_role=follower): the wire owns
    // the period, so period-trimming servos (bsync) must not run.
    bool wsyncFollower = false;
    // true while the beacon-sync servo is running: it caches the nominal period at start and dithers
    // the period register at 1 kHz, so nothing else may change the period under it. Owned by
    // BeaconSyncService (set in onStart, cleared in onStop) — the guard lives with the producer so a
    // caller other than the console cannot miss it.
    bool bsyncOwnsPeriod = false;
#if WITH_WSYNC
    pcnt_unit_handle_t wsyncPcnt_ = nullptr;
    // Why wired sync ended up in the state it did. Without this, `wsync` cannot tell a USB-pad
    // fallback from a configured sync_role=none -- both simply have no edge counter.
    WsyncMode wsyncMode = WsyncMode::configured_none;
    // Set by setup() from the `wsync arm` NVS one-shot, before init(). Skips the USB host
    // pre-check for this boot only; the line still has to qualify. Owned by main.cpp so the
    // flag is consumed (cleared + committed) before anything here touches the pad.
    bool wsyncArmRequest = false;
#endif
    // A wired-sync edge counter exists (sync_role != none on a WSYNC build). Separate from the
    // count itself: a count is a valid non-negative number, so it cannot double as a sentinel.
    [[nodiscard]] bool wsyncHasCounter() const {
#if WITH_WSYNC
        return wsyncPcnt_ != nullptr;
#else
        return false;
#endif
    }

    // Sync-pin edge count, sampled between wsyncClear() and here. The 16-bit hw counter wraps
    // at wsyncCountMax, but the unit accumulates the overflow (see drvInit), so the returned
    // count is the true edge count and an over-rate reads high, not plausibly low.
    static constexpr int wsyncCountMax = 32767;

    void wsyncClear() {
#if WITH_WSYNC
        if (wsyncPcnt_) pcnt_unit_clear_count(wsyncPcnt_);
#endif
    }

    [[nodiscard]] int wsyncCount() const {
        int c = 0;
#if WITH_WSYNC
        if (wsyncPcnt_) pcnt_unit_get_count(wsyncPcnt_, &c);
#endif
        return c;
    }

    [[nodiscard]] uint16_t getDtHlTicks() const { return drvDtHlTicks(); }

    [[nodiscard]] uint16_t getDtLhTicks() const { return drvDtLhTicks(); }

    // The active driver can retune its dead-time at runtime. False is NOT "dead-time is 0" — it is
    // "there is no dead-time module to read or set", which callers must report as such.
    [[nodiscard]] bool hasDeadTime() const { return drvDtResolutionHz() != 0 && !pwmEnLogic; }

    // Configured HS->LS dead-time. The realized gap is one tick less: the HS generator carries a
    // 1-tick delay from claiming its dead-time path (see MCPWM_SyncLeg::setDeadTimeTicks).
    [[nodiscard]] float getDeadTimeHlNs() const {
        const uint32_t hz = drvDtResolutionHz();
        return hz ? (float) drvDtHlTicks() * 1e9f / (float) hz : 0.f;
    }

    // Configured LS->HS dead-time, carved out of pwmMax at the period wrap. No fixed hardware
    // offset like the hl edge; the realized band is one tick WIDER (>= lh+1), because every caller
    // caps cmpLS at pwmMax-1.
    [[nodiscard]] float getDeadTimeLhNs() const {
        const uint32_t hz = drvDtResolutionHz();
        return hz ? (float) drvDtLhTicks() * 1e9f / (float) hz : 0.f;
    }

    // Tick count a given ns request would quantize to, 0 if there is no dead-time module. Lets the
    // console compare like with like: 199 ns against a 200 ns setting is the SAME 32 ticks, so it
    // is not a decrease and must not be gated as one.
    [[nodiscard]] uint16_t deadTimeTicksFor(float ns) const {
        const uint32_t hz = drvDtResolutionHz();
        if (!hz || !(ns >= 0.f && ns <= 1e6f)) return 0;
        return (uint16_t) std::min<long>(std::lround(ns * 1e-9f * (float) hz), 65535);
    }

    // Console (core 0): queue a dead-time change, applied by the RT loop within one control tick
    // (poll deadTimeIdle()). Returns nullptr when queued, else a static reason. hlTicks/lhTicks
    // report the quantized values. RAM only -- board.conf::pwm_deadtime_{hl,lh}_ns still own the
    // boot values.
    const char *requestDeadTimeNs(float hlNs, float lhNs, uint16_t &hlTicks, uint16_t &lhTicks) {
        const uint32_t hz = drvDtResolutionHz();
        const uint16_t period = drvPeriodTicks();
        if (!hz || !period) return "needs the MCPWM driver (converter.conf::pwm_driver)";
        if (pwmEnLogic) return "InEn gate driver has its own dead-time (HiLi boards only)";
        // Arming a bypassed module puts both gates on the HS waveform between the two register
        // writes -- see MCPWM_SyncLeg::setDeadTimeTicks. Only retuning is safe.
        if (!drvDtHlTicks())
            return "module bypassed at boot; set board.conf pwm_deadtime_hl_ns and reboot";
        // Bound before lround: converting an out-of-range float to long is UB. 1 ms is already
        // dozens of periods, so nothing legitimate is lost.
        if (!(hlNs >= 0.f && hlNs <= 1e6f) || !(lhNs >= 0.f && lhNs <= 1e6f))
            return "expected dead-time in ns"; // also rejects NaN
        const long hl = std::lround(hlNs * 1e-9f * (float) hz);
        const long lh = std::lround(lhNs * 1e-9f * (float) hz);
        // Physical floor, not just the arithmetic one. The gap has to outlast the MOSFET turn-off
        // plus the gate driver's channel-to-channel skew; at 160 MHz the 2-tick minimum the driver
        // enforces is 6.25 ns realized, which shoots through on real silicon. kMinGapNs is
        // deliberately well below any dead-time we ship (200 ns) -- it exists to stop a typo, not
        // to express a tuned value. hl realizes one tick SHORT (the path-1 FED claim); lh realizes
        // one tick WIDE (cmpLS is capped at pwmMax-1), so its floor needs no correction and is
        // conservative by a tick.
        constexpr float kMinGapNs = 50.f;
        const long gapMin = (long) std::ceil(kMinGapNs * 1e-9f * (float) hz);
        if (hl < std::max(2L, gapMin + 1))
            return "hl below the minimum safe gap (realized gap must exceed FET turn-off)";
        if (lh < std::max(1L, gapMin))
            return "lh below the minimum safe gap (realized gap must exceed FET turn-off)";
        // Two ceilings. lh is reserved out of pwmMax, so it eats commandable duty; hl delays the LS
        // turn-on, so pwmRectMin grows with it (rectRefreshTicks + hl) and eats more. Both together
        // must stay a small part of the period.
        if (hl > period / 32 || lh > period / 32) return "above 1/32 of the switching period";
        if (!isBoost && (long) rectRefreshTicks + hl > period / 4)
            return "would leave too little duty range next to the LS minimum";
        hlTicks = (uint16_t) hl;
        lhTicks = (uint16_t) lh;
        uint32_t seq = ((dtReqWord.load(std::memory_order_relaxed) >> 22) + 1) & kDtSeqMask;
        if (!seq) seq = 1;
        dtReqWord.store((seq << 22) | ((uint32_t) hlTicks << 11) | lhTicks, std::memory_order_release);
        return nullptr;
    }

    [[nodiscard]] bool deadTimeIdle() const {
        return dtAckWord.load(std::memory_order_acquire) == dtReqWord.load(std::memory_order_acquire);
    }

    // RT-CORE ONLY. Applies a queued dead-time change, pulling the live comparators inside the new
    // envelope. The clamp is split around the driver write -- widening before, capping after; see
    // the ordering note below. A driver call that fails after the widening leaves the LS span a
    // little wide for the dead-time still in force, which is harmless (more rectifier conduction,
    // still inside the unchanged pwmMax) and cannot happen for a request that passed the producer.
    void applyPendingDeadTimeRt() {
        const uint32_t req = dtReqWord.load(std::memory_order_acquire);
        if (req == dtAckWord.load(std::memory_order_relaxed)) return;
        const auto hl = (uint16_t) ((req >> 11) & kDtTickMask);
        const auto lh = (uint16_t) (req & kDtTickMask);
#if HAVE_MCPWM
        // Mirror the producer's ceiling, not just the driver's: newMax below 2 would make the
        // std::clamp() below run with hi < lo, which is UB.
        if (useMcpwm && hl >= 2 && hl <= mcpwmDrv.periodTicks / 32
            && lh >= 1 && lh <= mcpwmDrv.periodTicks / 32) {
            // rectOnOffset and rectRefreshTicks are counts of TIMER ticks, whose duration is set by
            // resolutionHz and does not move with the dead-time — they stay put. pwmRectMin does
            // follow, because it reserves the HS->LS dead-time on top of the refresh pulse (init()).
            const auto newMax = (uint16_t) (mcpwmDrv.periodTicks - lh);
            const auto newRectMin = isBoost ? (uint16_t) 0 : (uint16_t) (rectRefreshTicks + hl);
            // -1 because cmpLS must stay below the period (see computePwmRectMax): at
            // pwmCtrl == newCtrlMax the widest legal LS span is newMax - newCtrlMax - 1, and that
            // has to be newRectMin, not one short of it.
            const auto newCtrlMax = (uint16_t) std::clamp<int32_t>(
                isBoost ? (int32_t) ((float) newMax * 0.9f)
                        : (int32_t) newMax - (int32_t) newRectMin - 1,
                1, (int32_t) newMax - 1);
            // Both bands can tighten the envelope, independently: lh lowers the pwmMax ceiling, hl
            // raises the LS floor (it delays LS turn-on, so a span legal at the old dead-time now
            // realizes hl fewer ticks of conduction -- 353 counts at hl 32->128 realizes 225, well
            // below the measured 300-count bootstrap-refresh floor).
            const bool tightens = newMax < driverPwmMax || newRectMin > pwmRectMin;
            // Order matters, and it is not the same for the two directions. Both the delay
            // registers and the comparators latch on TEZ, so if no period boundary falls between
            // the two writes the change is atomic; if one does, a single period runs the new
            // dead-time against the old comparators.
            //
            // That transient is shoot-through-safe either way -- the LS->HS band is
            // periodTicks - cmpLS and touches no delay register, so the old cmpLS <= periodTicks -
            // oldLh - 1 keeps at least the OLD band -- but it is NOT bootstrap-safe when hl grows:
            // an LS span sitting at the old floor realizes rectRefreshTicks - (newHl - oldHl) for
            // that period. So widen the LS span BEFORE arming the longer delay. Capping (lh
            // growing) stays after the driver write: a shorter span is safe against the old
            // dead-time, and doing it late keeps a rejected driver call from stepping duty down
            // against a ceiling that never moved.
            if (tightens && newRectMin > pwmRectMin) {
                // Clamp against whichever pwmMax is in force in BOTH periods, so the pre-write
                // comparators are legal before and after the latch. pwmCtrl is pulled down first
                // with the same conservative ceiling, which guarantees newRectMin <= rectCap.
                const auto safeMax = std::min(driverPwmMax, newMax);
                const auto safeCtrlMax = (uint16_t) std::clamp<int32_t>(
                    isBoost ? (int32_t) ((float) safeMax * 0.9f)
                            : (int32_t) safeMax - (int32_t) newRectMin - 1,
                    1, (int32_t) safeMax - 1);
                pwmCtrl = std::min(pwmCtrl, safeCtrlMax);
                const auto rectCap = (uint16_t) (safeMax - pwmCtrl - 1);
                pwmRect = (uint16_t) std::clamp<int32_t>(pwmRect, newRectMin, rectCap);
                pwmRectMax = (uint16_t) std::clamp<int32_t>(pwmRectMax, newRectMin, rectCap);
                mcpwmDrv.setHsOff(pwmCtrl);
                mcpwmDrv.setLsOff(pwmCtrl + pwmRect);
            }
            esp_err_t err = mcpwmDrv.setDeadTimeTicks(hl, lh);
            if (err == ESP_OK) {
                if (tightens) {
                    // Re-run against the final limits. When only the floor rose this is a no-op
                    // (the pre-write clamp already used them); when the ceiling dropped it is the
                    // cap that had to wait for the driver write to land.
                    pwmCtrl = std::min(pwmCtrl, newCtrlMax);
                    const auto rectCap = (uint16_t) (newMax - pwmCtrl - 1);
                    pwmRect = (uint16_t) std::clamp<int32_t>(pwmRect, newRectMin, rectCap);
                    pwmRectMax = (uint16_t) std::clamp<int32_t>(pwmRectMax, newRectMin, rectCap);
                    mcpwmDrv.setHsOff(pwmCtrl);
                    mcpwmDrv.setLsOff(pwmCtrl + pwmRect);
                }
                driverPwmMax = mcpwmDrv.pwmMax;
                pwmRectMin = newRectMin;
                pwmCtrlMax = newCtrlMax;
                ESP_LOGI("converter", "dead-time hl=%u ct (%.0f ns) lh=%u ct (%.0f ns), "
                         "pwmMax=%u minLS=%u maxHS=%u",
                         (unsigned) hl, getDeadTimeHlNs(), (unsigned) lh, getDeadTimeLhNs(),
                         (unsigned) driverPwmMax, (unsigned) pwmRectMin, (unsigned) pwmCtrlMax);
            } else {
                ESP_LOGE("converter", "dead-time hl=%u lh=%u ct rejected: %s",
                         (unsigned) hl, (unsigned) lh, esp_err_to_name(err));
            }
        }
#endif
        dtAckWord.store(req, std::memory_order_release);
    }

    // True timer period in ticks (0 on drivers without one) and the tick rate it counts at.
    [[nodiscard]] uint16_t getPeriodTicks() const { return drvPeriodTicks(); }

    [[nodiscard]] uint32_t getPwmResolutionHz() const { return drvDtResolutionHz(); }

    // Realized switching frequency. Not the requested one: bestTiming() rounds the period to whole
    // ticks, and its own actual_freq is resolution/ticks in uint32 integer division, which truncates
    // (4103 ticks reports 38995 where the true value is 38995.86). This is the only honest source.
    [[nodiscard]] double realizedFreqHz() const {
        const uint32_t res = drvDtResolutionHz();
        const uint16_t p = drvPeriodTicks();
        return (res && p) ? (double) res / (double) p : (double) pwmFrequency;
    }

    // Console (core 0): queue a switching-frequency change, applied by the RT loop within one
    // control tick (poll pwmFreqIdle()). Returns nullptr when queued, else a static reason.
    // `ticks` reports the resulting timer period. RAM only — board.conf::pwm_freq still owns the
    // boot value, so a power cycle returns to a known frequency.
    const char *requestPwmFrequency(uint32_t hz, uint16_t &ticks) {
        const uint32_t res = drvDtResolutionHz();
        const uint16_t period = drvPeriodTicks();
        if (!res || !period) return "needs the MCPWM driver (converter.conf::pwm_driver)";
#if WITH_WSYNC
        // The leader's pulse comparators are absolute ticks written once in initSyncOut(), and a
        // follower's period is baked wsyncLeadTicks short in init(); neither has a re-arm path, and
        // the pad qualifier assumes both ends run the same pwm_freq.
        if (wsyncMode == WsyncMode::leader || wsyncMode == WsyncMode::armed_follower)
            return "wired sync owns the period (converter.conf::sync_role)";
#endif
        if (!(hz > 5000u && hz < 500000u)) return "expected 5..500 kHz";
        // The prescaler is fixed when the timer is created and setPeriod() writes the period
        // register only, so the resolution cannot follow. bestTiming keeps the prescaler at 1 across
        // the whole 5..500 kHz range on a 160 MHz source, so this never fires today — it guards the
        // day the source clock or the range changes, because a resolution change would silently
        // rescale rect_offset_ns, boot_refresh_ns and every dead-time tick count.
        if (bestTiming(hz).resolution_hz != res) return "would need a different timer prescaler";
        // Same layer as the wsync refusal above, and for the same reason: bsync caches nomPeriod_ at
        // start and dithers this very register at 1 kHz, so it would drive the period straight back.
        // The flag is owned by the service, so a second (non-CLI) producer cannot miss the guard.
        if (bsyncOwnsPeriod) return "bsync owns the period (svc off bsync first)";
        const long t = std::lround((double) res / (double) hz);
        if (t < 16 || t > 65535) return "period out of range for the timer";
        // Ripple guard, the runtime twin of the boot assert. It REFUSES rather than throwing: an
        // out-of-range pwm_freq in the config keeps the board from booting at all, which looks
        // exactly like a bad flash.
        const float fLnew = (float) hz * coilL0 * InductivityDcBias;
        if (!(fLnew > 1.f && fLnew < 20.f)) return "fsw*L0 out of range for this coil (coil.conf::L0)";
        // Both dead-bands are fixed tick counts, so a shorter period is what makes them too large.
        // Same ceilings the `dt` command enforces, measured against the NEW period.
        const long hl = drvDtHlTicks(), lh = drvDtLhTicks();
        if (hl > t / 32 || lh > t / 32) return "dead-time would exceed 1/32 of the new period";
        if (!isBoost && (long) rectRefreshTicks + hl > t / 4)
            return "would leave too little duty range next to the LS minimum";
        ticks = (uint16_t) t;
        uint32_t seq = ((freqReqWord.load(std::memory_order_relaxed) >> 20) + 1) & kFreqSeqMask;
        if (!seq) seq = 1;
        freqReqWord.store((seq << 20) | (hz & kFreqHzMask), std::memory_order_release);
        return nullptr;
    }

    [[nodiscard]] bool pwmFreqIdle() const {
        return freqAckWord.load(std::memory_order_acquire) == freqReqWord.load(std::memory_order_acquire);
    }

    // RT-CORE ONLY. Applies a queued frequency change and returns the count-domain scale that was
    // applied, 0 when nothing changed — duty targets held outside this class (MpptController's
    // manualTarget and its two captured MPPs) are raw counts and must be rescaled by it.
    float applyPendingPwmFreqRt() {
        const uint32_t req = freqReqWord.load(std::memory_order_acquire);
        if (req == freqAckWord.load(std::memory_order_relaxed)) return 0.f;
        float scale = 0.f;
#if HAVE_MCPWM
        const uint32_t hz = req & kFreqHzMask;
        const uint16_t oldTicks = useMcpwm ? mcpwmDrv.periodTicks : 0;
        // Derived here rather than carried, so the period and the Hz can never come from different
        // requests. Same inputs and same rounding as the producer, at a prescaler that cannot move.
        const long t = (useMcpwm && hz) ? std::lround((double) mcpwmDrv.resolutionHz / (double) hz) : 0;
        const auto newTicks = (uint16_t) t;
        const long hl = mcpwmDrv.getDtHlTicks(), lh = mcpwmDrv.getDtLhTicks();
        // Mirror the producer's ceilings, exactly as applyPendingDeadTimeRt() mirrors its own: they
        // are what makes newMax >= 2 and pwmRectMin <= rectCap, and a std::clamp() with hi < lo is
        // UB. The RT core must not depend on a producer it cannot see.
        if (useMcpwm && oldTicks && t >= 16 && t <= 65535
            && hl <= t / 32 && lh <= t / 32
            && (isBoost || (long) rectRefreshTicks + hl <= t / 4)) {
            const auto newMax = (uint16_t) (newTicks - (uint16_t) lh);
            // pwmRectMin does NOT scale: it is rectRefreshTicks + hl, both fixed times at a tick
            // whose duration the prescaler pins. So a rising frequency eats duty range at the top —
            // the clamp below is what the console reports back.
            const auto newCtrlMax = (uint16_t) std::clamp<int32_t>(
                isBoost ? (int32_t) ((float) newMax * 0.9f)
                        : (int32_t) newMax - (int32_t) pwmRectMin - 1,
                1, (int32_t) newMax - 1);
            const float r = (float) newTicks / (float) oldTicks;
            auto scaled = [r](int32_t v) { return (int32_t) std::lround((float) v * r); };
            // Scale the commanded counts so the DUTY RATIO survives, then clamp into the new
            // envelope. A disabled converter stays disabled: pwmCtrl == 0 is the "off" state, not a
            // duty to preserve, and pwmRectMin would otherwise lift the LS span off zero.
            const bool off = (pwmCtrl == 0);
            const auto newCtrl = off ? (uint16_t) 0
                                     : (uint16_t) std::clamp<int32_t>(scaled(pwmCtrl), 1, newCtrlMax);
            const auto rectCap = (uint16_t) (newMax - newCtrl - 1);
            // Backstop, not arithmetic: the ceilings above make rectCap >= pwmRectMin. std::clamp
            // with hi < lo is UB, and this runs on the RT core on a value that came via a mailbox.
            const auto rectLo = std::min(pwmRectMin, rectCap);
            const auto newRect = off ? (uint16_t) 0
                                     : (uint16_t) std::clamp<int32_t>(scaled(pwmRect), rectLo, rectCap);
            const auto newLsOff = off ? (uint16_t) 0 : (uint16_t) (newCtrl + newRect);

            // ORDER, twice over. The period register and both comparators latch on TEZ, so with no
            // period boundary between the writes the change is atomic; a TEZ falling between any two
            // of them publishes a MIXED pair, and both mixes have to be safe.
            //
            // (a) period vs comparators. Shrink: comparators first — a comparator left above a
            //     newly shortened period never fires and leaves LS high for a whole cycle (see
            //     setPeriod()). Grow: period first. IDF enforces this for us as it happens
            //     (mcpwm_comparator_set_compare_value rejects cmp > peak_ticks, and set_period
            //     updates peak immediately), but the reason stands on its own.
            //
            // (b) cmpHS vs cmpLS, which is the one that bites. In HiLi the LS generator has NO TEZ
            //     action (mcpwm.h: HIGH at cmpHS, LOW at cmpLS), so LS holds its level across the
            //     wrap. A latched pair with cmpLS < cmpHS therefore runs: cmpLS passes as a no-op,
            //     cmpHS drives LS HIGH, and at TEZ HS goes HIGH on top of it — BOTH FETS ON for most
            //     of a period. Rescaling moves cmpHS by hundreds of counts, so the mixed pair really
            //     can invert (75->39 kHz at duty 0.61: cmpHS 2464 against a stale cmpLS 2100).
            //     Write whichever comparator keeps the mixed pair ordered: LS first when HS widens,
            //     HS first when HS narrows. Both mixes then satisfy cmpLS >= cmpHS.
            auto commitPair = [&](uint16_t hs, uint16_t ls, uint16_t hsNow) {
                if (hs >= hsNow) { mcpwmDrv.setLsOff(ls); mcpwmDrv.setHsOff(hs); }
                else             { mcpwmDrv.setHsOff(hs); mcpwmDrv.setLsOff(ls); }
            };
            const bool shrink = newTicks < oldTicks;
            if (shrink) commitPair(newCtrl, newLsOff, pwmCtrl);
            esp_err_t err = mcpwmDrv.setPeriod(newTicks);
            if (err == ESP_OK) {
                if (!shrink) commitPair(newCtrl, newLsOff, pwmCtrl);
                pwmCtrl = newCtrl;
                pwmRect = newRect;
                pwmRectMax = (uint16_t) std::clamp<int32_t>(scaled(pwmRectMax), rectLo, rectCap);
                if (manualRect >= 0)
                    manualRect = std::clamp<int32_t>(scaled(manualRect), rectLo, rectCap);
                driverPwmMax = mcpwmDrv.pwmMax;
                pwmCtrlMax = newCtrlMax;
                scale = r;
                // float, not the double realizedFreqHz(): this runs on the RT core.
                ESP_LOGI("converter", "pwm period %u ct, pwmMax=%u maxHS=%u hs_off=%u ls_off=%u",
                         (unsigned) newTicks, (unsigned) driverPwmMax,
                         (unsigned) pwmCtrlMax, (unsigned) pwmCtrl, (unsigned) (pwmCtrl + pwmRect));
            } else {
                // Only reachable if the driver rejects what the producer already validated. On the
                // shrink path the comparators were narrowed already, so put them back — same
                // ordering rule, since restoring pwmCtrl WIDENS cmpHS again.
                if (shrink)
                    commitPair(pwmCtrl, pwmCtrl ? (uint16_t) (pwmCtrl + pwmRect) : (uint16_t) 0, newCtrl);
                ESP_LOGE("converter", "pwm period %u ct rejected: %s",
                         (unsigned) newTicks, esp_err_to_name(err));
            }
            // Nominal frequency and the ripple basis. Deliberately NOT gated on the period having
            // moved: two nearby requests can round to the same tick count (39000 and 39100 are both
            // 4103 ct) and the CLI reports that as success, so an early-out there would leave fL —
            // the CCM/DCM decision input — on the old frequency behind a success reply.
            if (err == ESP_OK) {
                pwmFrequency = hz;
                fL = (float) hz * coilL0 * InductivityDcBias;
            }
        }
#endif
        freqAckWord.store(req, std::memory_order_release);
        return scale;
    }

    [[nodiscard]] bool isEnLogic() const { return pwmEnLogic; }

    [[nodiscard]] uint32_t getPwmFrequency() const { return pwmFrequency; }

    uint16_t pwmCtrlMax{}, pwmRectMin{}, pwmCtrlMin{};
    uint32_t pwmFrequency{};

    [[nodiscard]] bool disabled() const { return pwmCtrl == 0; }

    [[nodiscard]] float getDutyCycle() const { return (float) pwmCtrl / (float) pwmCtrlMax; }

    [[nodiscard]] uint16_t getCtrlOnPwmCnt() const { return pwmCtrl; }

    [[nodiscard]] uint16_t getCtrlOnPwmMin() const { return pwmCtrlMin; }

    [[nodiscard]] uint16_t getRectOnPwmCnt() const { return pwmRect; }

    [[nodiscard]] uint16_t getLowSideOnPwmCnt() const { return isBoost ? pwmCtrl : pwmRect; }

    [[nodiscard]] uint16_t getLowSideMinPwmCnt() const { return isBoost ? 0 : pwmRectMin; }

    [[nodiscard]] uint16_t getRectOnPwmMax() const { return pwmRectMax; }

    [[nodiscard]] uint16_t getRectOnPwmMin() const { return pwmRectMin; }

    [[nodiscard]] int16_t getRectOnOffset() const { return rectOnOffset; }

    void setRectOnOffset(int o) { rectOnOffset = (int16_t) o; } // DCM LS dead-time offset, counts

    // Comparator counts per second = fsw * the full timer period. Used to convert the physical
    // rect_offset_ns / boot_refresh_ns <-> counts independent of driver resolution. It is the FULL
    // period, not pwmMax: the dead-time is carved out of pwmMax but does not change how long a tick
    // lasts, so a plain fsw*pwmMax would re-read a fixed count as a different time after a runtime
    // dead-time change. The lh ticks are 0 on drivers without a dead-time module, so this is
    // unchanged there.
    // Duration of one duty count, as a rate. On MCPWM this is the peripheral resolution itself:
    // deriving it as pwmFrequency * periodTicks is 106 ppm off (the realized fsw is not the nominal
    // one) and is wrong by 0.05 % on a wsync follower, whose period is shortened while its clock is
    // not. Either way it must not move with the dead-time, or rect_offset_ns and boot_refresh_ns
    // would silently rescale on a runtime `dt`.
    [[nodiscard]] float getPwmTickRate() const {
        const uint32_t hz = drvDtResolutionHz();
        return hz ? (float) hz : (float) pwmFrequency * (float) (driverPwmMax + drvDtLhTicks());
    }

    // Emitted from init() (boot serial) and re-fired once MQTT is up (so it lands in the MQTT/telnet
    // log too — init() runs during setup(), before those sinks exist).
    void logConfig() const {
        ESP_LOGI("converter", "Coil L0=%.1f µH rect_offset=%.0f ns (%d ct)",
                 coilL0 * 1e6f, rectOffsetNsFromCounts(rectOnOffset, getPwmTickRate()), rectOnOffset);
    }

    [[nodiscard]] float voltageRatio() const { return outInVoltageRatio; } // M

    [[nodiscard]] bool manualRect_() const { return manualRect >= 0; }

    // Bench: pin the low-side on-count to `ls`, held against the auto diode-emulation logic
    // (lets you sweep LS timing by hand). `ls<0` restores automatic control. Clamped to
    // [pwmRectMin, complementary max]. NOTE: exceeding the natural zero-crossing draws reverse
    // current - same envelope as forced PWM; protections stay active.
    void setManualRect(int ls) {
        if (ls < 0) {
            manualRect = -1;
            return;
        }
        // -1: cmpLS == pwmMax would land on the timer-wrap, never firing the turn-off event;
        // LS would stay HIGH the whole period.
        manualRect = constrain(ls, (int) pwmRectMin, (int) (driverPwmMax - pwmCtrl - 1));
        pwmRect = (uint16_t) manualRect;
        drvCommitLsOff(pwmCtrl, pwmRect);
    }

    void init(const ConfFile &converterConf, const ConfFile &boardConf, const ConfFile &coilConf) {
        auto topo = converterConf.getString("topo", "buck");
        assert_throw(topo == "buck" or topo == "boost", "");
        isBoost = topo == "boost";
        forcedPwm = converterConf.getByte("forced_pwm", 0);

        if (forcedPwm)
            ESP_LOGW("converter", "%s", "forced_pwm");

#if defined(HAVE_MCPWM) && defined(HAVE_LEGACY)
        {
            // Both drivers compiled: pick at runtime. Default ledc — an MCPWM board must opt in.
            auto drv = converterConf.getString("pwm_driver", "ledc");
            if (drv != "mcpwm" && drv != "ledc")
                throw std::runtime_error("unrecognized pwm_driver " + drv);
            useMcpwm = (drv == "mcpwm");
        }
#endif

        coilL0 = coilConf.getFloat("L0");
        const float L0 = coilL0;
        // rectOnOffset is derived from coil.conf::rect_offset_ns after the driver is up (needs pwmMax
        // for the ns->counts conversion); see the end of init().

        pwmFrequency = boardConf.getLong("pwm_freq"); //39000; //  converter switching frequency
        assert_throw(pwmFrequency > 5e3 && pwmFrequency < 5e5, "");

        fL = (float) pwmFrequency * L0 * InductivityDcBias; // for ripple current computation
        assert_throw(fL < 20, "pwmFreq*L0 out-of-range");
        assert_throw(fL > 1, "pwmFreq*L0 out-of-range");
        // Lo: https://www.ti.com/lit/ds/symlink/lm5163.pdf#page=18

        auto drvInpLogic = boardConf.getString("pwm_driver_logic"); // driver input logic "in,en", "hi,li" and en
        uint8_t pinCtrl, pinRect;


        if (drvInpLogic == "InEn") {
            // e.g. Infineon ir2814

            pwmEnLogic = true;
            auto pnCtrl = isBoost ? "pwm_en" : "pwm_in";
            auto pnRect = isBoost ? "pwm_in" : "pwm_en";

            pinCtrl = boardConf.getByte(pnCtrl);
            pinRect = boardConf.getByte(pnRect);
            assert_throw(pinCtrl != pinRect, "");

            if (!boardConf.getByte("skip_assert", 0)) {
                // ti, infineon gate drivers: in pins pulled low, EN/SD pins pulled high
                assertPinState(pinCtrl, false, pnCtrl, true);
                assertPinState(pinRect, false, pnRect, true);
            }
        } else if (drvInpLogic == "HiLi") {
            // e.g. TI UCC21330x with optional DIS pin (SD pin)
            auto pnCtrl = isBoost ? "pwm_li" : "pwm_hi";
            auto pnRect = isBoost ? "pwm_hi" : "pwm_li";

            pinCtrl = boardConf.getByte(pnCtrl);
            pinRect = boardConf.getByte(pnRect);
            pinSd = boardConf.getByte("pwm_sd", 255); // DIS

            assert_throw(pinCtrl != pinRect, "");
            assert_throw(pinCtrl != pinSd, "");
            assert_throw(pinRect != pinSd, "");

            if (!boardConf.getByte("skip_assert", 0)) {
                assertPinState(pinCtrl, false, pnCtrl, false);
                assertPinState(pinRect, false, pnRect, false);
                if (pinSd != 255) assertPinState(pinSd, true, "pwm_sd", false);
            }
        } else {
            throw std::runtime_error("unrecognized pwm_driver_logic " + drvInpLogic);
        }

        std::string syncRole = "none";
        float syncPhaseNs = 0;
#if WITH_WSYNC
        syncRole = converterConf.getString("sync_role", "none");
        if (syncRole != "none" && syncRole != "leader" && syncRole != "follower")
            throw std::runtime_error("unrecognized sync_role " + syncRole);
        assert_throw(syncRole == "none" || useMcpwm, "sync_role needs pwm_driver=mcpwm");
        // phase as an angle (frequency-independent, 180 = interleave), plus an additive ns trim
        // for wire + receiver propagation delay, which is a time and does not scale with pwm_freq
        float periodNs = 1e9f / (float) pwmFrequency;
        float syncPhaseDeg = converterConf.getFloat("sync_phase_deg", 0.f);
        float trimNs = converterConf.getFloat("sync_phase_ns", 0.f);
        // one turn only: periodNs comes from the nominal pwm_freq, not the realized
        // resolutionHz/periodTicks (~100 ppm apart), and that error compounds past 360 deg
        assert_throw(std::abs(syncPhaseDeg) <= 360.f, "sync_phase_deg out of range");
        assert_throw(std::abs(trimNs) <= periodNs, "sync_phase_ns out of range");
        syncPhaseNs = syncPhaseDeg * (periodNs / 360.f) + trimNs;
#else
        if (converterConf.getString("sync_role", "none") != "none")
            ESP_LOGW("converter", "%s", "sync_role set but firmware built without FUGU_WITH_WSYNC");
#endif

        // rect_offset is stored as a time (rect_offset_ns) and converted to counts below, so it
        // survives the LEDC<->MCPWM resolution change without re-measuring.
        drvInit(pinCtrl, pinRect, boardConf, syncRole, syncPhaseNs);
        ESP_LOGI("converter", "gate driver: %s (pwmMax=%u)", driverName, (unsigned) driverPwmMax);

        if (pinSd != 255) {
            pinMode(pinSd, OUTPUT);
            digitalWrite(pinSd, 1);
        }

        // Minimum LS on-time refreshing the HS gate-driver bootstrap cap when the switch node is
        // not otherwise pulled low (high duty, or DCM/zero coil current). Fixed time, not a duty
        // fraction: the recharge need is set by HS gate charge, independent of fsw. During normal
        // conversion the LS body diode refreshes the cap, so this only binds in those corners.
        // 2000 ns. NOT the 500 ns this shipped with, and not the ~1500 ns that merely restores
        // the hardcoded MinDutyCycleLS = 0.06 it replaced -- 6% is ~244 counts here and that is
        // BELOW the measured floor.
        //
        // MEASURED on fbuck 2026-08-11, buck output open, Vin 40.9 V. Out of `reset` the
        // converter comes up at DCM(H|L|Lm)=1506|80|80, i.e. the LS on-time pinned at this
        // minimum. With no load the LS body diode never conducts, so this refresh is the ONLY
        // thing recharging the HS bootstrap cap (see the comment above). Starved, the high-side
        // gate drive does not fail cleanly -- it fires in BURSTS, so the switch node shows a
        // clean hard turn-off but no fast turn-on for most cycles. Fab established LS >= 300
        // counts as the safe lower bound on this board; 2000 ns gives 320 counts at 39.3 kHz on
        // the 4069-count driver, keeping margin above it.
        //
        // Being a time rather than a count, this stays correct across the LEDC<->MCPWM
        // resolution change; the 300-count floor is board- and fsw-specific, the 2 us is not.
        //
        // pwmRectMin is a COMMANDED span (cmpLS - cmpHS) while the refresh is the REALIZED pulse,
        // hardware dead-time delays the LS turn-on by the HS->LS value — so that is reserved on
        // top of the refresh, not taken out of it. Without that the realized pulse is hl ticks short
        // (286 counts at 200 ns on fbuck — under the 300-count floor above).
        auto bootRefreshNs = boardConf.getFloat("boot_refresh_ns", 2000.f);
        rectRefreshTicks = isBoost ? 0 : (uint16_t) std::ceil(bootRefreshNs * 1e-9f * getPwmTickRate());
        pwmRectMin = isBoost ? 0 : (uint16_t) (rectRefreshTicks + drvDtHlTicks());
        // Boost: cap max duty at 90% so the LS FET always turns off to release inductor energy
        // -1: cmpLS must stay under the period, so the widest LS span at max duty is
        // driverPwmMax - pwmCtrlMax - 1 and that must still reach pwmRectMin
        pwmCtrlMax = isBoost ? (uint16_t)(driverPwmMax * 0.9f)
                             : (uint16_t)(driverPwmMax - pwmRectMin - 1);
        pwmCtrlMin = 1; //isBoost ? 0 : 0;
        // note that mosfets have different Vg(th) and switching times worst case is Vi/o=80/12
        // ^ set pwmMinHS a bit lower than pwmMinLS (might cause no-load output over-voltage otherwise)

        ESP_LOGI("converter", "drv=%s f=%lu boost=%d pwmMax=%hu minLS=%hu minHS=%hu maxHS=%hu",
                 driverName, pwmFrequency, isBoost, driverPwmMax, pwmRectMin, pwmCtrlMin, pwmCtrlMax);

        // rect_offset_ns is the fixed gate-drive/MOSFET turn-off delay; convert to counts the same
        // way boot_refresh_ns is handled (ns -> counts via the tick rate), so the calibration is
        // invariant to PWM resolution and fsw. Default 0 (no offset) when unset.
        rectOnOffset = (int16_t) rectOffsetCountsFromNs(coilConf.getFloat("rect_offset_ns", 0.f), getPwmTickRate());
        logConfig();
    }

    void computePwmRectMax() {
        // update pwmRectMax for DCM or DCM case
        if (dcmHysteresis) {
            // DCM: the LS turn-off count is the fractional ideal pwmCtrl*ratio quantized to whole
            // PWM ticks. Plain round() stair-steps it, so as duty sweeps the turn-off lands a tick
            // before/after the inductor zero crossing, modulating delivered charge (the duty-pinned
            // SR reverse-current oscillation). First-order error feedback carries the rounding
            // remainder forward so the time-average turn-off tracks the ideal and the beat averages
            // out. Same conservative target (convRatioWCE), so no extra reverse-current bias.
            // rectOnOffset compensates a fixed dead-time/gate-delay between the commanded LS count
            // and the true zero crossing (measure_coil.py --ls-sweep). >0 turns LS off later,
            // recovering body-diode loss but eating reverse-current margin; default 0 = unchanged.
            float ideal = (float) pwmCtrl * pwmRectRatioDCM + (float) rectOnOffset;
            if (rectDither) {
                float v = ideal + rectDitherErr;
                long ls = std::lround(v);
                rectDitherErr = v - (float) ls;
                pwmRectMax = (uint16_t) std::max<long>(0, ls);
            } else {
                pwmRectMax = (uint16_t) std::round(ideal);
            }
        } else {
            // CCM
            pwmRectMax = driverPwmMax - pwmCtrl - 1;   // -1: keep cmpLS < period (timer-wrap)
        }
        pwmRectMax = std::min<uint16_t>(std::max(pwmRectMin, pwmRectMax), driverPwmMax - pwmCtrl - 1);
    }

    void pwmPerturb(int16_t direction) {
        bool enabling = unlikely(disabled() and direction > 0);
        if (enabling) {
            UART_LOG("Converter enabled");
            if (pinSd != 255) digitalWrite(pinSd, 0);
        }

        pwmCtrl = constrain(pwmCtrl + direction, pwmCtrlMin, pwmCtrlMax);


        bool largerDecrease = (-direction > driverPwmMax / 50);

        // update pwmRect block
        if (manualRect >= 0) {
            // bench: hold LS at the requested count (clamped to the complementary max)
            pwmRect = (uint16_t) constrain(manualRect, (int) pwmRectMin,
                                           (int) (driverPwmMax - pwmCtrl - 1));
        } else {
            if (largerDecrease && !forcedPwm)
                // assume DCM as it will always give equal or less pwmRectMax
                dcmHysteresis = true;

            computePwmRectMax();

            if (largerDecrease && !forcedPwm) {
                // we don't know if we end up in CCM/DCM, so set rect duty cycle to min
                // let the converter and sensors converge
                if (pwmRect > pwmRectMin + (driverPwmMax / 64))
                    UART_LOG("Set pwmLS %hu -> pwmRectMin=%hu\n", pwmRect, pwmRectMin);
                pwmRect = pwmRectMin;
            } else {
                if (pwmRect - pwmRectMax > (driverPwmMax / 16)) {
                    // report if rect duty cycle is significantly above upper limit
                    UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu\n", pwmRect, pwmRectMax);
                }

                // "fade-in" the low-side duty cycle
                // start slowly, then quickly step towards pwmRectMax
                auto step = 1 + ((pwmRect > pwmRectMin + (driverPwmMax / 64)) ? (pwmRectMax - pwmRect) / 64 : 0);
                pwmRect = syncRectEnabled
                              ? constrain(pwmRect + step, pwmRectMin, pwmRectMax)
                              : pwmRectMin;
            }
        }


        drvCommit(enabling, largerDecrease, direction);
    }


    /**
     * Perturb by a fractional step.
     * Instantly perturbs the integer part and accumulates the remainder in a buffer
     *
     * @param directionFloat
     */
    void pwmPerturbFractional(float directionFloat) {
        if (!(std::abs(directionFloat) <= driverPwmMax)) {
            ESP_LOGE("buck", "perturbation out of range %f", directionFloat);
            //return;
            throw std::out_of_range("pwmPerturbFractional");
        }
        //assert(std::abs(directionFloat) <= driverPwmMax);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int16_t) (directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }


    void disable() {
        if (pinSd != 255)digitalWrite(pinSd, 1);
        drvShutdown();

        if (pwmCtrl > pwmCtrlMin)
            UART_LOG("PWM disabled (duty cycle was %d)\n", (int) pwmCtrl);

        pwmCtrl = 0;
        pwmRect = 0;
        syncRectEnabled = false;
    }


    [[nodiscard]] inline float rippleCurrent(float hv, float lv) const {
        // this does not consider dc bias, just a constant 0.95 inductivity factor
        return lv / fL * (1.0f - lv / hv);
    }

    /**
     *
     * @param vh higher-voltage side (buck: Vin)
     * @param vl lower-voltage side (buck: Vout)
     * @param il inductor dc current
     */
    [[nodiscard]] bool computeDCM(float vh, float vl, float il) {
        auto ir = rippleCurrent(vh, vl);
        auto dcm = ir > il * (dcmHysteresis ? DcmExitRippleRatio : DcmEnterRippleRatio)
                   || il < DcmForceCurrent;
        if (forcedPwm) dcm = false;
        if (dcm != dcmHysteresis) {
            dcmHysteresis = dcm;
            UART_LOG("converter: %s -> %s (M=%.2f, I=%.2f, ∆I/2=%.2f, pwm=%hu)",
                     dcm ? "CCM" : "DCM", dcm ? "DCM" : "CCM",
                     isBoost ? (vh / vl) : (vl / vh), il, ir * .5f, pwmCtrl);
        }
        return dcm;
    }

    /*!
      * @brief  Compute t_rect/t_ctrl ratio for DCM
      *
      * Used in DCM to limit t_on for the rect switch to prevent reverse coil current (forced PWM).
      * For the buck converter it is t_onLS/t_onHS. (HS=Ctrl)
      * For the boost converter t_onHS/t_onLS. (LS=Ctrl)
      * See doc/Diode Emulation.rst.
      *
      * @param m converter voltage ratio
      * @return pwmRect/pwmCtrl ratio
      */
    [[nodiscard]] float rectCtrlRatio(float m) const {
        return isBoost ? (1.f / (m - 1.f)) : (1.f / m - 1.f);
    }

    [[nodiscard]] bool inDCM() const {
        return dcmHysteresis;
    }

    /**
     * Compute low-side switch duty cycle to emulate a diode for synchronous buck (sensor-less)
     * Prevents reverse current through the LS switch, which would lead to voltage boost and reverse current
     * and can eventually destroy the LS switch.
     * High-voltages at the input can destroy anything connected (including the board itself!)
     *
     * The function does some error estimation and decides whether we are in DCM or CCM mode and limits the LS duty cycle accordingly.
     * See https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=19 for more info about sync buck modes and timings
     *
     * by Fabian S. (fl4p) https://github.com/fl4p/fugu-mppt-firmware/
     *
     * @param pwmCtrl Duty cycle of the ctrl switch (buck:HS, boost:LS)
     * @param pwmMax The maximum duty cycle value
     * @param voltageRatio Vout/Vin ratio, D (the greater, the safer but less efficient)
     * @return
     */
    void computeSyncRectRatio(float vh, float vl, float il) {
        // computation of t_onCtrl and t_onRect ratio is quite error sensitive
        // e.g. at VR=0.64 a -5% error causes a 13% deviation of pwmMaxLs !
        // so we do some proper error computation here
        constexpr float voltageMaxErr = 0.01f; // inc -> safer, less efficient

        // WCEF = worst case error factor
        // compute the worst case error (Vout estimated too low, Vin too high => VR estimated too low)
        // this is true for buck and boost
        constexpr float voltageRatioWCEF = (1.f - voltageMaxErr) / (1.f + voltageMaxErr); // < 1.0


        float convRatioWCE =
        (isBoost
             ? ((vh > vl && vl > MinRatioVoltage) ? constrain(vh / vl, MinVoltageRatio, MaxBoostRatio) : MaxBoostRatio) // boost
             : ((vh > vl && vh > MinRatioVoltage) ? constrain(vl / vh, MinVoltageRatio, 1.f - UnityRatioMargin) : 1.0f) //buck
        ) / voltageRatioWCEF;

        // the WCEF division can push M past its physical bound (buck: <1, boost: >1),
        // which would make rectCtrlRatio() negative; clamp back
        convRatioWCE = isBoost ? std::max(convRatioWCE, 1.f + UnityRatioMargin)
                               : std::min(convRatioWCE, 1.f - UnityRatioMargin);

        outInVoltageRatio = convRatioWCE;

        if (computeDCM(vh, vl, il)) {
            if (il < SyncRectOffCurrent || vl < SyncRectOffVoltage) {
                if (pwmRectRatioDCM > 0.2f && pwmRect > pwmRectMin)
                    ESP_LOGI("converter", "Disable sync rect, low I(%.2f)/V(%.2f) pwm=%hu|%hu", il,
                         vl, pwmCtrl, pwmRect);
                pwmRectRatioDCM = 0.0f;
            } else {
                pwmRectRatioDCM = rectCtrlRatio(convRatioWCE);
                //if(dcmHysteresis)pwmRectRatioDCM = min(pwmRectRatioDCM, 1.5f); // TODO
            }
        }
    }
    
    const float &updateSyncRectMaxDuty(float vin, float vout, float il) {
        auto &vh(isBoost ? vout : vin);
        auto &vl(isBoost ? vin : vout);

        computeSyncRectRatio(vh, vl, il);
        if (manualRect >= 0) return outInVoltageRatio; // bench: hold manual LS, skip clamp
        computePwmRectMax();

        if (pwmRect > pwmRectMax) {
            if (pwmRect - pwmRectMax > (driverPwmMax / 40)) {
                UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu (VR=%.3f)", pwmRect, pwmRectMax, outInVoltageRatio);
            }
            pwmRect = pwmRectMax;
            drvCommitLsOff(pwmCtrl, pwmRect); // instantly commit if limit decreases
        }

        return outInVoltageRatio;
    }


    /**
     * Permanently set enable/disable low-side switch
     * @param enable
     */
    void enableSyncRect(bool enable, bool overwriteFPWM = false) {
        if (forcedPwm && !overwriteFPWM) enable = true;
        if (enable != syncRectEnabled) {
            UART_LOG("Sync rect %s", enable ? "enabled" : "disabled");
        }
        syncRectEnabled = enable;
        if (!enable)
            syncRectMinDuty();
    }

    /**
     * Set rect sync duty cycle to minimum (buck: disable power conduction, just keep the bootstrapping powered for HS drive)
     * Notice that this is only needed if no inductor is connected to the half-bridge, as the inductor will push the
     * switch node voltage until LS diode conducts.
     * Boost converter has a min duty cycle of 0 (HS)
     */
    void syncRectMinDuty() {
        if (pwmRect > pwmRectMin) {
            if (pwmRect > pwmRectMin + pwmRectMin / 2) {
                ESP_LOGW("dcdc", "set sync-rect PWM to minimum %hu -> %hu (vRatio=%.3f, pwmMaxLS=%hu)", pwmRect,
                         pwmRectMin, outInVoltageRatio, pwmRectMax);
            }
            pwmRect = pwmRectMin;
            drvCommitLsOff(pwmCtrl, pwmRect);
        }
    }

    void shortLs() {
        disable();
        drvShortLs();
        if (pinSd != 255)digitalWrite(pinSd, 0);
    }

    [[nodiscard]] const uint16_t &pwmCounts()  const { return driverPwmMax; }
};
