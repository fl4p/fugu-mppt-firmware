#pragma once

#include <atomic>

#include "freertos/FreeRTOS.h"

#include "adc/sampling.h"

#include "tele/telemetry.h"
#include "util.h"

#include "adc/temperature.h"
#include "cooling.h"

#include "buck.h"
#include "pwm/backflow.h"
#include "pwm/lv_pgood.h"
#include "battery.h"
#include "viz/lcd.h"
#include "tracker.h"
#include "peak_scan.h"
#include "pd_control.h"
#include "store.h"
#include "metering.h"
#include "charger.h"
#include "etc/plot.h"
#include "etc/mailbox.h"
#include "app_state.h"
#include "math/pv_model.h"
#include "conv_side.h"
#include "out_impedance.h"
#include "rect_cal.h"
#include "selftest/hs_short.h"

bool rectCalPersist(int counts, uint8_t cal); // coil.conf, core 0
void wireGuardConf(const ConfFile &c, WireGuardConf &w); // limits.conf wire_*, throws on bad values

struct Limits {
    const float Vin_max{};
    const float Vin_min{};

    const float Vout_max{};

    const float Iin_max{}, Ishort{};
    const float Iout_max{};

    const float P_max{};

    const float Temp_max{};

    const float Temp_derate{};

    const bool reverse_current_paranoia{};


    // The per-side hardware ratings are named by side (hv_max/lv_max, hv_i_max/lv_i_max) and mapped
    // to roles by topo (conv_side.h). The old role names throw.
    Limits(const ConfFile &limits, bool boost)
        : Vin_max((rejectLimitRoleKeys(limits, boost), readMappedLimit(limits, "vin_max", boost))),
          Vin_min(limits.getFloat("vin_min")),
          Vout_max(readMappedLimit(limits, "vout_max", boost)),
          Iin_max(readMappedLimit(limits, "iin_max", boost)),
          Ishort(limits.getFloat("iout_short")),
          Iout_max(readMappedLimit(limits, "iout_max", boost)),
          P_max(limits.getFloat("p_max")), Temp_max(limits.getFloat("temp_max", 90.0f)),
          Temp_derate(limits.getFloat("temp_derate")),
          reverse_current_paranoia(limits.getByte("reverse_current_paranoia", 1) != 0) {
        assert_throw(Vin_max > Vin_min, "");
        assert_throw(Vin_max * Iin_max > P_max, "");
        assert_throw(Vin_max * Iin_max < P_max * 4, "");
        assert_throw(Temp_derate < Temp_max, "");
        assert_throw(20 < Temp_max and Temp_max < 120, "");
    }

    explicit Limits(const ConfFile &limits) : Limits(limits, readTopoIsBoost("limits.conf")) {}

    Limits() = default;

    Limits(const Limits &lim) = default;

    Limits &operator=(const Limits &right) {
        if (this == &right) return *this;
        this->~Limits();
        new(this) Limits(right);
        return *this;
    }

    //Limits &operator=(const Limits& other) = default;
};


struct TeleConf {
    IPAddress influxdbHost;

    TeleConf() : influxdbHost(0UL) {
    }

    TeleConf(const ConfFile &teleConf) {
        auto host = teleConf.getString("influxdb_host", "");
        influxdbHost = host.empty() ? IPAddress(0UL) : IPAddress(host.c_str());
    }
};

enum class MpptControlMode : uint8_t {
    None = 0,
    CV,
    CC, // = 2
    CP,
    MPPT, // 4
    Sweep,
    Max,
};

enum class PsuSetpointError : uint8_t {
    None = 0,
    OutOfRange,
    TelemetryUnavailable,
    BoostBelowInput,
    OvLimitConflict,
};

enum class PsuCommand : uint8_t {
    None = 0, Enable, DisableMppt, DisableManual, StartSweep, ShortLowSide, EnablePv
};

struct PsuReq {
    float v = NAN, isc = NAN, k = NAN; // v: PSU setpoint or PV Voc
    int32_t rect = -1;
    uint16_t duty = 0;
    bool rebase = false, boot = false, keepBackoff = false; // keep: backoff and CV-floor latch survive a mode change
    bool stop = false; // manual duty 0: PWM off at once, no ramp
};

struct PvCurve {
    float isc, voc, k;
    bool active;
};


// inline + const char* => single copy in flash .rodata, no per-TU std::string array or global ctor
inline constexpr std::array<const char *, (size_t) MpptControlMode::Max> MpptState2String{
    "N/A",
    "CV",
    "CC",
    "CP",
    "MPPT",
    "SWEEP"
};


template<typename T_NUM, T_NUM max = std::numeric_limits<T_NUM>::max()>
struct MinSampler {
    T_NUM min{max};

    void add(const T_NUM &v) { if (v < min) min = v; }

    void reset() { min = max; }

    bool empty() const { return min == max; }

    const T_NUM &get() const {
        return min;
    }

    T_NUM pop() {
        auto r = get();
        reset();
        return r;
    }
};

/**
 * Implements
 * - Protection of DCDC converter
 * - Voltage and current control loop
 * - MPP global scan
 * - Telemetry
 */
class MpptController {
    ADC_Sampler &sampler;

public:
    SynchronousConverter &converter;
    LCD &lcd;

private:
    float cntrlValue = 0.0f;

    struct {
        MpptControlMode mode: 3 = MpptControlMode::None;
        bool _limiting: 1 = false; // control limited (no MPPT)
        bool _limReal: 1 = false; // the current limit condition went past its limit
        bool _limPast: 1 = false; // any limiter past its limit this update (rect-cal)
        uint8_t limIdx: 4 = 15;
    } ctrlState;

public:
    //MinSampler<MpptControlMode, MpptControlMode::Max> ctrlModeSampled{};
    MinSampler<uint8_t, 15> limIdxSampled{};
    uint16_t targetDutyCycle = 0; // one-shot ramp target from sweep/MPP (cleared on arrival)
    // Written by console/self-test tasks and consumed by the RT loop. The target itself is the
    // mailbox; PWM hardware remains RT-owned.
    std::atomic<uint16_t> manualTarget{0};
    uint32_t lastVoutSamples = 0; // RT only, edge-detects fresh Vout data for the forced-PWM gate
    uint8_t minPulseCnt = 0; // RT only, fresh Vout samples in a row above the LS min pulse ratio
    bool minPulseFloor = false; // RT only, update() holds Vin above the LS min pulse ratio
    bool minPulseWasExposed = false; // RT only, protect()'s previous minPulseExposed()
    uint8_t ilSamples = 0; // RT only, edge-detects fresh samples of the current the converter's DCM test uses

private:
    bool _sweeping = false; // global scan
    uint32_t _limitSinceMs = 0; // start of the current limit condition
    float _limitPert = 0; // duty counts the current limit condition has taken


    struct {
        float power = 0;
        float voltage = 0; // Vin EWM, as delayed as the power
        float slope = 0;   // [V/count] Vin fall per duty count there
        uint16_t dutyCycle = 0;
    } maxPowerPoint; // MPP during sweep

    // sweep verdict, RT: landed power vs the sweep MPP once settled
    uint8_t _svPhase = 0; // 1 landing, 2 settling, 3 return pass
    uint8_t _svLandTrim = 0; // 1: landing trim pending, 2: verdict
    uint8_t _svFlag = 0;  // 0 none, 1 clean, 2 light changed
    struct {
        uint8_t n: 3 = 0; // consecutive no-MPP sweeps, up to 4
        bool armed: 1 = false; // the running backoff is theirs
        bool open: 1 = false; // Vout outside the battery band during it
        uint8_t inBand: 2 = 0; // ~1 s ticks back in the band
        bool tick: 1 = false;
    } _noMpp;
    float _svLand = 0, _svI = 0; // landed power, input current then
    float _capV = 0; // [V] Vout cap that ended the last sweep while BMS cell data was missing or at no power, 0 none
    uint32_t _svT = 0, _resweepAt = 0, _retryAt = 0; // [ms] settle start, scheduled re-sweep, scheduling of the last one that ran
    void sweepVerdict(uint32_t nowMs, float p, bool ok);
    void scheduleResweep(uint32_t nowMs, uint32_t inMs);
    void capNote(float cap);
    void capResweep(uint32_t nowMs);
    void noMppBatCheck();
    void sweepJudge(float p, float d, bool rescan);

    // A sweep only commits its captured MPP as the operating target above this power. In marginal
    // light (e.g. a dawn cold-start sweep) the "peak" is just noise at near-max duty; committing it
    // strands the converter there until it's driven to the opposite rail. Below it: no target, fall
    // back to normal MPPT / the next periodic re-sweep. MPPT still harvests sub-threshold light.
    static constexpr float SweepMinPowerFrac = 0.002f; // of p_max: 2 W at 1 kW
    static constexpr uint8_t MinPulseDebounce = 4; // fresh Vout samples
    [[nodiscard]] float sweepMinPower() const { return limits.P_max * SweepMinPowerFrac; }

    Plot sweepPlot{};

    time_ms lastTimeProtectPassed = 0;
    time_us _lastPointWrite = 0;
    time_us _backoffUntilUs = 0;
    uint32_t _backoffArmedSec = 0; // length of the running backoff, so a repeat can't extend it
    time_ms psuTripWindowStart = 0;
    std::atomic<uint8_t> psuTripCount{0};
    // PSU/PV mailbox. The command word's tag is the request ticket, its payload the PsuCommand, so
    // a later request cannot make an earlier RT completion wake it. The payload is posted in the
    // same seqlock window as the command, so the RT core never pairs one with another's payload.
    SeqLock<PsuReq> psuReq; // read only along with a posted command
    ReqSlot<8> psuCmd;
    // More than one non-RT producer can wait concurrently. A single "last completed" ticket
    // loses A when B completes before A is scheduled. Keep a bounded result slot per ticket;
    // there are currently at most two producers (console and measure-coil), with ample margin.
    static constexpr size_t PsuResultSlots = 8;
    std::array<std::atomic<uint32_t>, PsuResultSlots> completedPsuResults{};
    portMUX_TYPE psuMailboxMux = portMUX_INITIALIZER_UNLOCKED;
    std::atomic<PsuSetpointError> lastPsuRequestError{PsuSetpointError::None};
    std::atomic<float> publishedPsuSetpoint{NAN};
    // Runtime console setting: both CLI and RT protection read it, so it cannot live in the
    // otherwise single-core charger params object as an ordinary float.
    std::atomic<float> explicitOvLimit{NAN};
    // Published PV curve for non-RT readers (status/OTA/measure-coil); RT is the only writer.
    SeqLock<PvCurve> pvPub{{NAN, NAN, NAN, false}};
    std::atomic<float> pvBaseIsc{NAN}; // last full `pv` Isc, reference for `pv scale`
    unsigned short _teleNumPoints = 0;

    const VIinVout<const Sensor *> &sensors;

    uint32_t postPsuCommand(PsuCommand command, const PsuReq &req = {}) {
        // Producers can be separate tasks on NON_RT_CORE (console, OTA and measure-coil).
        // A plain atomic spin lock can deadlock if a higher-priority producer preempts its
        // owner on that same core. The IDF port mux disables local preemption while owned;
        // the RT consumer never takes it and only observes the seqlock.
        portENTER_CRITICAL(&psuMailboxMux);
        psuReq.begin();
        psuReq.put(req);
        const uint32_t w = psuCmd.post(static_cast<uint8_t>(command));
        psuReq.end();
        portEXIT_CRITICAL(&psuMailboxMux);
        return ReqSlot<8>::tag(w);
    }

    void completePsuCommand(uint32_t ticket, PsuSetpointError error) {
        completedPsuResults[ticket % PsuResultSlots].store(
            (ticket << 8) | static_cast<uint8_t>(error), std::memory_order_release);
    }

    void publishPvState(bool active, float isc, float voc, float k) { pvPub.write({isc, voc, k, active}); }

    void pvDeactivateRt() {
        if (!pvSim.active) return;
        pvSim.active = false;
        publishPvState(false, NAN, NAN, NAN);
    }


    // Gains below are the defaults; converter.conf::ctrl_<name>_{kp,kd,td} override them (begin()).
    // They are not the whole loop gain: update() scales the result into a duty slew rate
    // by a further per-path constant (see kCtrlSlewLimit in mppt.cpp).
    PD_Control VinController{-100, -200, true}; // Vin under-voltage
    PD_Control VoutController{1500, /*100**/ 12 * 1000, true}; // Vout over-voltage  TODO 8k, 10k prevents full sweep
    PD_Control IinController{100, 200, true}; // Iin over-current
    PD_Control_SmoothSetpoint IoutCurrentController{200, 400, 200}; // Iout over-current // TODO PID?
    PD_Control_SmoothSetpoint powerController{20, 5, 200}; // over-power // TODO PID?
    //PD_Control LoadRegulationCTRL{5, -200, true}; //

public:
    const Sensor *sensorPhysicalI{nullptr};
    const Sensor *sensorPhysicalU{nullptr};
    Tracker tracker{};
    PeakScan scan{};

private:
    uint8_t ledPinSimple = 255;

    uint16_t targetPwmCnt = 0; // config-derived boot duty; cleared on mppt/sweep to allow tracking

    float sweepSpeed = 4.0f; // global-sweep speed (tracker.conf::sweep_speed)
    float sweepPace = 4.0f;  // tracker.conf::sweep_pace, largest sweep speed-up where Vin moves slowly, 1 = off
    float _paceV = 0, _paceVo = 0, _pace = 1, _paceMax = 1, _paceS = 0; // Vin, Vout at the pace anchor, speed-up, bound, Vin fall/count
    float _paceCap = 0, _lagF = 0; // [W] input caps' discharge cIn * Vin * -dVin/dt over the last pace window; [s] filter delay
    uint16_t _paceD = 0;         // duty at the pace anchor
    uint32_t _paceT = 0;         // [us] time at the pace anchor
    float cIn = 1e-3f;           // [F] converter.conf::c_in, input capacitance (sweep slow-down)
    float sweepVerify = 0.1f; // tracker.conf::sweep_verify, 0 = off

public:
    Limits limits{};
    TeleConf tele{};
    BatteryCharger charger;
    BackflowDriver bflow{};
    LvPgood lvPgood{};
    SolarEnergyMeter meter{};
    OutputImpedance outZ{};
    TempSensorGPIO_NTC ntc;
    Esp32TempSensor ucTemp;
    Fan fan{};

    float speedScale = 1;

    explicit MpptController(ADC_Sampler &dcdcPwr, const VIinVout<const Sensor *> &sensors,
                            SynchronousConverter &converter, LCD &lcd)
        : sampler(dcdcPwr), converter(converter), lcd(lcd), sensors{sensors},
          charger{} {
    }

    void initSensors(const ConfFile &boardConf) {
        assert_throw(sensors.Vout, "");
        assert_throw(sensors.Iout, "");

        if (sensors.Iout->isVirtual) {
            ESP_LOGI("mppt", "Iout sensor is virtual, using Iin");
            sensorPhysicalI = sensors.Iin;
            sensorPhysicalU = sensors.Vin;
        } else {
            // use Iout by default
            sensorPhysicalI = sensors.Iout;
            sensorPhysicalU = sensors.Vout;
        }
        if (sensorPhysicalI->isVirtual) throw std::runtime_error("no physical I sensor");
        if (sensorPhysicalU->isVirtual) throw std::runtime_error("no physical U sensor");

        ntc.begin(boardConf);
        ucTemp.begin();
        ucTemp.read();
    }

    __attribute__((cold)) void begin(const ConfFile &trackerConf, const ConfFile &boardConf, const ConfFile &converterConf,
               const Limits &limits_, const TeleConf &tele_);

    [[nodiscard]] MpptControlMode getState() const { return ctrlState.mode; }

    // [s] the larger mean delay of the Vin and power filters: median-5 (2 samples), a 2-pass EWM (span - 1)
    // and the ripple comb (L/2)
    [[nodiscard]] float sweepFiltDelay() const {
        auto d = [](const Sensor *s) {
            float fs = s->isVirtual ? 0 : static_cast<const PhysicalSensor *>(s)->notchFs;
            float k = (float) (s->ewm.span() + 1);
            if (s->comb) k += .5f * s->comb->length();
            return fs > 0 ? k / fs : 1.f;
        };
        return std::max(d(sensors.Vin), std::max(d(sensorPhysicalI), d(sensorPhysicalU)));
    }

    // 99 % step settling: 2-pass EWMA 6.64 tau (tau ~ span/2 samples) + median-of-5 delay + the comb's full window
    [[nodiscard]] static float settleMs(const Sensor *s) {
        if (s->isVirtual) return 0;
        float fs = static_cast<const PhysicalSensor *>(s)->notchFs;
        float k = 3320.f * (float) s->ewm.span() + 2000.f;
        if (s->comb) k += 1000.f * (s->comb->length() + 1.f);
        return fs > 0 ? k / fs : 1000.f;
    }

    [[nodiscard]] float powerSettleMs() const { return std::max(settleMs(sensorPhysicalI), settleMs(sensorPhysicalU)); }

    struct TeleSnap {
        float Ui, Uo, I, P, mcuTemp, ntcTemp;
        uint16_t duty;
        uint8_t mode, limIdx;
    };

    TeleSnap teleSnap() {
        float i = sensorPhysicalI->med3.get();
        return {.Ui = sensors.Vin->med3.get(), .Uo = sensors.Vout->med3.get(), .I = i,
                .P = i * sensorPhysicalU->med3.get(), .mcuTemp = ucTemp.lastFresh(), .ntcTemp = ntc.lastFresh(),
                .duty = (uint16_t) converter.getCtrlOnPwmCnt(),
                .mode = (uint8_t) ctrlState.mode, .limIdx = ctrlState.limIdx};
    }
    [[nodiscard]] bool active() const { return _sweeping or sampler.isCalibrating() or !converter.disabled(); }
    [[nodiscard]] bool isSweeping() const { return _sweeping; }
    [[nodiscard]] const Plot &sweepCurve() const { return sweepPlot; }
    [[nodiscard]] const auto &sweepMpp() const { return maxPowerPoint; }
    [[nodiscard]] uint8_t sweepFlag() const { return _svFlag; }
    [[nodiscard]] float sweepLand() const { return _svLand; }

    // Default backoff blocks startCondition() so MPPT doesn't re-poke pwmPerturb()
    // every tick after a protection trip — otherwise OV/OC violations spam the log
    // and toggle the converter at sample rate (see Vin-OV regression with Voc>Vin_max).
    // Callers that want immediate-recovery semantics (calibration done, user `dc 0`)
    // must pass 0 explicitly. `who` tags the trip path in the backoff log so a
    // stuck post-sweep state names the responsible protect.
    void shutdownDcdc(const char *who, uint32_t backoffSec = 5) {
        _limitPert = 0;
        lvPgood.release();
        bflow.enable(false);
        converter.disable();
        if (backoffSec) {
            _noMpp.armed = false;
            // PSU fast-retry: only for transient faults (Vout-OV, supply-UV). Other faults
            // (sensor-fail, revI, highI) keep their normal backoff even in PSU mode.
            // stopAndBackoff caller is maintenance, not a fault — don't count it.
            bool psuFastRetry = g_app.psuMode() && backoffSec <= 5
                && strcmp(who, "stopAndBackoff") != 0
                && (strstr(who, "Vout-OV") || strstr(who, "supply-UV"));
            if (psuFastRetry) {
                auto nowMs = wallClockMs();
                if (nowMs - psuTripWindowStart > 60000) {
                    psuTripWindowStart = nowMs;
                    psuTripCount.store(0, std::memory_order_relaxed);
                }
                const uint8_t tripCount = psuTripCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (tripCount > 8) {
                    psuLatched = true;
                    _backoffUntilUs = wallClockUs() + 30000000ULL;
                    _backoffArmedSec = 30;
                    ESP_LOGE("mppt", "PSU latch [%s] (%u trips)", who, tripCount);
                } else if (tripCount > 4) {
                    psuEscalated = true;
                    if (!inBackoff() || backoffSec > _backoffArmedSec) {
                        _backoffUntilUs = wallClockUs() + static_cast<time_us>(backoffSec) * 1000000ULL;
                        _backoffArmedSec = backoffSec;
                        ESP_LOGW("mppt", "PSU escalated backoff %lus [%s] (trip %u)",
                                 (unsigned long) backoffSec, who, tripCount);
                    }
                } else {
                    _backoffUntilUs = wallClockUs() + 100000ULL;
                    _backoffArmedSec = 0;
                    ESP_LOGW("mppt", "PSU fast retry [%s] (trip %u)", who, tripCount);
                }
            } else {
                if (!inBackoff() || backoffSec > _backoffArmedSec) {
                    _backoffUntilUs = wallClockUs() + static_cast<time_us>(backoffSec) * 1000000ULL;
                    _backoffArmedSec = backoffSec;
                    _noMpp.armed = !strcmp(who, "sweep-no-mpp");
                    if (!_noMpp.armed)
                        ESP_LOGW("mppt", "backoff %lus [%s]%s", (unsigned long) backoffSec, who,
                                 _sweeping ? " mid-sweep" : "");
                    else
                        ESP_LOGI("mppt", "backoff %lus [%s]", (unsigned long) backoffSec, who);
                }
            }
            _sweeping = false;
        }
    }

    // Manual override (console `sweep`): drop any pending backoff so the user's
    // intent isn't silently absorbed by a stale trip timer.
    void clearBackoff() {
        _backoffUntilUs = 0;
        _backoffArmedSec = 0;
    }

    // Discard a running scan without committing its MPP. active() stays true while _sweeping is
    // set, so a scan the user overrides (manual PWM) would otherwise keep the state machine in
    // sweep mode until the next `mppt`.
    void abortSweep() {
        _sweeping = false;
        maxPowerPoint = {};
        targetDutyCycle = 0;
        _svPhase = 0;
        scan.reset(wallClockMs());
    }

    // RT, sweeping, per pwmCtrlMax/128 counts or Voc/256 of Vin: speed-up 1.._paceMax where Vin moved less than Voc
    // per pwmCtrlMax counts (the knee past the MPP is steeper and keeps 1); 1 at most while Vout moves by 1/256 of
    // itself (no battery: the ramp charges the output caps, which reads as power). Slow-down to 1/32 where the input
    // caps' discharge cIn * Vin * |dVin/dt|, which reads as power too, exceeds 5 % of the power p.
    __attribute__((always_inline)) void sweepPaceUpdate(float p) {
        const uint16_t d = converter.getCtrlOnPwmCnt();
        const float v = sensors.Vin->ewm.avg.get(), dv = _paceV - v;
        if (d < _paceD + converter.pwmCtrlMax / 128 + 1 && std::fabs(dv) * 256 < scan.voc) return;
        const auto t = (uint32_t) wallClockUs();
        const float vo = sensors.Vout->ewm.avg.get(), cap = cIn * v * std::max(0.f, dv) * 1e6f / (float) std::max<uint32_t>(t - _paceT, 1000);
        float pace = _paceMax;
        if (d > _paceD) {
            const float num = scan.voc * (float) (d - _paceD), den = std::fabs(dv) * (float) converter.pwmCtrlMax;
            _paceS = dv / (float) (d - _paceD);
            if (den * pace > num) pace = std::max(1.f, num / den);
        }
        if (std::fabs(vo - _paceVo) * 256 > vo) pace = std::min(1.f, pace);
        if (p >= sweepMinPower() && cap * 20 > p) pace = std::min(pace, std::max(1 / 32.f, _pace * p / (20 * cap)));
        _pace = pace, _paceCap = cap, _paceV = v, _paceVo = vo, _paceD = d, _paceT = t;
    }

    // Vin lower limit; minPulse: also above the LS min pulse ratio (buck DCM start ratio)
    [[nodiscard]] __attribute__((always_inline)) float vinFloor(bool minPulse) const {
        return minPulse && !converter.boost()
                   ? std::max(limits.Vin_min, sensors.Vout->ewm.avg.get() / converter.minPulseRatioMax(true))
                   : limits.Vin_min;
    }

    // Break a CV-floor lockup in place (no reboot): release the charger's Vout pinning and reset the
    // Vout controller + limit/target state so normal MPPT can climb again. Returns true if the
    // charger pin actually moved (i.e. there was a latch to clear).
    bool releaseCvFloorLatch(const char *why) {
        bool changed = charger.releaseVoutPinning(why);
        VoutController.reset();
        ctrlState._limiting = false;
        _limitPert = 0;
        ctrlState.limIdx = 15;
        if (ctrlState.mode == MpptControlMode::CV) ctrlState.mode = MpptControlMode::None;
        targetDutyCycle = 0;
        clearBackoff();
        return changed;
    }

    [[nodiscard]] bool inBackoff() const { return wallClockUs() < _backoffUntilUs; }
    [[nodiscard]] bool noMppBackoff() const { return _noMpp.armed && inBackoff(); }

    [[nodiscard]] float boardPowerSupplyVoltage() const {
        constexpr auto diodeFwdVoltage = 0.3f;
        return std::max(sensors.Vin->last, sensors.Vout->last) - diodeFwdVoltage;
    }

    [[nodiscard]] bool boardPowerSupplyUnderVoltage(bool start = false) const {
        if (isnan(sensors.Vin->last) || isnan(sensors.Vout->last))
            return false;
        return boardPowerSupplyVoltage() < (start ? 9.5f : 9.f);
    }

    // Buck start gates: the lower of the EWM and the median, as update() holds Vin with the median and the despike lets
    // the EWM lead it on a step
    [[nodiscard]] float vinStart() const { return std::fmin(sensors.Vin->ewm.avg.get(), sensors.Vin->med3.get()); }

    // buck: a start at output vOut would run the LS min pulse above the start ratio
    [[nodiscard]] bool minPulseStartBlocked(float vOut) const {
        return !converter.boost() && !(vOut < vinStart() * converter.minPulseRatioMax(true));
    }

    // A fitted sensor without a valid reading; an unfitted one never counts as stale.
    [[nodiscard]] bool tempStale() const {
        return (ntc.fitted() && !ntc.fresh()) || (ucTemp.fitted() && !ucTemp.fresh());
    }

    // Core 0, every loop pass: re-reads a stale fitted sensor every 100 ms instead of every lfPeriod, so
    // a fresh sensor (e.g. the NTC once the sampler has data) unblocks the start without delay.
    void tempPollStale(time_us nowUs) {
        static time_us lastUs = 0;
        if (nowUs - lastUs < 100000) return;
        lastUs = nowUs;
        if (ntc.fitted() && !ntc.fresh()) ntc.read(false);
        if (ucTemp.fitted() && !ucTemp.fresh()) ucTemp.read(false);
    }

    // Temperature start gate, 3 °C below the trip level
    [[nodiscard]] const char *tempStartBlock() const {
        if (tempStale()) return "temp-stale";
        if (ntc.last() > limits.Temp_max - 3 || !(ucTemp.last() < limits.Temp_max - 3)) return "temp";
        return nullptr;
    }

    // A reset filter (calibration end, ADC restart) reads NaN until its median window refills
    [[nodiscard]] bool filtersEmpty() const {
        for (const Sensor *s: {sensors.Vin, sensors.Vout, sensors.Iin, sensors.Iout})
            if (!std::isfinite(s->med3.get()) || !std::isfinite(s->ewm.avg.get())) return true;
        return false;
    }

    // First startCondition() clause currently blocking a start, or nullptr if clear.
    // Single source of truth for startCondition(); also logged while idle in START.
    [[nodiscard]] const char *startBlockReason() const {
        if (psuLatched) return "psu-latch";
        if (converter.startInhibited(SynchronousConverter::InhibitBoot)) return "restart";
        if (converter.startInhibited(SynchronousConverter::InhibitHs)) return hsFault.load() ? "hs-short" : "hs-check";
        if (converter.startInhibited(SynchronousConverter::InhibitBf)) return "bf-fail";
        if (converter.startInhibited(SynchronousConverter::InhibitCal)) return "conf-save";
        if (g_app.psuMode()) {
            const auto feasibility = currentPsuFeasibility();
            if (feasibility == PsuSetpointError::BoostBelowInput)
                return "psu-boost-setpoint";
            if (feasibility == PsuSetpointError::OvLimitConflict)
                return "psu-ov-conflict";
        }
        if (inBackoff()) return "backoff";
        if (charger.wireGuard.tripped()) return "wire-trip";
        if (const char *t = tempStartBlock()) return t;
        if (!g_app.psuMode() &&
            !(converter.boost()
                  ? sensors.Vin->ewm.avg.get() < sensors.Vout->ewm.avg.get() + 1
                  : vinStart() > sensors.Vout->ewm.avg.get() + 1))
            return "Vin-Vout";
        if (minPulseStartBlocked(std::max(sensors.Vout->ewm.avg.get(), g_app.psuMode() ? psuVsetpoint : 0.f)))
            return "minpulse-M";
        if (g_app.psuMode() && sensors.Vout && sensors.Vout->last > computeOvThreshold() * 0.98f)
            return "Vout-still-OV";
        if (boardPowerSupplyUnderVoltage(true)) return "supply-UV";
        if (sampler.isCalibrating()) return "calibrating";
        if (filtersEmpty()) return "filters-empty";
        if (converter.startInhibited(SynchronousConverter::InhibitIdle)) return kIdleBlock; // last: the only blocker
        return nullptr;
    }

    static const char kIdleBlock[]; // one object: callers compare the pointer across translation units

    [[nodiscard]] bool startCondition() const { return startBlockReason() == nullptr; }

    // RT start gates: startBlockReason(); a start refused only by the idle profile asks it to leave
    const char *startBlockReasonRt() {
        if (_noMpp.armed) noMppBatCheck();
        const char *r = startBlockReason();
        if (r == kIdleBlock) converter.wantStart();
        return r;
    }

    bool protectLf(bool ignoreUV) {
        //auto nowMs = loopWallClockMs();

        // power supply under-voltage shutdown
        if (boardPowerSupplyUnderVoltage() and not ignoreUV) {
            if (!converter.disabled())
                ESP_LOGW("mppt", "Supply under-voltage! Vin %.1f and Vout %.1f < 10", sensors.Vin->last,
                     sensors.Vout->last);
            shutdownDcdc("supply-UV");
            rt_defer([](void *m, float, float) { ((MpptController *) m)->meter.commit(); }, this);
            return false;
        }

        // detect battery voltage
        // TODO move this to charger ?
        if (!g_app.psuMode() && !charger.params.haveVbatMax()) {
            auto vout = sensors.Vout->calibrationAvg;
            float detectedVout_max = detectMaxBatteryVoltage(vout);
            if (std::isnan(detectedVout_max)) {
                ESP_LOGW("mppt", "Unable to detect battery voltage Vout=%.2fV", vout);
                converter.disable();
                rt_defer([](void *s, float, float) { ((ADC_Sampler *) s)->startCalibration(); }, &sampler);
                return false;
            } else {
                ESP_LOGI("mppt", "Detected max battery voltage %.2fV (from Vout=%.2fV)", detectedVout_max, vout);
                charger.params.Vbat_max = min(limits.Vout_max, detectedVout_max);
            }
        }

        static bool staleLogged = false;
        if (unlikely(tempStale())) {
            RT_COLD(
            if (!staleLogged) ESP_LOGE("mppt", "temperature stale (ntc=%d mcu=%d), shutdown",
                                       (int) ntc.fresh(), (int) ucTemp.fresh());
            staleLogged = true;
            shutdownDcdc("temp-stale", 5);
            );
            return false;
        }
        staleLogged = false;

        if (unlikely(ntc.last() > limits.Temp_max || ucTemp.last() > limits.Temp_max)) {
            RT_COLD(
            ESP_LOGE("mppt", "Temp %.1f (or mcu %.1f) > %.1f°C, shutdown", ntc.last(), ucTemp.last(), limits.Temp_max);
            shutdownDcdc("temp", 5);
            );
            return false;
        }

        if (unlikely(charger.wireGuard.tripped())) {
            RT_COLD(
            ESP_LOGE("mppt", "wire fault trip, shutdown (wire-reset to clear)");
            shutdownDcdc("wire-trip", 5);
            );
            return false;
        }

        return true;
    }

    bool protect(bool ignoreUV) {
        auto nowMs = wallClockMs();

        if (unlikely(g_hwOvTrip)) {
            g_hwOvTrip = false;
            RT_COLD(
            ESP_LOGE("mppt", "INA226 bus OV alert, Vin %.2f Vout %.2f", sensors.Vin->last, sensors.Vout->last);
            shutdownDcdc("hw-OV");
            );
            return false;
        }

        if (g_app.psuMode()) {
            const auto feasibility = currentPsuFeasibility();
            if (feasibility == PsuSetpointError::BoostBelowInput
                || feasibility == PsuSetpointError::OvLimitConflict) {
                RT_COLD(
                if (!psuLatched.exchange(true, std::memory_order_relaxed)) {
                    ESP_LOGE("mppt", "active PSU setpoint %.2f V became infeasible (Vin %.2f V, OV %.2f V)",
                             psuVsetpoint, sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN,
                             getExplicitOvLimit());
                    shutdownDcdc("psu-setpoint-infeasible", 30);
                }
                );
                return false;
            }
        }

        // input over-voltage
        if (sensors.Vin->last > limits.Vin_max) {
            // input over-voltage
            RT_COLD(
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", sensors.Vin->last, limits.Vin_max);
            shutdownDcdc("Vin-OV");
            );
            return false;
        }

        if (bmsCutoff()) {
            RT_COLD(
            ESP_LOGE("mppt", "BMS cut-off? Iout %.2f (avg %.2f) Vout %.2f (avg %.2f) Vin %.2f (avg %.2f)",
                     sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(),
                     sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get(),
                     sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
            shutdownDcdc("BMS-cutoff", 30);
            );
            return false;
        }

        // output over-voltage
        // An explicit `ovset` overrides the derived threshold (issue #59). Without it the threshold
        // is derived from Vbat_max × a factor (1.03 with reverse_current_paranoia, 1.5 without).
        // Until the battery is identified, the hard configured ceiling is the only threshold we
        // have. Deriving one from an unusable Vbat_max instead would yield ~0 and trip OV on any
        // output voltage — and protectLf()'s re-detect never runs, because a trip returns before it.
        // Release the downward-retarget hold once Vout is inside the new band (same 2 % margin
        // as startBlockReason), or when PSU mode is gone.
        if (std::isfinite(psuOvHoldBase)
            && (!g_app.psuMode() || !std::isfinite(psuVsetpoint)
                || sensors.Vout->last < 0.98f * psuVsetpoint
                                        * (limits.reverse_current_paranoia ? 1.03f : 1.5f)))
            psuOvHoldBase = NAN;
        auto ovTh = computeOvThreshold();
        //if (adcSampler.med3.s.chVout.get() > ovTh) {
        if (sensors.Vout->last > ovTh) {
            RT_COLD(
            //  && sensors.Vout->previous > ovTh * 0.9f
            bool wasDisabled = converter.disabled();
            shutdownDcdc("Vout-OV");

            auto vout = std::max(sensors.Vout->last, sensors.Vout->previous);

            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (prev=%.1fV,ewma=%.1fV,std=%.4f,D=%hu) > %.1fV + 5pct!",
                     sensors.Vout->last, sensors.Vout->previous,
                     sensors.Vout->ewm.avg.get(), sensors.Vout->ewm.std.get(), converter.getCtrlOnPwmCnt(),
                     ovTh
            );


            if (!g_app.psuMode() && nowMs - lastTimeProtectPassed > 20000) {
                // Persistent OV: auto-detect Vout_max by clearing the setpoint so protectLf()
                // re-detects it. But an explicitly commanded `vset` setpoint must not be silently
                // discarded — that widens the protection threshold (to the board ceiling) with no
                // feedback to the host (issue #59).
                if (charger.params.vbatMaxExplicit) {
                    ESP_LOGE("mppt", "Vout OV persists %.1fs, but Vbat_max=%.2fV was set via vset — NOT resetting",
                             (float) (nowMs - lastTimeProtectPassed) / 1000.f, charger.params.Vbat_max);
                } else {
                    ESP_LOGW("mppt", "Vout OV persists, clearing Vbat_max for auto-detect");
                    charger.params.Vbat_max = NAN;
                    sampler.startCalibration();
                }
            }

            rt_defer([](void *l, float v, float th) {
                ((LCD *) l)->displayMessageF("OV shutdown!\nVout=%.1fV max=%.1fV", 10000, v, th);
            }, &lcd, vout, ovTh);
            );

            return false;
        }


        // input over current
        if (sensors.Iin->last > limits.Iin_max * 1.3f && !converter.disabled()) {
            RT_COLD(
            shutdownDcdc("Iin-OC");
            ESP_LOGW("mppt", "Iin %.1f >1.3x lim (Iout=%.1f Vin=%.2f), shutdown",
                     sensors.Iin->last,
                     sensors.Iout->last, sensors.Vin->last);
            );
            return false;
        }

        // output over current
        if ((sensors.Iout->last > limits.Iout_max * 1.5f
             or sensors.Iout->med3.get() > limits.Iout_max * 1.25f
             or sensors.Iout->ewm.avg.get() > limits.Iout_max * 1.15f
            ) and not converter.disabled()) {
            RT_COLD(
            shutdownDcdc("Iout-OC", 30);
            ESP_LOGW("mppt", "Iout %.2f (med %.2f avg %.2f) >lim %.2f, shutdown", sensors.Iout->last,
                     sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(), limits.Iout_max);
            );
            return false;
        }

        const bool bfT = bfTest.mode.load(std::memory_order_relaxed); // the self-test drives reverse current itself
        if (sensorPhysicalI->last < -1 && sensorPhysicalI->previous < -1 && !converter.forcedPwm_() && !bfT) {
            RT_COLD(
            if (sensors.Iout->ewm.avg.get() > 10) {
                //buck.halfDutyCycle();
                shutdownDcdc("revI-highAvg");
                ESP_LOGE("MPPT", "Reverse I %.2fA, noise? high avg, shutdown", sensorPhysicalI->last);
            } else {
                if (bflow.state() || converter.getRectOnPwmCnt() > converter.getRectOnPwmMin())
                    ESP_LOGE("MPPT", "Reverse I %.2fA, noise? disable BFC+LS FET (pwm=%hu)",
                         sensorPhysicalI->last, converter.getCtrlOnPwmCnt());
                bflow.enable(false); // reverse current
                converter.syncRectMinDuty();
            }
            );
        }

        if (sensorPhysicalI->ewm.avg.get() < -1 /*&& !converter.forcedPwm_()*/) {
            RT_COLD(
            if (!converter.disabled())
                ESP_LOGE("MPPT", "Reverse avg current %.1f A, shutdown!", sensorPhysicalI->ewm.avg.get());
            shutdownDcdc("revI-ewm");
            );
            return false;
        }

        if (!converter.boost()) {
            if (sensors.Vout->ewm.avg.get() > (sensors.Vin->ewm.avg.get() + 1.0f) * 1.25f) {
                RT_COLD(
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown duty=%i", sensors.Vout->ewm.avg.get(),
                         sensors.Vin->ewm.avg.get(), (int) converter.getCtrlOnPwmCnt());
                shutdownDcdc("Vout>Vin-avg");
                );
                return false;
            }
            if (sensors.Vout->last > (sensors.Vin->last + .5f) * 2) {
                RT_COLD(
                ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", sensors.Vout->last, sensors.Vin->last);
                shutdownDcdc("Vout>2Vin");
                );
                return false;
            }

            // try to prevent voltage boost and disable low side for low currents
            auto currentFilt = fminf(sensorPhysicalI->ewm.avg.get(),
                                     std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
            if (currentFilt < -0.05f && limits.reverse_current_paranoia && !bfT) {
                RT_COLD(
                if (converter.getRectOnPwmCnt() > converter.getRectOnPwmMax() / 2 &&
                    converter.getRectOnPwmCnt() > (converter.pwmRectMin + converter.pwmCtrlMax / 20)) {
                    ESP_LOGW("MPPT", "Low I, set LS min duty (ewm(%s)=%.2f, max=%.2f)",
                             sensorPhysicalI->params.teleName.c_str(),
                             sensorPhysicalI->ewm.avg.get(),
                             std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
                }
                if (bflow.state())
                    ESP_LOGW("MPPT", "Low current %.2f, disable backflow", currentFilt);
                if (converter.getRectOnPwmCnt() > converter.getRectOnPwmMin())
                    ESP_LOGW("MPPT", "Low current %.2f, disable sync rect", currentFilt);
                converter.syncRectMinDuty();
                bflow.enable(false); // low current
                );
            }
        } else {
            // TODO Vin
        }

        if (sensors.Iout->ewm.avg.get() > limits.Ishort and sensors.Vout->ewm.avg.get() < 1) {
            RT_COLD(
            if (!converter.disabled())
                ESP_LOGE("MPPT", "Output short circuit detected! (V=%.2f, I= %.1fA)",
                     sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get());
            shutdownDcdc("short", 30);
            );
            return false;
        }

        // if bflow switch is powered by HS gate drive, need a min duty cycle
        // TODO lift this, bflow switch will be powered from bootstrap cap and not gate drive signal
        constexpr auto BflowMinDutyCycle = 0.1f;
        if (bflow && (!bflow.state() || converter.getDutyCycle() < BflowMinDutyCycle)) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                RT_COLD(
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High-current through open backflow switch!");
                shutdownDcdc("highI-bflowOpen");
                );
                return false;
            }

            // in case the current sensor is wrong: a DCM duty bounds the current by itself
            constexpr float BflowOpenMaxDcmCurrent = 1.f; // A
            float iDcm = NAN;
            const float vi = sensors.Vin->ewm.avg.get(), vo = sensors.Vout->ewm.avg.get();
            if (!bfT && converter.getDutyCycle() > 0.33f &&
                !((iDcm = converter.dcmCurrent(vi, vo)) < BflowOpenMaxDcmCurrent)) {
                // a sweep at the ratio limit with <0.05 A has no more power to find: end it, not a fault.
                // Vin can't really sit below Vout here; the 5% margin covers Vin sensor error at PV collapse
                if (_sweeping && std::isnan(iDcm) && !converter.boost() && !converter.forcedPwm_() && vo > 1.f &&
                    vi > 0.95f * vo && sensorPhysicalI->ewm.avg.get() < 0.05f && sensorPhysicalI->last < 0.05f) {
                    RT_COLD(
                    ESP_LOGI("mppt", "Sweep at D=%.2f with open backflow switch, ending it", converter.getDutyCycle());
                    _stopSweep(MpptControlMode::Sweep, -1, nullptr);
                    );
                    if (converter.disabled()) { // no-MPP backoff
                        ctrlState.mode = MpptControlMode::None;
                        return false;
                    }
                    // MPP committed: drop to it and keep protecting; returning false would stopAndBackoff()
                    converter.pwmPerturb((int16_t) ((int) targetDutyCycle - (int) converter.getCtrlOnPwmCnt()));
                } else {
                    RT_COLD(
                    if (!converter.disabled())
                        ESP_LOGE("MPPT", "High duty cycle with open backflow switch! (Idcm=%.2f Vin=%.2f Vout=%.2f)", iDcm, vi, vo);
                    shutdownDcdc("highD-bflowOpen");
                    );
                    return false;
                }
            }
        }

        if (!converter.syncRectEnabled_()) {
            if (sensorPhysicalI->ewm.avg.get() > 6) {
                RT_COLD(
                if (!converter.disabled())
                    ESP_LOGE("MPPT", "High current without sync rectification!");
                shutdownDcdc("highI-noSyncRect");
                );
                return false;
            }
        }

        // TODO move this to control loop
        //float vOut = fmaxf(sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get());
        //float vIn = fminf(sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
        float vOut = sensors.Vout->ewm.avg.get();
        float vIn = sensors.Vin->ewm.avg.get();
        // TODO smoothing!
        // protect() runs on every loop pass, but vIn/vOut only move when the sampler produces new
        // data - the forced-PWM gate measures its hold in voltage samples, so tell it which is which.
        const uint32_t voutSamples = sensors.Vout->numSamples;
        const bool freshV = (voutSamples != lastVoutSamples);
        lastVoutSamples = voutSamples;

        // buck DCM: the LS min pulse pumps the output back into the input above minPulseRatioMax(false). update() holds
        // Vin above the start ratio; a source it cannot move (stiff) shows as reverse current there. DCM entered above
        // the stop ratio (from CCM, stiff source) pumps at once: no debounce
        {
            const bool exposed = converter.minPulseExposed();
            const bool overStop = exposed && !(vOut < vIn * converter.minPulseRatioMax(false));
            if (overStop && !minPulseWasExposed)
                minPulseCnt = MinPulseDebounce;
            else if (freshV)
                minPulseCnt = overStop || (exposed && sensorPhysicalI->ewm.avg.get() < -0.05f &&
                                           !(vOut < vIn * converter.minPulseRatioMax(true)))
                                  ? (uint8_t) (minPulseCnt + 1) : 0;
            minPulseWasExposed = exposed;
        }
        if (minPulseCnt >= MinPulseDebounce) {
            minPulseCnt = 0;
            RT_COLD(
            ESP_LOGW("mppt", "Vout %.2f / Vin %.2f (I %.2f) at the LS min pulse ratio limit, shutdown", vOut, vIn,
                     sensorPhysicalI->ewm.avg.get());
            shutdownDcdc("minpulse-M");
            );
            return false;
        }
        const Sensor *il = converter.boost() ? sensors.Iin : sensors.Iout;
        if (const auto n = (uint8_t) (il->numSamples - ilSamples)) {
            ilSamples += n;
            // mean delay of ewm.avg in samples: 2-pass EWMA span-1, median 2 (despike passes through), comb L/2
            converter.decayDrop(n, il->ewm.span() - 1 + (Sensor::despikeK > 0 ? 0 : 2) +
                                   (il->comb ? (uint32_t) (il->comb->length() * .5f) : 0));
        }
        auto vr = converter.updateSyncRectMaxDuty(vIn, vOut, il->ewm.avg.get(), freshV);

        auto iOutSmall = sensorPhysicalI->ewm.avg.get() < (limits.Iout_max * 0.01f);

        if (iOutSmall && converter.getCtrlOnPwmCnt() > converter.pwmRectMin * 2 and
            // Requested, not gated: a board that asks for forced PWM does so because its current
            // sensor is unusable, and iOutSmall is then permanently true. Judging it by the strict
            // branch while the bring-up gate is still armed trips this mid-ramp - the more so as
            // getDutyCycle() normalizes to pwmCtrlMax, ~8.5% short of a period on a buck.
            (converter.forcedPwmRequested()
                 ? (vOut < 1 or (converter.getDutyCycle() * 0.5f) > vr)
                 : (converter.getDutyCycle() * 0.8f) > vr)
            and limits.reverse_current_paranoia
            and !(g_app.psuMode() && vOut > 0.5f * psuVsetpoint && vOut < psuVsetpoint)) {
            RT_COLD(
            if (!converter.disabled())
                ESP_LOGE("MPPT",
                     "Buck D=%d%% but Vout(%.2f,vr=%.2f) Iout(%.2f,last=%.2f) low! sensor/HB fail",
                     100 * converter.getCtrlOnPwmCnt() / converter.pwmCtrlMax, vOut, vr,
                     sensors.Iout->ewm.avg.get(),
                     sensors.Iout->last
            );

            shutdownDcdc("Vr-sensor-fail");
            );
            return false;
        }

        lastTimeProtectPassed = nowMs;

        return true;
    }


    /**
     * Start a global MPPT scan.
     */
    void startSweep() {
        _sweeping = true;
        if (_resweepAt) _retryAt = _resweepAt = 0; // pre-empted: it never ran, so it doesn't count to the cap
        _svPhase = _svFlag = 0;
        _capV = 0;

        converter.disable();
        ctrlState._limiting = false;
        ctrlState.limIdx = 15;
        _limitPert = 0;
        targetDutyCycle = 0;

        VinController.reset();
        VoutController.reset();
        IinController.reset();
        IoutCurrentController.reset();
        //LoadRegulationCTRL.reset();

        ESP_LOGI("mppt", "Start sweep");


        maxPowerPoint = {};

        // after a failed calibration sweep once on the old zeros: a persistent failure must not block every start
        if (sampler.takeCalibFailure()) ESP_LOGW("mppt", "Last calibration failed, sweep on the old zeros");
        else sampler.startCalibration();

        sweepPlot.restart();
        rt_defer([](void *c, float, float) {
            auto m = (MpptController *) c;
            m->sweepPlot.begin();
            rtcount_en = false;
            vTaskDelay(10);
            m->meter.commit(); // not real-time safe
            m->lcd.periodicInit(); // not real-time safe
            rtcount_en = true;
        }, this);
    }

    // Called from the console task (core 0). Stores the target only; the RT loop
    // ramps pwmCtrl on core 1 — keeps all PWM writes on one core so a concurrent
    // disable() can't race an in-flight ledc_update_duty (LEDC has no force-low latch).
    void setManualTarget(uint16_t duty) {
        if (duty > converter.pwmCtrlMax) duty = converter.pwmCtrlMax;
        manualTarget = duty;
    }

    // Drop the config-derived boot duty cap so update() resumes normal MPPT tracking.
    void clearBootTarget() { targetPwmCnt = 0; }

    // RT-CORE ONLY. Applies a queued `pwm-freq` change and carries the duty state that lives HERE
    // through it. manualTarget, the boot duty cap, the one-shot ramp target and the tracker's
    // captured MPP are all raw PWM counts: rescaling only the converter would leave the very next
    // control tick ramping back toward a count that no longer means the same duty — on a bench in
    // manual PWM that undoes the frequency change's operating point within milliseconds.
    void applyPendingPwmFreqRt() {
        const float r = converter.applyPendingPwmFreqRt();
        if (!(r > 0.f)) {
            converter.ackPendingPwmFreqRt();
            return;
        }
        RT_COLD(
        const uint16_t hi = converter.pwmCtrlMax;
        auto rescale = [r, hi](uint16_t v) {
            return (uint16_t) std::min<int32_t>(std::lround((float) v * r), hi);
        };
        manualTarget.store(rescale(manualTarget.load(std::memory_order_relaxed)),
                           std::memory_order_relaxed);
        targetPwmCnt = rescale(targetPwmCnt);
        targetDutyCycle = rescale(targetDutyCycle);
        // Rescaled, not cleared: the operating point is preserved across the change, so a captured
        // MPP is still the same physical point — only its count changed. Clearing it would send a
        // sweep-driven fade to duty 0. There are TWO captures and both are raw counts: the tracker's
        // (P&O) and this class's own sweep capture, which _stopSweep() commits as targetDutyCycle.
        tracker.maxPowerPoint.dutyCycle = rescale(tracker.maxPowerPoint.dutyCycle);
        maxPowerPoint.dutyCycle = rescale(maxPowerPoint.dutyCycle);
        // Last: the console returns as soon as this lands, and the next command it dispatches must
        // find every count already in the new period's scale.
        converter.ackPendingPwmFreqRt();
        );
    }

    // One-shot automatic ramp target (consumed by update() sweep/MPP fade path).
    void setAutoRampTarget(uint16_t duty) {
        if (duty > converter.pwmCtrlMax) duty = converter.pwmCtrlMax;
        targetDutyCycle = duty;
    }

    float psuVsetpoint = NAN; // PSU CV setpoint (V); NAN = not commanded
    // OV base held over a downward PSU retarget: the old setpoint (or the measured Vout, if
    // higher) until Vout has come down into the new band. Without it `psu 20` from 49 V
    // derives a 30 V threshold at once and trips Vout-OV on the converter's own output.
    float psuOvHoldBase = NAN;
    // Downward PSU retarget slews psuVsetpoint toward psuTarget at psuSlewDownVps
    // (converter.conf::psu_slew_down). A step lets the CV loop pull the output down as fast as
    // forced PWM allows, and a boost returns that charge to its INPUT: fboost, unloaded, 49 -> 20 V
    // pumped its 10 V bench-supply rail to 19.8 V and latched psu-setpoint-infeasible (2026-10-09).
    // Upward steps stay immediate.
    float psuTarget = NAN;
    float psuSlewDownVps = 2.0f;
    std::atomic<bool> psuEscalated{false};
    std::atomic<bool> psuLatched{false};

    HsShortGuard hsGuard{};
    HsClampArm hsArm{};
    std::atomic<uint8_t> hsFault{0};
    std::atomic<char> hsWhy{0};
    std::atomic<bool> hsClearReq{false};
    float hsI0 = 0;
    uint32_t hsClampMs = 0, hsRampUs = 0, hsRampPos = 0; // ramp: last step, position (µs)
    uint8_t hsRise = 0, hsRampCnt = 0;
    std::atomic<bool> hsClampUnsafe{false}; // a clamp met battery current: disarmed until a fresh blocks pass (persisted)
    void hsLatchRt(uint32_t nowMs, float i, char why);
    void hsClearRt();
    void hsPendingRt(uint32_t nowMs, float i);
    void hsRampRt(uint32_t nowMs);
    void hsGatesOffRt(const char *why);
    void bfTestRt();
    void bfToggleRt(uint8_t m);
    void bfTestStopRt();

    // PV-sim (solar-array-simulator): the output follows the panel curve V = f(Iout).
    // RT-owned; while active, psuVsetpoint is advanced along the curve each tick
    // (pvAdvanceSetpoint) instead of being a constant. slewVps/iout span are boot-config
    // (begin(), before the RT loop runs).
    struct {
        bool active = false;
        PvModel model{};
        float slewVps = 200.f;
        EWMA<float> iout{16};
    } pvSim;

    void setPsuSetpoint(float v) {
        if (!std::isfinite(v) || v <= 0 || v > limits.Vout_max) return;
        float hold = g_app.psuMode() ? psuVsetpoint : NAN;
        const float vout = sensors.Vout ? sensors.Vout->last : NAN;
        if (std::isfinite(vout) && !(vout <= hold)) hold = vout;
        psuOvHoldBase = (std::isfinite(hold) && hold > v) ? hold : NAN;
        VoutController.reset();
        psuTarget = v;
        // slew down only from an active plain-PSU setpoint; PV-sim owns its own slew
        if (!(g_app.psuMode() && !pvSim.active && v < psuVsetpoint))
            psuVsetpoint = v;
        publishedPsuSetpoint.store(psuVsetpoint, std::memory_order_release);
    }

    // RT-only, plain PSU: advance psuVsetpoint one tick toward psuTarget (downward slew).
    void psuAdvanceSetpoint(float dt) {
        if (!(psuVsetpoint > psuTarget)) return;
        dt = std::min(dt > 0 ? dt : 6e-4f, 0.01f); // same stale-dt clamp as pvAdvanceSetpoint
        psuVsetpoint = std::max(psuTarget, psuVsetpoint - psuSlewDownVps * dt);
        publishedPsuSetpoint.store(psuVsetpoint, std::memory_order_release);
    }

    // Shared boost headroom: setpoint feasibility gate and the PV-sim curve floor.
    static constexpr float BoostHeadroomV = 0.5f;

    static PsuSetpointError validatePsuSetpoint(float requested, float voutMax, bool boost,
                                                float vin, float explicitOvLimit) {
        if (!std::isfinite(requested) || requested <= 0 || requested > voutMax)
            return PsuSetpointError::OutOfRange;
        // An explicit OV threshold is a hard protection setting, not a second CV target. Leave
        // the same 2% approach margin startBlockReason() uses so the requested operating point
        // cannot be inside the trip band by construction.
        if (std::isfinite(explicitOvLimit) && explicitOvLimit > 0
            && requested >= 0.98f * explicitOvLimit)
            return PsuSetpointError::OvLimitConflict;
        // A boost can regulate only ABOVE its input. Unknown Vin is not evidence that the
        // setpoint is feasible; entry waits for a fresh sample and rejects an unusable one.
        if (boost && !std::isfinite(vin))
            return PsuSetpointError::TelemetryUnavailable;
        if (boost && requested <= vin + BoostHeadroomV)
            return PsuSetpointError::BoostBelowInput;
        return PsuSetpointError::None;
    }

    [[nodiscard]] uint32_t queuePsuSetpoint(float v, bool bootRequest = false) {
        const auto precheck = validatePsuSetpoint(v, limits.Vout_max, false, NAN,
                                                  getExplicitOvLimit());
        if (precheck != PsuSetpointError::None) {
            lastPsuRequestError.store(precheck, std::memory_order_relaxed);
            return 0;
        }
        PsuReq r;
        r.v = v;
        r.boot = bootRequest;
        return postPsuCommand(PsuCommand::Enable, r);
    }

    [[nodiscard]] bool requestPsuSetpoint(float v, bool bootRequest = false) {
        return queuePsuSetpoint(v, bootRequest) != 0;
    }

    // The alpha family degenerates below k=0.5 (solver floors, ~linear curve) and k→1 is
    // pathological, hence the narrower range than vconv's model.
    [[nodiscard]] static bool validPvParams(float isc, float voc, float k) {
        return std::isfinite(isc) && isc > 0 && std::isfinite(voc) && voc > 0
               && std::isfinite(k) && k >= 0.5f && k <= 0.95f;
    }

    // rebase=false (`pv scale`) keeps pvBaseIsc as the reference for later scales.
    [[nodiscard]] uint32_t queuePvCurve(float isc, float voc, float k,
                                        bool bootRequest = false, bool rebase = true) {
        if (!validPvParams(isc, voc, k)) {
            lastPsuRequestError.store(PsuSetpointError::OutOfRange, std::memory_order_relaxed);
            return 0;
        }
        const auto precheck = validatePsuSetpoint(voc, limits.Vout_max, false, NAN,
                                                  getExplicitOvLimit());
        if (precheck != PsuSetpointError::None) {
            lastPsuRequestError.store(precheck, std::memory_order_relaxed);
            return 0;
        }
        PsuReq r;
        r.v = voc;
        r.isc = isc;
        r.k = k;
        // The base is committed by the RT consumer on success — a producer-side store would
        // survive an RT-side rejection and corrupt a later `pv scale`.
        r.rebase = rebase;
        r.boot = bootRequest;
        return postPsuCommand(PsuCommand::EnablePv, r);
    }

    // Coherent read of the published curve. Returns active; outputs are NAN when never set.
    // Callers are non-RT tasks (console/OTA/measure-coil). Bounded retry with a yield: an odd
    // seq can persist for a whole interrupt on the RT core, so spin briefly, then sleep-retry;
    // the final fallback (possibly-mixed read) only fires if RT died mid-publish — better than
    // wedging the caller forever.
    bool getPvCurve(float &isc, float &voc, float &k) const {
        PvCurve c;
        bool ok = false;
        for (int tries = 0; tries < 64 && !ok; ++tries) {
            if (tries >= 8) vTaskDelay(1);
            ok = pvPub.tryRead(c);
        }
        if (!ok) c = pvPub.get();
        isc = c.isc;
        voc = c.voc;
        k = c.k;
        return c.active;
    }

    [[nodiscard]] bool isPvActive() const { return pvPub.get().active; }
    [[nodiscard]] float getPvBaseIsc() const {
        return pvBaseIsc.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t requestPsuOff(bool keepBackoff = false) {
        PsuReq r;
        r.keepBackoff = keepBackoff;
        return postPsuCommand(PsuCommand::DisableMppt, r);
    }

    [[nodiscard]] uint32_t requestPsuManual(uint16_t duty, int manualRect, bool keepBackoff = false, bool stop = false) {
        PsuReq r;
        r.duty = duty;
        r.rect = manualRect;
        r.keepBackoff = keepBackoff;
        r.stop = stop;
        return postPsuCommand(PsuCommand::DisableManual, r);
    }

    [[nodiscard]] uint32_t requestPsuSweep() { return postPsuCommand(PsuCommand::StartSweep); }

    [[nodiscard]] uint32_t requestPsuShortLowSide() { return postPsuCommand(PsuCommand::ShortLowSide); }

    [[nodiscard]] bool hasPendingPsuCommand() const { return !psuCmd.idle(); }
    [[nodiscard]] bool isPsuCommandDone(uint32_t ticket) const {
        if (ticket == 0) return false;
        return (completedPsuResults[ticket % PsuResultSlots].load(std::memory_order_acquire) >> 8)
               == ticket;
    }
    [[nodiscard]] PsuSetpointError getPsuCommandError(uint32_t ticket) const {
        if (ticket == 0) return lastPsuRequestError.load(std::memory_order_relaxed);
        const uint32_t result =
            completedPsuResults[ticket % PsuResultSlots].load(std::memory_order_acquire);
        return (result >> 8) == ticket
               ? static_cast<PsuSetpointError>(result & 0xffu)
               : PsuSetpointError::None;
    }
    bool cancelPsuCommand(uint32_t ticket) {
        const uint32_t w = psuCmd.peek();
        return w && ReqSlot<8>::tag(w) == ticket && psuCmd.clear(w);
    }
    [[nodiscard]] float getPsuSetpoint() const {
        return publishedPsuSetpoint.load(std::memory_order_acquire);
    }
    [[nodiscard]] float getExplicitOvLimit() const {
        return explicitOvLimit.load(std::memory_order_acquire);
    }
    void setExplicitOvLimit(float v) {
        explicitOvLimit.store(v, std::memory_order_release);
    }
    [[nodiscard]] float getRequestedPsuSetpoint() const {
        const auto command = static_cast<PsuCommand>(ReqSlot<8>::payload(psuCmd.peek()));
        if (command == PsuCommand::Enable || command == PsuCommand::EnablePv)
            return psuReq.get().v;
        // PV-sim active: the commanded top of the curve, not the moving setpoint — `ovset`
        // validates against this, and an OV limit inside (moving-Vset, Voc) would latch.
        const PvCurve c = pvPub.get();
        if (c.active) return c.voc;
        return getPsuSetpoint();
    }
    [[nodiscard]] uint16_t getManualTarget() const {
        return manualTarget.load(std::memory_order_relaxed);
    }

    // RT-CORE ONLY. Console code posts a mailbox command; this is the single writer for the
    // controller resets, trip counters, sweep/backoff state and PSU setpoint transition.
    void applyPendingPsuCommandRt(bool freshTelemetry = true) {
        if (psuCmd.peek()) applyPendingPsuCommandSlow(freshTelemetry);
    }

    __attribute__((noinline, cold)) void applyPendingPsuCommandSlow(bool freshTelemetry) {
        const uint32_t seq = psuReq.enter();
        if (seq & 1u) return;
        const uint32_t word = psuCmd.peek();
        if (word == 0) return;
        const auto command = static_cast<PsuCommand>(ReqSlot<8>::payload(word));
        const uint32_t ticket = ReqSlot<8>::tag(word);
        // An Enable needs the current Vin for boost feasibility. Off/manual/sweep remain
        // available during ADC loss, but stale telemetry must never be used to energize.
        const bool pvCmd = command == PsuCommand::EnablePv;
        if ((command == PsuCommand::Enable || pvCmd) && !freshTelemetry) return;
        const PsuReq r = psuReq.get();
        if (!psuReq.still(seq) || !psuCmd.clear(word)) return;
        const float requested = r.v, pvIsc = r.isc, pvK = r.k;
        const bool pvRebase = r.rebase, bootRequest = r.boot;
        const uint16_t manualDuty = r.duty;
        const int manualRect = r.rect;

        if (command != PsuCommand::Enable && !pvCmd) {
            abortSweep();
            if (!r.keepBackoff) clearBackoff();
            converter.setManualRect(-1);
            rectCal.drop();
            psuVsetpoint = NAN;
            psuTarget = NAN;
            psuOvHoldBase = NAN;
            publishedPsuSetpoint.store(NAN, std::memory_order_release);
            pvDeactivateRt();
            psuResetTripState();
            if (!r.keepBackoff) releaseCvFloorLatch("psu off"); // it clears the backoff too
            lastPsuRequestError.store(PsuSetpointError::None, std::memory_order_relaxed);
            if (command == PsuCommand::DisableManual) {
                if (r.stop && !manualDuty) shutdownDcdc("stop", 0);
                setManualTarget(manualDuty);
                converter.setManualRect(manualRect);
                if (manualDuty != 0 && !limits.reverse_current_paranoia) {
                    converter.enableSyncRect(true);
                    bflow.enable(true);
                }
                g_app.opMode = OpMode::Manual;
                ESP_LOGI("mppt", "PSU off, manual PWM mode");
            } else if (command == PsuCommand::StartSweep) {
                g_app.opMode = OpMode::Mppt;
                clearBootTarget();
                startSweep();
                ESP_LOGI("mppt", "PSU off, sweep started");
            } else if (command == PsuCommand::ShortLowSide) {
                setManualTarget(0);
                g_app.opMode = OpMode::Manual;
                converter.shortLs();
                ESP_LOGI("mppt", "PSU off, low side shorted for diagnostic");
            } else {
                g_app.opMode = OpMode::Mppt;
                clearBootTarget();
                tracker.restart(sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN);
                ESP_LOGI("mppt", "PSU off, MPPT mode");
            }
            completePsuCommand(ticket, PsuSetpointError::None);
            return;
        }

#if WITH_PSU
        const float vin = sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN;
        // For a PV curve the no-load operating point IS Voc, so Voc carries the feasibility check.
        auto error = validatePsuSetpoint(requested, limits.Vout_max, converter.boost(), vin,
                                         getExplicitOvLimit());
        if (pvCmd && error == PsuSetpointError::None && !validPvParams(pvIsc, requested, pvK))
            error = PsuSetpointError::OutOfRange;
        lastPsuRequestError.store(error, std::memory_order_relaxed);
        if (error != PsuSetpointError::None) {
            const char *why = error == PsuSetpointError::TelemetryUnavailable
                              ? "fresh Vin telemetry unavailable"
                              : error == PsuSetpointError::BoostBelowInput
                              ? "boost setpoint must exceed Vin by 0.5 V"
                              : error == PsuSetpointError::OvLimitConflict
                                ? "setpoint conflicts with explicit OV threshold"
                                : "setpoint out of range";
            ESP_LOGE("mppt", "PSU request %.2f V rejected: %s (Vin %.2f V, OV %.2f V)",
                     requested, why, vin, getExplicitOvLimit());
            if (bootRequest)
                g_app.setupFail("PSU boot setpoint: %s", why);
            completePsuCommand(ticket, error);
            return;
        }
        if (pvCmd && pvK * requested < vin + 2)
            ESP_LOGW("mppt", "PV MPP %.1f V is below the boost floor (Vin %.1f V) — knee not emulatable",
                     pvK * requested, vin);
        if (pvCmd && pvSim.active) {
            // In-place curve update: keep the current setpoint (the slew limiter walks it to
            // the new curve), no controller reset — a jump to Voc under load is the excursion
            // the limiter exists to prevent. Only a deliberate full `pv <isc> <voc>` re-issue
            // (rebase set) is the unlatch escape hatch, like `psu <v>`; a `pv scale` or a
            // save/restore must not silently cancel a fault backoff or the trip history.
            if (pvRebase) {
                psuResetTripState();
                clearBackoff();
            }
            pvSim.model.set(pvIsc, requested, pvK);
            if (pvRebase) pvBaseIsc.store(pvIsc, std::memory_order_relaxed);
            publishPvState(true, pvIsc, requested, pvK);
            ESP_LOGI("mppt", "PV curve update, Isc=%.2fA Voc=%.2fV k=%.2f", pvIsc, requested, pvK);
            completePsuCommand(ticket, PsuSetpointError::None);
            return;
        }
        abortSweep();
        clearBackoff();
        converter.setManualRect(-1);
        rectCal.drop();
        psuResetTripState();
        if (pvCmd) {
            pvSim.model.set(pvIsc, requested, pvK);
            pvSim.iout.reset();
            pvSim.active = true;
            if (pvRebase) pvBaseIsc.store(pvIsc, std::memory_order_relaxed);
            publishPvState(true, pvIsc, requested, pvK);
            setPsuSetpoint(requested);
            g_app.opMode = OpMode::Psu;
            ESP_LOGI("mppt", "PV-sim mode, Isc=%.2fA Voc=%.2fV k=%.2f", pvIsc, requested, pvK);
        } else {
            pvDeactivateRt(); // a plain `psu <V>` while PV is active reverts to fixed CV
            setPsuSetpoint(requested);
            g_app.opMode = OpMode::Psu;
            ESP_LOGI("mppt", "PSU mode, vset=%.2fV", requested);
        }
        completePsuCommand(ticket, PsuSetpointError::None);
#else
        (void) requested; (void) pvIsc; (void) pvK; (void) pvRebase; (void) bootRequest;
        completePsuCommand(ticket, PsuSetpointError::OutOfRange);
#endif
    }

    // RT-only, PV-sim active: advance psuVsetpoint one tick along the curve.
    // No controller reset (a per-tick reset would kill the Vout PD's derivative).
    void pvAdvanceSetpoint(float iout, float vin, float dt);

    [[nodiscard]] PsuSetpointError getLastPsuRequestError() const {
        return lastPsuRequestError.load(std::memory_order_relaxed);
    }

    [[nodiscard]] PsuSetpointError currentPsuFeasibility() const {
        const float vin = sensors.Vin ? sensors.Vin->ewm.avg.get() : NAN;
        // PV-sim: judge feasibility at the curve top (Voc), not the moving setpoint — the
        // per-tick Vin-floor clamp keeps the moving value feasible by construction, so the
        // infeasibility latch fires only when Vin rises to within the headroom of Voc.
        const float v = pvSim.active ? pvSim.model.voc : psuVsetpoint;
        return validatePsuSetpoint(v, limits.Vout_max, converter.boost(), vin,
                                   getExplicitOvLimit());
    }

    void psuResetTripState() {
        psuTripCount.store(0, std::memory_order_relaxed);
        psuTripWindowStart = 0;
        psuEscalated = false;
        psuLatched = false;
    }

    [[nodiscard]] uint8_t getPsuTripCount() const {
        return psuTripCount.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool isPsuEscalated() const {
        return psuEscalated.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool isPsuLatched() const {
        return psuLatched.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool bmsCutoff() const {
        if (converter.disabled() || converter.boost() || g_app.psuMode() || !charger.params.haveVbatMax())
            return false;
        return bmsCutoffSignature(sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(),
                                  std::max(1.f, limits.Iout_max * 0.05f),
                                  sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get(),
                                  charger.params.Vbat_max * 0.02f,
                                  sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
    }

    [[nodiscard]] float computeOvThreshold() const {
        const float ovLimit = getExplicitOvLimit();
        if (std::isfinite(ovLimit) && ovLimit > 0)
            return std::min(ovLimit, limits.Vout_max);
        // PV-sim: pin the OV band to Voc so it doesn't follow the moving setpoint down the curve.
        float psuBase = pvSim.active ? pvSim.model.voc : psuVsetpoint;
        if (!pvSim.active && psuOvHoldBase > psuBase) psuBase = psuOvHoldBase;
        if (g_app.psuMode() && std::isfinite(psuBase))
            return std::min(psuBase * (limits.reverse_current_paranoia ? 1.03f : 1.5f), limits.Vout_max);
        if (charger.params.haveVbatMax())
            return std::min(charger.params.Vbat_max * (limits.reverse_current_paranoia ? 1.03f : 1.5f),
                            limits.Vout_max);
        return limits.Vout_max;
    }

    struct CVP {
        MpptControlMode mode;
        PD_Control &crtl;

        struct {
            float actual, target;
        };
    };


    /**
     * Stops MPPT scan and set duty cycle to captured MPP
     * @param controlMode
     */
    void _stopSweep(MpptControlMode controlMode, int limIdx, CVP *limCtrl) {
        _sweeping = false;

        if (maxPowerPoint.power < sweepMinPower()) {
            // Marginal light: no real MPP found. Don't commit a phantom target (would strand the
            // converter at near-max duty). Drop the target and back off, 30 s doubling per repeat up to 300 s;
            // the converter then resumes via normal MPPT / the next periodic re-sweep once there's real power.
            targetDutyCycle = 0;
            // cut by the Vout cap before any power: the cap lifting ends the backoff (noMppBatCheck)
            _capV = limIdx == 1 ? limCtrl->target : 0;
            const uint32_t sec = std::min(30u << _noMpp.n, 300u);
            if (_noMpp.n < 4) ++_noMpp.n;
            ESP_LOGI("mppt", "Stop sweep: no MPP (best %.2fW < %.1fW), backing off %lus", maxPowerPoint.power,
                     sweepMinPower(), (unsigned long) sec);
            shutdownDcdc("sweep-no-mpp", sec);
            return;
        }

        _noMpp = {};
        _capV = 0;
        if (controlMode == MpptControlMode::CV && limIdx == 1) capNote(limCtrl->target);
        targetDutyCycle = maxPowerPoint.dutyCycle;
        // dim: the MPP is the largest of noisy samples, biased high; such a sweep teaches nothing either
        const bool verify = sweepVerify > 0 && maxPowerPoint.power >= 25 * sweepMinPower();
        _svPhase = 1, _svLandTrim = 1 | verify << 1;
        scan.onSweepMpp(maxPowerPoint.voltage, maxPowerPoint.power, 25 * sweepMinPower(), verify);
        // two or more maxima: the way back probes them (a bright buck sweep only)
        if (!converter.boost() && maxPowerPoint.power >= 25 * sweepMinPower() &&
            scan.startReturn(maxPowerPoint.voltage, maxPowerPoint.power, 1.05f * vinFloor(true), maxPowerPoint.dutyCycle,
                             (uint32_t) wallClockMs()))
            targetDutyCycle = 0, _svPhase = 3;

        auto z = outZ.finish((uint32_t) (wallClockUs() / 1000000));
        if (z.n)
            ESP_LOGI("mppt", "Ro fit %s: %.2f+-%.2f mOhm res %.0f mV span %.1f A n %hu", z.ok ? "ok" : "rejected",
                     z.r * 1e3f, z.se * 1e3f, z.res * 1e3f, z.span, z.n);

        ESP_LOGI("mppt",
                 "Stop sweep %.2fs mode=%s (lim=%i tgt=%.2f act=%.2f) PWM=%hu MPP=(%.1fW,%hu,%.1fV)",
                 (wallClockUs() - sampler.getCalibrationStartUs()) * 1e-6f,
                 MpptState2String[(uint8_t) controlMode], limIdx,
                 limCtrl ? limCtrl->target : NAN, limCtrl ? limCtrl->actual : NAN,
                 converter.getCtrlOnPwmCnt(), maxPowerPoint.power, maxPowerPoint.dutyCycle, maxPowerPoint.voltage
        );

        sweepPlot.handOver();
        rt_defer([](void *c, float p, float v) {
            auto m = (MpptController *) c;
            // TODO replace displayMessageF with a general message callback call
            m->lcd.displayMessageF("MPP Scan done\n%.1fW @ %.1fV", 6000, p, v);
            m->sweepPlot.plot(p, v);
        }, this, maxPowerPoint.power, maxPowerPoint.voltage);
    }

    void telemetry();

    // HS-short checks (selftest/hs_short.h): RT, once per new sample
    void hsShortRt(uint32_t nowMs);
    void hsClear() { hsClearReq.store(true, std::memory_order_release); }
    // latched: LS clamped on, all gates off, gates off until Vin stays low (clamp pending), LS width ramp
    static constexpr uint8_t HsClamped = 1, HsOff = 2, HsPending = 3, HsRamp = 4;
    // 0 none, else one of the latched states above
    [[nodiscard]] uint8_t hsFaultState() const { return hsFault.load(std::memory_order_relaxed); }
    // pending: how long Vin has been below HsClampArm::VinMax (RT-written, approximate from core 0)
    [[nodiscard]] uint32_t hsPendingHeldMs(uint32_t nowMs) const { return hsArm.heldMs(nowMs); }
    // I idle current, V idle voltage, P restored at boot
    [[nodiscard]] char hsFaultWhy() const { return hsWhy.load(std::memory_order_relaxed); }
    [[nodiscard]] float hsZero() const { return hsGuard.zero; }
    // before the RT task: a latch persisted across the reboot, gates off
    void hsRestore() {
        hsGuard.latched = true;
        hsWhy = 'P';
        hsFault = 2;
        bflow.lockOff.fetch_or(BackflowDriver::LockHsShort);
        converter.inhibitStart(true, SynchronousConverter::InhibitHs);
    }

    // board.conf: bflow_bat=1 declares a backflow switch between the converter output and the battery with the
    // current sensor on the battery side (idle HS check, `selftest bflow`); hs_clamp=1 opts into the LS clamp
    bool bfBat = false, hsClampConf = false;
    [[nodiscard]] bool hsIdleCheck() const { return bfBat && bflow && !converter.boost() && sensorPhysicalI == sensors.Iout; }
    // the physical current sensor reads below -1 A: a signed ADC, or (rising transform) raw 0 maps below -1 A
    // after the learnt zero. A falling transform's floor depends on the ADC's full scale: counted unidirectional
    [[nodiscard]] bool batIBipolar() const {
        const auto *s = static_cast<const PhysicalSensor *>(sensorPhysicalI);
        if (s->adc->bipolar(s->params.adcCh)) return true;
        const float ofs = s->params.calibrationConstraints.calibrateOffset && std::isfinite(s->calibrationAvg)
                              ? s->calibrationAvg : 0.f;
        return s->params.transform.factor > 0 && s->params.transform.apply(0.f) - ofs < -1.f;
    }
    std::atomic<bool> bfRecent{false}; // blocks phase of `selftest bflow` passed within 36 h (written by core 0)
    std::atomic<bool> bfFail{false};   // latched `selftest bflow` failure
    void bfFailSet(bool on) {
        bfFail.store(on);
        converter.inhibitStart(on, SynchronousConverter::InhibitBf);
    }
    // byVoltage: the latch did not judge a current, so an unlearnt zero does not matter
    [[nodiscard]] bool hsClampAllowed(bool byVoltage) const {
        return hsClampConf && hsIdleCheck() && batIBipolar() && !g_app.psuMode() &&
               !hsClampUnsafe.load(std::memory_order_relaxed) &&
               !bfFail.load(std::memory_order_relaxed) &&
               (byVoltage || std::isfinite(hsGuard.zero)) && bfRecent.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool limiting() const { return ctrlState._limiting; }

    // `selftest bflow` RT side (selftest/bflow_test.cpp)
    // toggle variant: BfToggle the RT owns the switch, BfOn wanted on; BfFloor trip below iFloor with the switch on,
    // and on D Vin < Vout - 0.3 V; BfNoBat: the LS at its minimum until Vout reaches vProof, then BfEngage; trip on
    // |I| > 0.5 A, Vin > vinMax or (switch on, engaged) D Vin < Vout - 1 V
    static constexpr uint8_t BfBlocks = 1, BfConducts = 2, BfEngage = 4, BfStop = 8, BfFloor = 16, BfToggle = 32,
                             BfOn = 64, BfNoBat = 128;
    struct {
        std::atomic<uint8_t> mode{0};
        std::atomic<bool> tripped{false};
        float iMin = INFINITY, i = NAN, vin = NAN, vout = NAN, d = NAN; // trip record
        float iFloor = 0, vinMax = 0, vProof = 0; // vProof: no battery, Vout that proves it follows the duty
        float iMax = INFINITY; // conducts, before BfEngage: stop above this forward current
        uint8_t floorHold = 0, holdN = 0, iSeen = 0; // fresh current samples before iFloor applies after a turn-on
        uint8_t why = 0; // toggle trip: 1 floor, 2 D Vin below Vout, 3 no-battery current, 4 Vin rose; 5 above iMax
    } bfTest;

    time_us lastUs = 0;

    void update(); // normal update
    void updateManual(); // manual mode

    // rect_offset self-calibration and backstop (rect_cal.h)
    RectCalCtl<SynchronousConverter> rectCal{};

    RectCalIn rectCalIn(uint32_t nowUs) const {
        return {nowUs, sensors.Vin->last, sensors.Vout->last, sensors.Iout->last, sensors.Vin->ewm.avg.get(),
                sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get(), sensorPhysicalI->ewm.avg.get(),
                fminf(sensorPhysicalI->ewm.avg.get(), std::max(sensorPhysicalI->last, sensorPhysicalI->previous)),
                g_app.opMode.load() == OpMode::Mppt && !sampler.isCalibrating(), _sweeping || scan.active(), bflow.state(),
                ctrlState._limPast || (ctrlState._limiting && ctrlState._limReal), !sensors.Iout->isVirtual, limits.Vin_max};
    }

    // RT, once per new sample, before protect()
    void rectCalRt() {
        if (!sensorPhysicalI) return;
        if (const uint16_t hs = rectCal.rt(converter, rectCalIn((uint32_t) wallClockUs()))) setAutoRampTarget(hs);
    }

    void rectCalLf(); // core 0
    void scanLf(bool flashSafe); // core 0
    void scanSave();             // core 0, flash write safe

private:
    // RT: in update() after protect() and the limiters; true while the run holds HS/LS.
    __attribute__((always_inline)) bool rectCalHold(const time_us &nowUs) {
        if (!sensorPhysicalI || !rectCal.cal.running()) return false;
        uint16_t hs;
        const bool held = rectCal.hold(converter, rectCalIn((uint32_t) nowUs), hs);
        if (hs) setAutoRampTarget(hs);
        return held;
    }
};
