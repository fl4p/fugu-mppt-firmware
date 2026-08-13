#pragma once

//#include <utility>
#include <utility>
#include <vector>
#include <string>
#include <functional>
#include <cassert>
#include <atomic>

#include <esp_log.h>
#include <esp_attr.h> // IRAM_ATTR

#include "adc.h"
#include "math/statmath.h"
#include "util.h"
#include "etc/rt.h"

#include "tele/scope.h"
#include "math/notch.h"
#include "math/noise.h"
#include "math/ripple_freq.h"


struct LinearTransform {
    float factor;
    float midpoint;

    inline float apply(float x) const { return (x - midpoint) * factor; }

    inline float apply_inverse(float y) const { return (y / factor) + midpoint; }

    //const static LinearTransform Identity;
};

struct CalibrationConstraints {
    float maxAbsValue;
    float maxStddev;

    bool calibrateOffset;
};

/*struct _SensorCalibrationState {
    std::vector<float> acc{};
    std::vector<uint32_t> num {};
};*/

struct SensorParams {
    const uint8_t adcCh;
    const LinearTransform transform;
    //uint16_t ewmaSpan;
    CalibrationConstraints calibrationConstraints;
    const std::string teleName;
    char unit;
    /**
     * Whether to capture each ADC conversion. Can produce a lot of networking data
     */
    bool rawTelemetry;
    //uint16_t filtLen;
};

struct Sensor {
    const SensorParams params;

    float last = NAN, lastRaw = NAN;
    float previous = NAN;
    uint32_t numSamples = 0;
    NotchFilter *notchFilter = nullptr; // filter 50/60 Hz inverter noise
    RunningMedian5<float> med3{}; // filter burst noise
    AdaptiveNoiseFilter anf{};
    inline static bool anfEnabled = false; // diagnostics-only; kept out of the RT path unless `anf on`
    // Decision-based ("glitch-safe") median: the unconditional median-of-5 rejects sparse glitches
    // but UNDER-READS a current that draws dense periodic pulses (a cheap inverter), discarding real
    // charge -> ~5% low power. When despikeK > 0 the sample passes through unchanged unless it is an
    // extreme outlier (|v-median| > despikeK * running mean-deviation), so the recurring load pulses
    // reach the mean (unbiased) while genuine impulse glitches are still clipped. See conf `despike`.
    inline static float despikeK = 0;               // outlier threshold in robust-scale units; 0 = off
    float despikeScale = 0;                         // EWM of |v-median|, the robust scale
    uint32_t despikeTrips = 0;                      // cumulative clips (diagnostic + WARN)
    EWM<false, float> ewm; // filter residual noise
    MeanAccumulator calibBuffer{}; // for offset calibration

    float calibrationAvg = 0; // stores the mean value from calibBuffer
    bool calibrationComplete = false;

    const bool isVirtual;

    /*
    Sensor(SensorParams params, uint32_t ewmSpan)
            : params(std::move(params)), ewm{ewmSpan}, isVirtual(false) {
        anf.begin(ewmSpan, 0.2f);
    } */


    Sensor(const Sensor &other) = delete; // no-copy
    Sensor &operator=(const Sensor &) = delete; // no-copy

    void reset(bool resetCalibration = false) {
        last = NAN;
        previous = NAN;
        numSamples = 0;
        med3.reset();
        despikeScale = 0;
        ewm.reset();
        if (resetCalibration) {
            if (calibrationAvg != 0)
                ESP_LOGI("sensor", "%s reset calibration", params.teleName.c_str());
            calibrationAvg = 0;
            calibBuffer.clear();
            calibrationComplete = false;
        }
    }

    void add_sample(float x) {
        // IRAM_ATTR
        auto v = params.transform.apply(x);

        //if( !isfinite(v)) {
        //    ESP_LOGW("s", "not-finite sensor val %f (name=%s, x=%f) ", v, params.teleName.c_str(), x);
        //}

        if (params.calibrationConstraints.calibrateOffset) {
            //ESP_LOGI("sensor", "%s %.4f offset=%.4f std=%.4f n=%u", params.teleName.c_str(), v, calibrationAvg,
            //         ewm.std.get(), numSamples);
            v -= calibrationAvg;
        }

        previous = last;
        last = v;
        lastRaw = x;

        if (notchFilter)notchFilter->filter(&last, &v, 1);
        float m = med3.next(v); // always feed the median window so it tracks the signal
        if (despikeK > 0.f) {
            float dev = fabsf(v - m);
            if (despikeScale > 0.f && dev > despikeK * despikeScale) {
                v = m; // extreme outlier (glitch) -> clip; recurring load pulses pass through
                ++despikeTrips;
            }
            // update robust scale AFTER the decision so an outlier can't lift its own threshold
            despikeScale = (despikeScale > 0.f) ? (despikeScale + 0.095f * (dev - despikeScale)) : dev;
        } else {
            v = m; // legacy: unconditional median
        }
        ewm.add(v);
        if (anfEnabled) anf.add(v);

        ++numSamples;

        //ESP_LOGD("s", "Sensor %s: add sample %.5f #%d (ewm avg %.5f)", teleName.c_str(), last, numSamples, ewm.avg.get());
    }

protected:
    Sensor(SensorParams params, uint32_t ewmSpan, bool isVirtual)
        : params(std::move(params)), ewm{ewmSpan}, isVirtual{isVirtual} {
        anf.begin(ewmSpan, 0.2f);
    }

    //virtual ~Sensor() = default;  // make class polymorphic (to enable dynamic_cast)
};

struct PhysicalSensor : public Sensor {
    AsyncADC<float> *adc{nullptr};

    explicit PhysicalSensor(AsyncADC<float> *adc, const SensorParams &params, uint32_t ewmSpan)
        : Sensor(params, ewmSpan, false),
          adc(adc) {
    }

    float notchFs = 0; // sample rate this sensor's notch was tuned for

    // Tunes (or creates) the notch to f0Hz. Skipped when f0 is above this sensor's Nyquist
    // (a notch can't be placed there); the caller scales f0 by each sensor's own fs.
    void retuneNotch(float f0Hz, float Q, float gain) {
        if (notchFs <= 0) return;
        float fNorm = f0Hz / notchFs;
        if (fNorm <= 0 || fNorm >= 0.45f) return;
        try {
            if (!notchFilter) notchFilter = new NotchFilter();
            notchFilter->begin(fNorm, gain, Q); // preserves filter state -> glitch-free retune
        } catch (const std::exception &ex) {
            ESP_LOGE("sampling", "error %s", ex.what());
        }
    }

    void createNotchFilter(float fs, float f0Hz, float Q, float gain) {
        notchFs = fs;
        retuneNotch(f0Hz, Q, gain);
        ESP_LOGI("sampling", "%s notch fs=%.1fHz f0=%.1fHz Q=%.0f", params.teleName.c_str(), fs, f0Hz, Q);
    }
};

struct VirtualSensor : public Sensor {
    std::function<float()> func;

    explicit VirtualSensor(std::function<float()> func, uint32_t ewmSpan, const char *teleName, char unit)
        : Sensor({255, {1, 0}, {}, teleName, unit, false}, ewmSpan, true),
          func(std::move(func)) {
    }
};

template<typename T>
struct VIinVout {
    T Vin{}, Vout{}, Iin{}, Iout{};

    VIinVout(const VIinVout &) = delete;

    VIinVout(const VIinVout &&) = delete;

    VIinVout(T vin, T vout, T iin, T iout) : Vin{vin}, Vout{vout}, Iin{iin}, Iout{iout} {
    }
};


/**
 *
 * Implements asynchronous interleaved sampling of multiple sensors. Each sensor uses an ADC channel.
 *
 * Implement
 * - ADC channel abstraction (for Vin, Vout, Iin)
 * - Channel cycling
 * - Zero current calibration
 * - Exponentially weighted moving average (EWMA) filtering
 */
class ADC_Sampler {
public:
    bool ignoreCalibrationConstraints = false; // for testing

    //typedef Sensor Sensor;
    //using Sensor = Sensor;

private:
    struct AdcState {
        AsyncADC<float> *adc{nullptr};
        std::array<Sensor *, 8> sensorByCh{};
        std::vector<Sensor *> cycleSensors{}; // distinct sensors, only if readMode==MuxedRoundRobin
        std::vector<uint8_t> cycleOrder{};    // poll order into cycleSensors (Vout interleaved)
        uint8_t cycleSensorsPos = 0;          // position in cycleOrder
    };

    // Other cores only publish requests here. update() owns every sensor/filter mutation.
    enum class CalibrationState : uint32_t { Idle, StartRequested, Active, CancelRequested, Cancelling };
    std::atomic<CalibrationState> calibrationState_{CalibrationState::Idle};
    uint8_t calibrating_ = 0; // remaining sensors; RT task only
    time_us timeLastCalibration = 0;

    void applyCalibrationCommandRt() {
        auto state = calibrationState_.load(std::memory_order_acquire);
        if (state == CalibrationState::StartRequested) {
            if (!calibrationState_.compare_exchange_strong(
                    state, CalibrationState::Active, std::memory_order_acq_rel))
                return;

            calibrating_ = realSensors.size();
            for (auto &ch: sensors) {
                ch->reset(true);
                if (!ch->isVirtual)
                    static_cast<PhysicalSensor *>(ch)->adc->reset(ch->params.adcCh);
            }
        } else if (state == CalibrationState::CancelRequested) {
            if (!calibrationState_.compare_exchange_strong(
                    state, CalibrationState::Cancelling, std::memory_order_acq_rel))
                return;

            ESP_LOGI("mppt", "Cancel calibration");
            calibrating_ = 0;
            for (auto &ch: sensors) ch->reset(false);

            state = CalibrationState::Cancelling;
            calibrationState_.compare_exchange_strong(
                state, CalibrationState::Idle, std::memory_order_release, std::memory_order_relaxed);
        }
    }

    void finishCalibrationRt() {
        calibrating_ = 0;
        auto state = CalibrationState::Active;
        calibrationState_.compare_exchange_strong(
            state, CalibrationState::Idle, std::memory_order_release, std::memory_order_relaxed);
    }

public:
    volatile bool halted = false;


    std::function<void(const ADC_Sampler &sampler, const Sensor &)> onNewSample = nullptr;

    std::vector<AdcState> adcStates{};
    std::vector<Sensor *> sensors{}; // physical + virtual sensors
    std::vector<Sensor *> realSensors{}; // physical sensors from all ADCs
    std::vector<VirtualSensor *> virtualSensors{};

    // --- inverter-ripple notch auto-tuning -------------------------------------------------
    // The 2x-line ripple an inverter draws onto the DC bus corrupts the MPPT power estimate.
    // A detector watches the strongest-ripple channel (Vout) and retunes every sensor's notch
    // to the tone it finds, instead of assuming a fixed mains frequency. Runs entirely on the
    // RT core (detector fed + notches retuned here), so the biquad coeffs are never written
    // concurrently with filter().
    bool notchAdaptive = true;
    float notchFreq = 100.0f;          // active/fixed notch frequency [Hz]
    float notchQ = 20.0f, notchGain = -30.0f;
    static constexpr float NOTCH_SNR_MIN = 15.0f; // min peak/mean ratio to trust an estimate
    RippleFreqDetector<61> rippleDet{};
    const Sensor *rippleSrc = nullptr; // channel fed to the detector (Vout)
    float rippleSnr = 0;

    void configureNotch(bool adaptive, float freqHz, float Q, float gain) {
        notchAdaptive = adaptive;
        notchFreq = freqHz;
        notchQ = Q;
        notchGain = gain;
    }

    void setRippleSource(const Sensor *s) { rippleSrc = s; }

    // Glitch-safe (decision-based) median. k<=0 keeps the legacy unconditional median-of-5; k>0
    // enables it with that outlier threshold (in running-mean-deviation units, ~8 is sensible).
    void configureDespike(float k) { Sensor::despikeK = (k > 0.f) ? k : 0.f; }

    // Rate-limited WARN so the console always shows when the de-spiker is clipping outliers, without
    // flooding the RT path. Called once per update(); reports clips accumulated over each window.
    uint32_t despikeWarnDiv_ = 0, despikeTripsPrev_ = 0;
    void despikeWarnTick() {
        if (Sensor::despikeK <= 0.f || ++despikeWarnDiv_ < 2048) return;
        despikeWarnDiv_ = 0;
        uint32_t total = 0, worst = 0;
        const Sensor *worstS = nullptr;
        for (auto s: realSensors) {
            total += s->despikeTrips;
            if (s->despikeTrips > worst) { worst = s->despikeTrips; worstS = s; }
        }
        uint32_t delta = total - despikeTripsPrev_;
        despikeTripsPrev_ = total;
        if (delta)
            ESP_LOGW("sampling", "despike clipped %u outlier sample(s) (k=%.1f, mostly %s)",
                     delta, Sensor::despikeK, worstS ? worstS->params.teleName.c_str() : "?");
    }

    [[nodiscard]] float getNotchFreq() const { return notchFreq; }
    [[nodiscard]] float getRippleSnr() const { return rippleSnr; }

    void retuneNotches(float f0Hz) {
        for (auto s: realSensors)
            ((PhysicalSensor *) s)->retuneNotch(f0Hz, notchQ, notchGain);
    }


    AdcState &getAdcState(AsyncADC<float> *adc) {
        for (auto &s: adcStates)
            if (s.adc == adc)
                return s;
        return adcStates.emplace_back(AdcState{.adc = adc});
    }

    /**
     * Add a sensor with a given transform, a max expected value for ADC ranging and a name used for telemetry
     *
     * @param adcChannel ADC channel to read samples from
     * @param transform Transform applied to the samples
     * @param maxY Max expected value of the transformed sample (used to program the ADC PGA)
     */
    const Sensor *addSensor(AsyncADC<float> *adc, SensorParams params, float maxY, uint32_t ewmSpan) {
        assert(adc != nullptr);
        auto maxX = params.transform.apply_inverse(params.transform.factor < 0 ? -maxY : maxY);
        ESP_LOGI("sampler", "%s ADC ch %u maxY=%.4f, maxX=%.4f", params.teleName.c_str(), params.adcCh, maxY, maxX);
        adc->setMaxExpectedVoltage(params.adcCh, maxX);

        auto &sensorByCh(getAdcState(adc).sensorByCh);

        if (adc->readMode() == AdcReadMode::StreamedCallback)
            assert_throw(sensorByCh[params.adcCh] == nullptr, "duplicate sensor adc channel");

        auto sensorPtr = new PhysicalSensor{adc, params, ewmSpan};

        // todo scope:  adc->getSamplingRate() needs to be called after adc->setMaxExpectedVoltage()
        if (scope)
            scope->addChannel(adc, sensorPtr->params.adcCh, 'u', 12, sensorPtr->params.teleName.c_str());

        sensors.push_back(sensorPtr);
        realSensors.push_back(sensorPtr);
        if (!sensorByCh[sensorPtr->params.adcCh])
            sensorByCh[sensorPtr->params.adcCh] = sensorPtr;
        if (adc->readMode() == AdcReadMode::MuxedRoundRobin)
            getAdcState(adc).cycleSensors.push_back(sensorPtr);

        return sensorPtr;
    }

    const Sensor *addVirtualSensor(std::function<float()> func, uint32_t ewmaSpan, const char *teleName, char unit) {
        virtualSensors.push_back(new VirtualSensor{std::move(func), ewmaSpan, teleName, unit});
        sensors.push_back(virtualSensors.back());
        return sensors.back();
    }

    void _readNext(AdcState &state) {
        auto r = std::find_if(state.sensorByCh.begin(), state.sensorByCh.end(), [](Sensor *s) { return !!s; });
        assert(r != state.sensorByCh.end());
        state.adc->startReading((*r)->params.adcCh);
    }

    // Vout (added last, the OV-protection input) is interleaved between the other channels so its
    // refresh latency on a muxed ADC drops from N polls to 2 -> order [0,V,1,V,...]. Doubles Vout's
    // sample rate and halves the others'; effectiveSampleRate() compensates the notch tuning.
    static void buildCycleOrder(AdcState &s) {
        auto &order = s.cycleOrder;
        order.clear();
        uint8_t n = (uint8_t) s.cycleSensors.size();
        if (n > 2) {
            uint8_t vout = n - 1;
            for (uint8_t i = 0; i < vout; ++i) {
                order.push_back(i);
                order.push_back(vout);
            }
        } else {
            for (uint8_t i = 0; i < n; ++i) order.push_back(i);
        }
    }

    // getSamplingRate() reports the uniform per-channel rate (base/N). A muxed ADC with an
    // interleaved Vout samples each sensor at a different duty, so scale by N*appearances/pollLen.
    float effectiveSampleRate(PhysicalSensor *ps) {
        float fs = ps->adc->getSamplingRate(ps->params.adcCh);
        auto &s = getAdcState(ps->adc);
        if (!s.cycleOrder.empty()) {
            uint8_t app = 0;
            for (auto o: s.cycleOrder) if (s.cycleSensors[o] == ps) ++app;
            fs = fs * (float) s.cycleSensors.size() * (float) app / (float) s.cycleOrder.size();
        }
        return fs;
    }


    /**
     * user must call this from the same task that perform ADC reading (calls hasData & getSample)
     */
    void begin() {
        assert_throw(!adcStates.empty(), "adc null");
        assert_throw(!realSensors.empty(), "");

        for (auto &s: adcStates) {
            s.adc->start();
            if (s.adc->readMode() == AdcReadMode::MuxedRoundRobin) {
                assert_throw(!s.cycleSensors.empty(), "no sensors to cycle");
                buildCycleOrder(s);
                s.cycleSensorsPos = 0;
                s.adc->startReading(s.cycleSensors[s.cycleOrder[0]]->params.adcCh);
            } else if (s.adc->readMode() != AdcReadMode::StreamedCallback) {
                _readNext(s); // SnapshotAllChannels: prime the first channel
            }
            // StreamedCallback: driven by read(cb), nothing to prime
        }


        for (auto &s: sensors) {
            if (s->isVirtual) continue;
            auto ps = (PhysicalSensor *) s;
            ps->createNotchFilter(effectiveSampleRate(ps), notchFreq, notchQ, notchGain);
        }

        if (notchAdaptive && rippleSrc) {
            float fs = effectiveSampleRate((PhysicalSensor *) rippleSrc);
            // scan 80..140 Hz (50/60 Hz inverters -> 100/120 Hz, plus off-grid drift); ~0.75 s window
            uint16_t blockN = (uint16_t) std::min(std::max(fs * 0.75f, 128.f), 1500.f);
            rippleDet.configure(fs, 80.f, 140.f, blockN);
            ESP_LOGI("sampling", "ripple notch auto-tune on %s fs=%.0fHz block=%u",
                     rippleSrc->params.teleName.c_str(), fs, blockN);
        }
    }

    void startCalibration() {
        auto previous = calibrationState_.exchange(CalibrationState::StartRequested, std::memory_order_acq_rel);
        if (previous == CalibrationState::Idle)
            ESP_LOGI("mppt", "Start calibration");
    }

    void cancelCalibration() {
        auto state = calibrationState_.load(std::memory_order_acquire);
        while (state != CalibrationState::Idle && state != CalibrationState::CancelRequested &&
               state != CalibrationState::Cancelling) {
            if (calibrationState_.compare_exchange_weak(
                    state, CalibrationState::CancelRequested, std::memory_order_acq_rel))
                return;
        }
    }

    enum class UpdateRet : uint8_t {
        NoNewData = 0,
        NewData,
        Calibrating,
        CalibFailure,
        AdcError,
    };

    UpdateRet handleSensorCalib(Sensor &sensor) {
        if (calibrationState_.load(std::memory_order_acquire) == CalibrationState::Active &&
            !sensor.calibrationComplete && sensor.numSamples >= 100) {
            // calibZeroCurrent = ewm.s.chIin.avg.get();
            sensor.calibBuffer.add(sensor.last);

            if (sensor.calibBuffer.num > 100) {
                auto avg = sensor.calibBuffer.pop(); //sensor.ewm.avg.get();
                auto std = sensor.ewm.std.get();

                auto &constrains{sensor.params.calibrationConstraints};

                if (!ignoreCalibrationConstraints && (!std::isfinite(avg) or std::fabs(avg) > constrains.maxAbsValue)) {
                    ESP_LOGE("sampler", "Calibration failed, %s abs value %.6f > %.6f (last=%.6f, stdn=%.6f)",
                             sensor.params.teleName.c_str(), std::fabs(avg), constrains.maxAbsValue, sensor.last, std);
                    finishCalibrationRt();
                    //startCalibration();
                    return UpdateRet::CalibFailure;
                }

                if (!ignoreCalibrationConstraints &&
                    (std::abs(avg) < 1e-9f or std::isfinite(std)) // std can be non-finite for 0 values
                    and std * std::abs(avg) > constrains.maxStddev) {
                    ESP_LOGE("sampler", "Calibration failed, %s stddev %.6f > %.6f (last=%.6f, avg=%.6f)",
                             sensor.params.teleName.c_str(),
                             std * avg,
                             constrains.maxStddev, sensor.last, avg);
                    ESP_LOGW("sampler", "%s last=%.6f med3=%.6f avg=%.6f num=%lu", sensor.params.teleName.c_str(),
                             sensor.last,
                             sensor.med3.get(), sensor.ewm.avg.get(), sensor.numSamples);
                    finishCalibrationRt();
                    //startCalibration();
                    return UpdateRet::CalibFailure;
                }

                // TODO peak2peak

                sensor.calibrationAvg = avg;
                sensor.reset(false);
                sensor.calibrationComplete = true;

                ESP_LOGI("sampler", "Sensor %s calibration: avg=%.4f std=%.6f", sensor.params.teleName.c_str(), avg,
                         std);

                if (sensor.params.calibrationConstraints.calibrateOffset)
                    ESP_LOGI("sampler", "Sensor %s offset-calibrated: %.6f", sensor.params.teleName.c_str(), avg);

                --calibrating_;

                assert(calibrating_ < realSensors.size());

                if (calibrating_ == 0) {
                    auto state = CalibrationState::Active;
                    if (calibrationState_.compare_exchange_strong(
                            state, CalibrationState::Idle, std::memory_order_release, std::memory_order_relaxed)) {
                        ESP_LOGI("sampler", "Calibration done!");
                        timeLastCalibration = wallClockUs();
                    }
                    return UpdateRet::Calibrating;
                }
            }
        }

        return UpdateRet::NoNewData;
    }

    UpdateRet _addSensorSample(Sensor *sensor, float v) {
        if (unlikely(isnan(v)))
            return UpdateRet::CalibFailure;

        sensor->add_sample(v);
        rtcount("adc.update.addSample");

        if (notchAdaptive && sensor == rippleSrc && rippleDet.configured()) {
            rippleDet.push(sensor->last); // pre-notch value -> full ripple; Goertzel ignores DC
            rtcount("adc.update.rippleDetect");
        }

        if (onNewSample) {
            onNewSample(*this, *sensor);
            rtcount("adc.update.onNewSample");
        }

        auto calibRes = handleSensorCalib(*sensor);
        rtcount("adc.update.handleSensorCalib");
        return calibRes;
    }


    UpdateRet _updateAdc(AdcState &state) {
        auto adc = state.adc;

        auto hd = adc->hasData();
        rtcount("adc.update.hasData");

        auto readMode = adc->readMode();
        UpdateRet calibRes = UpdateRet::NoNewData;

        if (readMode == AdcReadMode::StreamedCallback) {
            // Drain read() BEFORE the isGood() no-sample watchdog: read() is the only place the
            // watchdog refreshes (lastDataUs_) and the only DMA drain, so gating it on isGood()
            // self-latches a dead-ADC state that can never clear. Drain first (a live DMA
            // self-clears), then report AdcError when still stale (a truly dead DMA -> backoff +
            // resetPeripherals), independent of hd so the safety halt is preserved.
            if (hd) {
                adc->read([&](uint8_t ch, float v) {
                    auto cr = _addSensorSample(state.sensorByCh[ch], v);
                    if (cr > calibRes) calibRes = cr;
                });
                rtcount("adc.update.read");
            }
            if (!adc->isGood())
                return UpdateRet::AdcError;
            if (!hd)
                return UpdateRet::NoNewData;
        } else {
            if (!adc->isGood())
                return UpdateRet::AdcError;
            if (!hd)
                return UpdateRet::NoNewData;

            if (readMode == AdcReadMode::SnapshotAllChannels) {
                for (int i = 0; i < state.sensorByCh.size(); ++i) {
                    auto sensor = state.sensorByCh[i];
                    if (!sensor) continue;
                    adc->startReading(i);
                    rtcount("adc.update.startReading");

                    auto x = adc->getSample();
                    rtcount("adc.update.getSample");

                    auto cr = _addSensorSample(sensor, x);
                    if (cr > calibRes) calibRes = cr;
                }
            } else {
                //using namespace std::string_literals;
                //assert_throw(false, "cycle readMode"s + "not implemented");

                auto &order = state.cycleOrder;
                const auto sensor(state.cycleSensors[order[state.cycleSensorsPos]]);

                auto x = adc->getSample();
                rtcount("adc.update.getSample");
                state.cycleSensorsPos = (state.cycleSensorsPos + 1) % order.size();
                state.adc->startReading(state.cycleSensors[order[state.cycleSensorsPos]]->params.adcCh);
                rtcount("adc.update.startReading");
                calibRes = _addSensorSample(sensor, x);
            }
        }

        if (calibRes != UpdateRet::NoNewData)
            return calibRes;

        /*if(calibrationState) {
            calibrationState->acc[cycleCh] += sensor.last;
            ++calibrationState->num[cycleCh];
        }*/


        return isCalibrating() ? UpdateRet::Calibrating : UpdateRet::NewData;
    }

    UpdateRet update() {
        UpdateRet res = UpdateRet::NoNewData;

        if (unlikely(halted)) {
            vTaskDelay(10);
            return res;
        }

        applyCalibrationCommandRt();

        bool updateVirtual = false;
        for (auto &state: adcStates) {
            auto r = _updateAdc(state);
            if (r > res) res = r;
            // virtuals update once per complete channel set: non-muxed ADCs read every channel
            // each poll; a muxed ADC completes its set when the round-robin wraps to 0.
            if (state.adc->readMode() != AdcReadMode::MuxedRoundRobin || state.cycleSensorsPos == 0)
                updateVirtual = true;
        }

        // update virtual sensors
        // TODO virtual sensor calibration?
        if (!isCalibrating() && updateVirtual) {
            for (auto &sn: virtualSensors) {
                sn->add_sample(sn->func());
                rtcount("adc.update.AddSampleVirtual");
            }
        }

        if (notchAdaptive && rippleDet.configured()) {
            float hz, snr;
            if (rippleDet.poll(hz, snr)) {
                rippleSnr = snr;
                if (snr >= NOTCH_SNR_MIN) {
                    // slew toward the estimate so a single noisy block can't yank the notch
                    float target = notchFreq + 0.5f * (hz - notchFreq);
                    if (fabsf(target - notchFreq) > 0.3f) {
                        notchFreq = target;
                        retuneNotches(notchFreq);
                        ESP_LOGI("sampling", "ripple notch -> %.1fHz (snr=%.0f)", notchFreq, snr);
                        rtcount("adc.update.retuneNotch");
                    }
                }
            }
        }

        despikeWarnTick();

        return res;
    }


    [[nodiscard]] bool isCalibrating() const {
        return calibrationState_.load(std::memory_order_acquire) != CalibrationState::Idle;
    }

    [[nodiscard]] time_us getTimeLastCalibrationUs() const { return timeLastCalibration; }


    void reInitADCs() {
        ConfFile boardConf{"/littlefs/conf/board.conf"};
        for (auto &s: adcStates) {
            s.adc->deinit();
            s.adc->init(boardConf);
            s.adc->start();
        }
    }

    bool resetPeripherals() {
        bool ok = true;
        for (auto &s: adcStates) {
            ok = s.adc->resetPeripherals() and ok;
        }
        return ok;
    }
};
