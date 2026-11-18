#include "mppt.h"

#include "app_state.h"        // g_app.maxLoopLag (peak RT-loop lag for telemetry)
#include "tele/telemetry.h"   // makeTelePoint / TelePoint (text or binary wire, build-time)
#ifdef WITH_BLE_TELE
#include "tele/tele_ble.h"    // teleBleStreaming gate
#endif
#include "tele/tele_mqtt.h"
#include <esp_heap_caps.h>
#include "etc/idle_profile.h"


constexpr auto withDebugFields = false;

uint32_t g_teleMinPointUs = kTeleMinPointUs; // idle profile, `pwr tele`
const char MpptController::kIdleBlock[] = "idle profile";

// Controller output -> duty slew rate [normalized duty per second]. The loop gain of a controller
// is Kp * this, so keep it named rather than as a literal buried in the update path.
// updateCV() additionally applies a low-duty gate factor on top (see there).
static constexpr float kCtrlSlewLimit = 25.f * 2.f / 2000.f; // limiter path, update()

// Gain loading lives in pd_control.h (pdLoadGains) so it is reachable from the unit tests.

/**
 * - Energy counter
 * - voltage and current control
 * - calls mpp tracker
 */
void MpptController::update() {
    //auto nowMs = wallClockMs();
    auto &nowUs = wallClockUs();
    // 0 on the first update after a reset: suppresses the D component instead of dividing by it
    const float dtCtrl = lastUs ? (float) (nowUs - lastUs) * 1e-6f : 0.f;

    // a sweep that ends other than through _stopSweep (trip, abort, restart) is dropped here
    if (!_sweeping || sampler.isCalibrating()) outZ.reset();

    if (converter.disabled() && startBlockReasonRt()) {
        bflow.enable(false);
        ctrlState.mode = MpptControlMode::None;
        return;
    }

    auto I_phys_smooth = (sensorPhysicalI->ewm.avg.get());
    auto V_phys_smooth = (sensorPhysicalU->ewm.avg.get());
    //auto Vout(sensors.Vout->ewm.avg.get());
    float power_smooth = I_phys_smooth * V_phys_smooth;
    float power = power_smooth; // sensorPhysicalI->med3.get() * sensorPhysicalU->med3.get();

    //avgIin.add(adcSampler.last.s.chIin);
    //avgVin.add(adcSampler.last.s.chVin);
    //float smoothPower = avgIin.get() * avgVin.get();

    meter.add(sensors.Iout->med3.get() * sensors.Vout->med3.get(), power_smooth,
              sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), nowUs);
    rtcount("mppt.update.meterAdd");


    // Chip temp stands in only for an unfitted NTC; a fitted one reading NaN keeps the 25% derate.
    const float ntcFresh = ntc.lastFresh();
    float ntcTemp = !ntc.fitted() ? ucTemp.lastFresh()
                    : std::isnan(ntcFresh) ? ntcFresh : fmaxf(ntcFresh, ucTemp.lastFresh());

    fan.fanUpdateTemp(ntcTemp, power_smooth);
    rtcount("mppt.update.thermals");

    float powerLimit = limits.P_max;
    if (ntcTemp > limits.Temp_derate) {
        auto powerScale = (limits.Temp_max - ntcTemp) / (limits.Temp_max - limits.Temp_derate);
        assert(powerScale < 1);
        if (powerScale < 0) powerScale = 0;
        powerLimit = limits.P_max * powerScale;
    } else if (isnan(ntcTemp)) {
        powerLimit = limits.P_max * .25f;
    }

    //float powerLimit = std::min(thermalPowerLimit(ntcTemp), limits.P_max);

    // charge current. PV-sim: cap at the emulated Isc — once the curve is clamped at the
    // Vin floor the converter is a stiff CV source, and without this the only limit left
    // is the hardware Iout_max.
    float Iout_max = g_app.psuMode()
                         ? (pvSim.active ? min(limits.Iout_max, pvSim.model.isc * 1.1f)
                                         : limits.Iout_max)
                         : min(limits.Iout_max, charger.Iout_max());

    // periodic sweep / scan
    // Skip while the battery is full / output-voltage-limited (CV): there's no MPP to find,
    // and ramping duty from 0 would just dump a charge pulse into a full pack (recharge-after-full).
    // Also wait out any pending backoff — a scheduled re-sweep firing into an active trip
    // timer would stall under the same livelock that startCondition() guards against.
    bool batteryFull = charger.chargeHold() || ctrlState.mode == MpptControlMode::CV;
    // the sweep start runs from flash: let a flash op finish first
    // P&O cannot see the MPP in low light (power steps below its thresholds) and drifts toward Vout:
    // re-sweep once power has doubled since a low-power sweep's MPP (< 5 % of p_max)
    const auto sinceSweepUs = nowUs - sampler.getTimeLastCalibrationUs();
    const uint32_t nowMs = (uint32_t) (nowUs / 1000);
    const bool grew = maxPowerPoint.power >= sweepMinPower() && maxPowerPoint.power < 25 * sweepMinPower() &&
                      power_smooth > 2 * maxPowerPoint.power && sinceSweepUs > 120 * 1000000;
    const bool retry = _resweepAt && (int32_t) (nowMs - _resweepAt) >= 0;
    if (!g_app.psuMode() && !_sweeping && !batteryFull && !inBackoff() && !flashOpActive() &&
        (grew || retry || sinceSweepUs > 30 * 60000000)) {
        ESP_LOGI("mppt", "%s sweep & sensor calibration", grew ? "power-doubled" : retry ? "retry" : "periodic");
        g_app.maxLoopLag = 0; // restart the lag window so telemetry tracks per-sweep peak, not all-time
        startSweep();
        rtcount("mppt.update.startSweep");
        return;
    }


    constexpr auto CV = MpptControlMode::CV, CC = MpptControlMode::CC, CP = MpptControlMode::CP;

    if (g_app.psuMode() && pvSim.active)
        pvAdvanceSetpoint(sensors.Iout->med3.get(), sensors.Vin->ewm.avg.get(), dtCtrl);
    else if (g_app.psuMode())
        psuAdvanceSetpoint(dtCtrl);

    // Vin floor at the LS min pulse ratio; its D term must not see the setpoint step when the floor comes or goes
    if (minPulseFloor != converter.minPulseExposed()) minPulseFloor = !minPulseFloor, VinController.reset();

    std::array<CVP, 5> controlValues{
        CVP{
            CV, VinController, {
                sensors.Vin->med3.get(),
                vinFloor(minPulseFloor)
            }
        },
        CVP{
            CV, VoutController, {
                _sweeping ? sensors.Vout->ewm.avg.get() : sensors.Vout->med3.get(),
                g_app.psuMode() ? psuVsetpoint : charger.Vout_max()
            }
        }, // todo last or med3
        CVP{
            CC, IinController,
            {_sweeping ? sensors.Iin->ewm.avg.get() : sensors.Iin->med3.get(), limits.Iin_max}
        },
        CVP{
            CC, IoutCurrentController,
            {_sweeping ? sensors.Iout->ewm.avg.get() : sensors.Iout->med3.get(), Iout_max}
        },
        CVP{CP, powerController, {power_smooth, powerLimit}},
        //CVP{CC, LoadRegulationCTRL, {sensors.Iout->last, Iout_max * 1.5f}},
    };

    // TODO sum negative values

    CVP *limitingControl = nullptr;
    float limitingControlValue = std::numeric_limits<float>::infinity();

    bool past = false;
    for (auto &c: controlValues) {
        auto cv = c.crtl.update(c.actual, c.target, dtCtrl);
        past |= (c.crtl.Kp != 0 ? c.crtl.Kp : c.crtl.Kd) * c.crtl._prevE < 0; // past its limit, D aside

        if (!std::isfinite(cv) && !converter.disabled() && converter.getDutyCycle() > 0.01f) {
            RT_COLD(
            ESP_LOGW("mppt", "Control value %f not finite act=%.3f tgt=%.3f idx=%i", cv, c.actual, c.target,
                     int(&c -controlValues.begin()));
            shutdownDcdc("ctrl-nan");
            );
            cv = -1;
        }

        if (cv < limitingControlValue) {
            limitingControlValue = cv;
            limitingControl = &c;
        }
    }

    //auto limitingControl = std::min_element(controlValues.begin(), controlValues.end(),
    //                                        [](const CVP &a, const CVP &b) { return a.second < b.second; });

    MpptControlMode controlMode = MpptControlMode::None;
    float controlValue = 0;
    bool slowEntry = false;

    // A limiter that only touches (D-term on a ripple crest) must not steer the tracker: resetting its
    // direction on every touch and an upward step on every release walked the duty up through the MPP.
    // After a touch the following loops give back what the touch took and the tracker carries on.
    const bool limSustained = ctrlState._limiting && nowMs - _limitSinceMs > 250;
    ctrlState._limPast = past;
    bool keepTracker = false;

    if (limitingControlValue < 0) {
        // limit condition
        controlMode = limitingControl->mode;
        controlValue = limitingControlValue;

        if (!ctrlState._limiting) _limitSinceMs = nowMs, _limitPert = 0, ctrlState._limReal = false;
        // past the limit (P term): the duty stays taken and the tracker restarts, as for a long limit
        if (limitingControl->crtl.Kp * limitingControl->crtl._prevE < 0 || g_app.psuMode())
            ctrlState._limReal = true, _limitPert = 0;
        keepTracker = !limSustained && !ctrlState._limReal;
        ctrlState._limiting = true;
        auto limIdx = (int) (limitingControl - controlValues.begin());
        ctrlState.limIdx = limIdx;
        limIdxSampled.add(limIdx);
    } else {
        // no limit condition
        if (ctrlState._limiting) {
            // recover from limit condition
            ctrlState._limiting = false;
            ctrlState.limIdx = 15;
            if (limSustained || ctrlState._limReal) {
                controlMode = MpptControlMode::MPPT;
                controlValue = limitingControlValue;
                _limitPert = 0;
            }
        }
    }

    if (rectCalHold(nowUs)) {
        lastUs = nowUs;
        return;
    }

    // bounce at pwm boundary
    if (converter.getCtrlOnPwmCnt() == converter.pwmCtrlMax) {
        controlMode = MpptControlMode::CV;
        controlValue = -1;
    } else if (converter.getCtrlOnPwmCnt() == converter.pwmCtrlMin && !_sweeping) {
        controlMode = MpptControlMode::CV;
        controlValue = 1;
    }

    // THIS CAN FAIL:
    // assert((controlMode == MpptControlMode::None) == (controlValue == 0));

    rtcount("mppt.update.control");

    if (_sweeping && !sampler.isCalibrating()) {
        if (converter.disabled()) {
            // second gate on the trip timer (shutdownDcdc() already ends the sweep): re-enabling
            // inside a backoff is what turns a repeating protection trip into a log flood
            if (inBackoff()) return;
            scan.onSweep(sensors.Vin->ewm.avg.get(), nowMs);
            converter.pwmPerturb(1);
        }

        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::Sweep;
            // sweep_speed scales both the per-tick cap and the limit-tracking gain,
            // so it actually controls speed near P/I/V limits, not just far from them
            controlValue = std::min(limitingControlValue * (sweepSpeed * 0.25f / 5.0f), sweepSpeed);

            // capture MPP during sweep, this will be our target afterward.
            // Ignore sub-sweepMinPower() samples so a marginal-light sweep can't "peak" at a
            // near-max-duty phantom (dawn cold-start) and strand the converter there.
            scan.sweepSample(sensors.Vin->ewm.avg.get(), power_smooth);
            if (power_smooth > maxPowerPoint.power && power_smooth >= sweepMinPower()) {
                maxPowerPoint.power = power_smooth;
                maxPowerPoint.dutyCycle = converter.getCtrlOnPwmCnt();
                maxPowerPoint.voltage = sensors.Vin->med3.get();
            }

            sweepPlot.add(sensors.Vin->med3.get(), converter.getDutyCycle(), power, limits.Vin_max);

            // ascending ramp only: the move back to the MPP runs after _stopSweep
            if (!converter.disabled() && !converter.boost() && !g_app.psuMode() && !sensors.Iout->isVirtual &&
                bflow.state())
                outZ.add(sensors.Vout->med3.get(), sensors.Iout->med3.get(), (uint32_t) nowUs);
            rtcount("mppt.update.sweeping");
        } else {
            _stopSweep(controlMode, limitingControl ? int(limitingControl - controlValues.begin()) : -1,
                       limitingControl);
            rtcount("mppt.update.stopSweep");
            if (converter.disabled()) { // no-MPP backoff
                ctrlState.mode = MpptControlMode::None;
                return;
            }
        }
    } else if (targetDutyCycle) {
        if (controlMode == MpptControlMode::None or controlMode == MpptControlMode::MPPT or
            (controlMode == MpptControlMode::CV && converter.getCtrlOnPwmCnt() > targetDutyCycle)) {
            controlMode = MpptControlMode::Sweep;
            controlValue = (float) constrain(targetDutyCycle - converter.getCtrlOnPwmCnt(),
                                             -(converter.pwmCounts() / 64) - 1,
                                             converter.pwmCounts() / 128 + 1);
            if (std::fabs(controlValue) <= 1) {
                ESP_LOGI("mppt", "Reached target duty cycle %hu", targetDutyCycle);
                targetDutyCycle = 0;
                if (_svPhase == 1) _svPhase = 2, _svT = nowMs;
            }
        } else {
            ESP_LOGI("mppt", "PWM fade to %i stopped at controlMode %s", (int) targetDutyCycle,
                     MpptState2String[(int) controlMode]);
            targetDutyCycle = 0;
            if (_svPhase == 1) sweepVerdict(nowMs, 0, false);
        }
    }
    if (_svPhase == 2 && nowMs - _svT > std::clamp(powerSettleMs(), 1500.f, 3000.f))
        sweepVerdict(nowMs, power_smooth, !batteryFull && !ctrlState._limiting && !converter.disabled() && !g_app.psuMode());


    float currentThreshold = limits.reverse_current_paranoia
                                 ? (bflow.state() ? 0.05f : 0.2f)
                                 : (bflow.state() ? 0.0f : 0.1f); // hysteresis; // hysteresis
    float I_phys_smooth_min = I_phys_smooth; //std::min(I_phys_smooth, sensorPhysicalI->med3.get());
    bool aboveThres = (I_phys_smooth_min > currentThreshold
                       || (I_phys_smooth_min > 0.05 && converter.getDutyCycle() > 0.3f)
    );

    if (controlMode == MpptControlMode::None && !_sweeping && !targetDutyCycle && !g_app.psuMode() && !batteryFull &&
        !converter.boost()) {
        const uint16_t d = converter.getCtrlOnPwmCnt();
        const float t = scan.update(nowMs, sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get(), power, d,
                                    1.05f * vinFloor(true), 25 * sweepMinPower(), [this] { return powerSettleMs(); });
        if (!std::isnan(t)) {
            // the tracker pauses; back at the start point it carries on, on another peak it restarts
            controlMode = MpptControlMode::Sweep, keepTracker = true;
            controlValue = constrain(t - (float) d, -(float) (converter.pwmCounts() / 64) - 1,
                                     (float) (converter.pwmCounts() / 128) + 1);
        } else if (scan.ended == 5) {
            scheduleResweep(nowMs);
            scan.ended = 0;
        } else if (scan.ended) {
            if (scan.ended == 1 || scan.ended == 3) tracker.resume(scan.pRef);
            else tracker.resetDirection(false, sensors.Vin->ewm.avg.get()), tracker.maxPowerPoint.power = 0; // probed below the peak
            if (scan.ended == 4) targetDutyCycle = scan.d0;
            scan.ended = 0;
        }
    } else if (scan.active() && !_sweeping) {
        scan.abort();
        if (!targetDutyCycle && !g_app.psuMode()) targetDutyCycle = scan.d0;
    }

    if (controlMode == MpptControlMode::None) {
        if (g_app.psuMode()) {
            controlMode = MpptControlMode::CV;
            controlValue = limitingControlValue;
        } else {
            controlMode = MpptControlMode::MPPT;
            const bool slow = tracker.slowMode;
            controlValue = tracker.update(power, converter.getCtrlOnPwmCnt(), sensors.Vin->ewm.avg.get(),
                                          [this] { return powerSettleMs(); });
            controlValue *= speedScale;
            slowEntry = tracker.slowMode && !slow;
        }
    } else if (!keepTracker) {
        // tracker.resetTracker(power_smooth, controlValue > 0);
        tracker.resetDirection(controlValue > 0, sensors.Vin->ewm.avg.get());
    }
    rtcount("mppt.update.tracker");

    // always cap control value
    // TODO instead of capping, use fade-to-target. the tracker might return big jumps
    controlValue = std::min(controlValue, limitingControlValue);
    ctrlState.mode = controlMode;
    cntrlValue = controlValue;


    if (lastUs) {
        // normalize the control value to pwmMax and scale it with update rate to fix buck slope rate
        auto fp = controlValue * kCtrlSlewLimit * (float) converter.pwmCtrlMax * dtCtrl;
        if (!_sweeping && converter.getCtrlOnPwmCnt() < converter.pwmCtrlMin * 2) {
            // slow-down control loop for low duty cycles (low-load condition)
            // TODO does this makes sense? the aim here is to stabilize Vout in low/no-load condition
            // can also slow-down the VoutCNTRL
            //fp *= 0.2f;
        }

        // constrain the buck step, this will slow down control for lower loop rates:
        // this causes very slow load response time, but works well when battery is connected
        const float fpMax = 16.0f * (float) converter.pwmCtrlMax / 2000.f;
        if (ctrlState._limiting) {
            if (!ctrlState._limReal)
                _limitPert += std::max(fp, (float) converter.getCtrlOnPwmMin() - (float) converter.getCtrlOnPwmCnt());
        } else if (_limitPert < 0) {
            // no faster than the limiters allow
            float back = std::min(-_limitPert, std::min(fpMax, limitingControlValue * kCtrlSlewLimit *
                                                                   (float) converter.pwmCtrlMax * dtCtrl));
            fp += back;
            _limitPert += back;
        }
        fp = constrain(fp, -(float) converter.getCtrlOnPwmCnt(), fpMax);
        // Same hold as updateManual(): while the forced-PWM gate is counting, the duty must stand
        // still or it outruns both the hold and the EWMA voltages the gate is judging it against.
        // Upward only - a limiter or the tracker must always be able to back off.
        if (fp > 0 && converter.forcedPwmGateArming()) fp = 0;
        const uint16_t d0 = converter.getCtrlOnPwmCnt();
        converter.pwmPerturbFractional(fp);
        if (slowEntry)
            ESP_LOGI("mppt", "Slow mode, duty %hu -> %hu (%+.2f, MPP duty %hu, P=%.2f)", d0, converter.getCtrlOnPwmCnt(),
                     fp, tracker.maxPowerPoint.dutyCycle, tracker._curPower);

        if (controlValue < -80 and fp < -0.01 and converter.getCtrlOnPwmCnt() > converter.getCtrlOnPwmMin()) {
            UART_LOG_ASYNC(
                "Limiting! ctrl %.2f => pert %.2f (to %hu) mode=%s idx=%i (act=%.3f tgt=%.3f)",
                controlValue, fp, converter.getCtrlOnPwmCnt(),
                MpptState2String[(int) controlMode], ctrlState.limIdx, limitingControl->actual,
                limitingControl->target);

            if (controlMode == MpptControlMode::CC)
                UART_LOG_ASYNC("Iout_max=%.2f powerLimit=%.2f", Iout_max, powerLimit);
        }
        rtcount("mppt.update.pwm");
    }
    lastUs = nowUs;

    // a shutdown in this tick (ctrl-nan, no-MPP) ends update() calls, so nothing would clear these again
    if (active()) {
        if (converter.syncRectEnabled_() != aboveThres)
            UART_LOG_ASYNC("Current %s threshold %.2f (pwm=%hu)", aboveThres ? "above" : "below", I_phys_smooth_min,
                           converter.getCtrlOnPwmCnt());
        bflow.enable((aboveThres || converter.boost() || g_app.psuMode()) && !(sensorPhysicalI->ewm.avg.get() < -0.05f && limits.reverse_current_paranoia));
        converter.enableSyncRect(aboveThres);
    }

    rtcount("mppt.update.en");

    if (ledPinSimple != 255) {
        bool ledState = (I_phys_smooth > 0.2f && controlMode == MpptControlMode::MPPT && controlValue > 0);
        digitalWrite(ledPinSimple, ledState);
        rtcount("mppt.update.led");
    }
}

void MpptController::pvAdvanceSetpoint(float iout, float vin, float dt) {
    if (std::isfinite(iout)) pvSim.iout.add(std::max(iout, 0.f));
    const float iF = pvSim.iout.get();
    float vTgt = std::isfinite(iF) ? pvSim.model.voltage(iF) : pvSim.model.voc;
    const float ceil_ = std::min(pvSim.model.voc, limits.Vout_max);
    // Boost floor: below Vin the topology can't regulate — truncate the curve. If Vin has
    // risen into the headroom of Voc the feasible interval is empty; hold the ceiling and
    // let the Voc-pinned feasibility latch own that fault.
    if (std::isfinite(vin)) vTgt = std::max(vTgt, vin + BoostHeadroomV);
    vTgt = std::min(vTgt, ceil_);
    // dt is stale-prone (early returns don't refresh lastUs); an unclamped step would both
    // jump the curve and kick the Vout PD's derivative.
    dt = std::min(dt > 0 ? dt : 6e-4f, 0.01f);
    const float dv = pvSim.slewVps * dt;
    psuVsetpoint += constrain(vTgt - psuVsetpoint, -dv, dv);
    publishedPsuSetpoint.store(psuVsetpoint, std::memory_order_release);
}

void MpptController::updateManual() {
    lastUs = wallClockUs();
    ctrlState.mode = MpptControlMode::None;

    const int16_t rampStep = std::max(1, (int) std::lround((float) converter.pwmMaxDriver() / 512.f));

    if (inBackoff() || charger.wireGuard.tripped() || (converter.disabled() && tempStartBlock())) {
        if (!converter.disabled()) converter.disable();
        return;
    }

    const uint16_t target = manualTarget.load(std::memory_order_relaxed);
    if (target == 0) {
        if (!converter.disabled()) {
            if (converter.getCtrlOnPwmCnt() <= converter.pwmCtrlMin)
                converter.disable();
            else
                converter.pwmPerturb(-rampStep);
        }
    } else {
        if (converter.disabled()) {
            if (minPulseStartBlocked(sensors.Vout->ewm.avg.get())) return;
            if (!limits.reverse_current_paranoia) {
                converter.enableSyncRect(true);
                bflow.enable(true);
            }
        }
        int16_t step = constrain((int32_t)target - (int32_t)converter.getCtrlOnPwmCnt(), -rampStep, rampStep);
        // Hold the duty while the forced-PWM gate counts out its hold - upward only, a retreat is
        // never blocked. The gate compares an instantaneous duty against EWMA-filtered voltages; if
        // the duty kept climbing through the wait (rampStep counts per ADC sample) it would both
        // outrun those filters and pile on current that the still-diode-emulating LS carries in its
        // body diode.
        if (step > 0 && converter.forcedPwmGateArming()) return;
        if (step) converter.pwmPerturb(step);
    }
}


void MpptController::begin(const ConfFile &trackerConf, const ConfFile &boardConf, const ConfFile &converterConf,
                           const Limits &limits_, const TeleConf &tele_) {
    limits = limits_;
    tele = tele_;

    pdLoadGains(converterConf, VinController, "vin");
    pdLoadGains(converterConf, VoutController, "vout");
    pdLoadGains(converterConf, IinController, "iin");
    pdLoadGains(converterConf, IoutCurrentController, "iout");
    pdLoadGains(converterConf, powerController, "power");

    float frac = trackerConf.getFloat("target_duty_cycle", 0.0f);
    if (std::isfinite(frac) && frac > 0.0f && frac <= 1.0f)
        targetPwmCnt = (uint16_t) std::round(frac * (float) converter.pwmMaxDriver());
    else
        targetPwmCnt = 0;

    if (targetPwmCnt) {
        g_app.opMode = OpMode::Manual;
        manualTarget.store(std::min(targetPwmCnt, converter.pwmCtrlMax),
                           std::memory_order_relaxed);
        ESP_LOGW("mppt", "target duty cycle PWM=%hu, manual mode (fixed duty), pwmMaxDriver=%u",
                 targetPwmCnt, (unsigned) converter.pwmMaxDriver());
    }

    auto mode = converterConf.getString("mode", "");
    bool psuBootQueued = false;
    if (targetPwmCnt && (mode == "psu" || mode == "pv")) {
        ESP_LOGE("mppt", "tracker.conf::target_duty_cycle overrides converter.conf mode=%s",
                 mode.c_str());
#if WITH_PSU
    } else if (mode == "psu") {
        float vout = converterConf.getFloat("psu_vout", 0.0f);
        if (!requestPsuSetpoint(vout, true)) {
            ESP_LOGE("mppt", "PSU mode but psu_vout invalid (%.2f), disabling", vout);
            g_app.setupErr = true;
        } else {
            ESP_LOGI("mppt", "PSU mode %.2fV queued until fresh telemetry", vout);
            psuBootQueued = true;
        }
    } else if (mode == "pv") {
        pvSim.slewVps = std::max(10.f, converterConf.getFloat("pv_slew", 200.0f));
        pvSim.iout.updateSpan(std::max(1.0f, converterConf.getFloat("pv_iout_span", 16.0f)));
        float isc = converterConf.getFloat("pv_isc", 0.0f);
        float voc = converterConf.getFloat("pv_voc", 0.0f);
        float k = converterConf.getFloat("pv_k", 0.8f);
        if (!queuePvCurve(isc, voc, k, true)) {
            ESP_LOGE("mppt", "PV mode but curve invalid (Isc=%.2f Voc=%.2f k=%.2f), disabling",
                     isc, voc, k);
            g_app.setupErr = true;
        } else {
            ESP_LOGI("mppt", "PV-sim Isc=%.2fA Voc=%.2fV k=%.2f queued until fresh telemetry",
                     isc, voc, k);
            psuBootQueued = true;
        }
#else
    } else if (mode == "psu" || mode == "pv") {
        ESP_LOGE("mppt", "converter.conf mode=%s needs CONFIG_FUGU_WITH_PSU, disabling", mode.c_str());
        g_app.setupErr = true;
#endif
    } else if (!mode.empty() && mode != "mppt" && !targetPwmCnt) {
        ESP_LOGE("mppt", "Unknown converter.conf mode '%s', disabling", mode.c_str());
        g_app.setupErr = true;
    }

    psuSlewDownVps = std::max(0.1f, converterConf.getFloat("psu_slew_down", 2.0f));
    sweepSpeed = std::max(0.1f, trackerConf.getFloat("sweep_speed", 4.0f));
    sweepVerify = std::max(0.f, trackerConf.getFloat("sweep_verify", 0.1f));
    scan.n = scan.saved = std::min<uint8_t>(6, trackerConf.getByte("pv_substrings", 0)); // learned, 0 = learn again
    scan.interval = (uint16_t) std::clamp(trackerConf.getLong("scan_interval", 300), 0L, 65535L);
    scan.trigger = trackerConf.getByte("scan_trigger", 1);

    if (tele.influxdbHost) {
        ESP_LOGI("main", "Influxdb telemetry to host %s", tele.influxdbHost.toString().c_str());
        // sampler.onNewSample = dcdcDataChanged;  // runs on the RT sampler task — would cross
        // tasks into teleBleEnqueue (single-task invariant, see tele_ble.cpp) if re-enabled
    }


    ledPinSimple = boardConf.getByte("led_simple", 255);
    if (ledPinSimple != 255) {
        pinMode(ledPinSimple, OUTPUT);
        digitalWrite(ledPinSimple, false);
    }

    fan.init(boardConf);

    bflow.init(boardConf);
    lvPgood.init(boardConf);
    meter.load();
    if (targetPwmCnt) {
        if (!limits.reverse_current_paranoia) {
            converter.enableSyncRect(true);
            bflow.enable(true);
        }
    } else if (psuBootQueued || g_app.psuMode()) {
        // At boot the queued Enable/EnablePv hasn't been applied by the RT consumer yet, so
        // psuMode() alone is false here — without psuBootQueued this would fall into the
        // sweep (and only calibrate as its side effect).
        sampler.startCalibration();
        ESP_LOGI("mppt", "PSU mode: calibration started, converter arm deferred to RT loop");
    } else {
        startSweep();
    }
}

void MpptController::telemetry() {
#if defined(WITH_NETW) || defined(WITH_BLE_TELE)
#ifdef WITH_NETW
    bool netReady = WiFi.isConnected() && tele.influxdbHost && timeSynced;
#else
    constexpr bool netReady = false;
#endif
#ifdef WITH_BLE_TELE
    bool bleReady = teleBleStreaming();
#else
    constexpr bool bleReady = false;
#endif
    if ((!netReady && !bleReady && !teleMqttActive()) || sampler.halted)
        return;

    if (wallClockUs() - _lastPointWrite < g_teleMinPointUs) {
        return;
    }

    auto I_phys_smooth = (sensorPhysicalI->ewm.avg.get());
    auto V_phys_smooth = (sensorPhysicalU->ewm.avg.get());
    //auto Vout(sensors.Vout->ewm.avg.get());
    float power_smooth = I_phys_smooth * V_phys_smooth;
    float power = sensorPhysicalI->med3.get() * sensorPhysicalU->med3.get();


    auto point = makeTelePoint("mppt");
    point.addTag("device", getHostname().c_str());
    point.addField("I", sensorPhysicalI->med3.get(), 3);
    point.addField("Ui", sensors.Vin->med3.get(), 2);
    point.addField("Uo", sensors.Vout->med3.get(), 2);
    //point.addField("U", V_phys_smooth, 2);
    point.addField("P", power, 2);
    if (withDebugFields)
        point.addField("P_smooth", power_smooth, 2);
    //point.addField("U_out", Vout, 2);

    point.addField("E", meter.totalEnergy.get(), 1);
    point.addField("E_today", meter.dailyEnergyMeter.today.energyYield, 1);

    const bool all = teleSparse();
    if (all || (_teleNumPoints % 20) == 0) {
        point.addField("pwm_dir_f", cntrlValue, 2);
        point.addField("mppt_state", int(ctrlState.mode));
        point.addField("P_max_today", meter.dailyEnergyMeter.today.pinMax, 1);
        const uint32_t nowS = (uint32_t) (wallClockUs() / 1000000);
        float ro = outZ.get(nowS);
        if (std::isfinite(ro))
            point.addField("Ro", ro * 1e3f, 2); // mΩ
        const float rc = charger.wire.own(nowS), rb = charger.wire.shared(nowS);
        if (std::isfinite(rc)) {
            point.addField("Rc", rc * 1e3f, 2); // mΩ
            point.addField("Rb", rb * 1e3f, 2);
        }
        const auto &wg = charger.wireGuard;
        if (wg.conf.mode) {
            point.addField("wf", (int) wg.wf());
            const float pw = wg.pw();
            if (std::isfinite(pw)) point.addField("Pw", pw, 2);
        }
    }

    if (all || (_teleNumPoints % 40) == 0) {
        point.addField("mcu_temp", ucTemp.lastFresh(), 1); // TODO to frequent
        point.addField("ntc_temp", ntc.lastFresh(), 1); // TODO to frequent
        point.addField("lag", (int) g_app.maxLoopLag); // peak RT-loop lag (µs), resets on reset-lag
        point.addField("rect_off", rectCal.offset()); // counts
        point.addField("heap", (int) heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        point.addField("heap_min", (int) heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
    }

    point.addField("pwm_duty", converter.getCtrlOnPwmCnt());
    if (g_app.psuMode() && (all || (_teleNumPoints % 10) == 0))
        point.addField("Vset", getPsuSetpoint(), 2); // moving in PV-sim, constant in plain PSU
    if (!converter.disabled() && (all || (_teleNumPoints % 10) == 0)) {
        point.addField("pwm_ls_duty", converter.getRectOnPwmCnt());
        point.addField("pwm_ls_max", converter.getRectOnPwmMax());
        point.addField("pwm_dcm", converter.inDCM());
    }


    if (ctrlState.mode == MpptControlMode::MPPT) {
        if (withDebugFields) {
            auto dP = tracker.dP;
            point.addField("P_filt", tracker._curPower, 2);
            point.addField("P_prev", tracker._lastPower, 2);
            point.addField("dP", dP, 2);

            //point.addField("P_filt", tracker.pwmPowerTable[buck.getBuckDutyCycle()].get(), 1);
            //point.addField("P_filt", tracker._powerBuf.getMean(), 1);
            if (std::abs(dP) < tracker.minPowerStep) {
                point.addField("dP_thres", 0.0f, 2);
            } else {
                point.addField("dP_thres", dP, 2);
            }
        }
    }

    if (!limIdxSampled.empty()) {
        point.addField("cv_lim_idx", limIdxSampled.pop());
    }

    point.setTimeMs();

    telemetryAddPoint(point, 80);
    _lastPointWrite = wallClockUs();
    _teleNumPoints++;
#endif
}

extern SynchronousConverter converter;
extern MpptController mppt;

// One rewrite of coil.conf (littlefs keeps the old file until close), under the admitted persist lock.
bool rectCalPersist(int counts, uint8_t cal) {
    char ns[16], cs[4];
    snprintf(ns, sizeof(ns), "%.0f", rectOffsetNsFromCounts(counts, converter.getPwmTickRate()));
    snprintf(cs, sizeof(cs), "%u", cal);
    ConfFile conf{"/littlefs/conf/coil.conf"};
    const bool ok = conf.update({{"rect_offset_ns", ns}, {"rect_offset_drv", converter.drvName()},
                                 {"rect_offset_cal", cs}}, "rect_offset"); // legacy count key
    if (!ok) ESP_LOGW("rectcal", "coil.conf write failed, retrying");
    return ok;
}

// RT: a sweep whose landing power misses its MPP saw the light change; it does not learn, the scan or one re-sweep per
// 10 min checks the other peaks. !ok: not evaluable (limiter, fade stopped), no verdict
__attribute__((cold, noinline)) void MpptController::sweepVerdict(uint32_t nowMs, float p, bool ok) {
    _svPhase = 0;
    const float mpp = maxPowerPoint.power, d = mpp > 0 ? p / mpp - 1 : 0;
    if (!ok) {
        ESP_LOGI("mppt", "sweep not verified (limiter)");
        return;
    }
    _svLand = p, _svFlag = std::fabs(d) > sweepVerify ? 2 : 1;
    if (_svFlag == 1) {
        scan.release();
        ESP_LOGI("mppt", "sweep verified: landed %.0f W vs MPP %.0f W (%+.0f %%)", p, mpp, 100 * d);
        return;
    }
    scan.held = false;
    const bool sc = scan.n && scan.voc > 0;
    ESP_LOGI("mppt", "sweep flagged: landed %.0f W vs MPP %.0f W (%+.0f %%)%s", p, mpp, 100 * d, sc ? ", scan now" : "");
    if (sc) scan.force = true;
    else scheduleResweep(nowMs);
}

__attribute__((cold, noinline)) void MpptController::scheduleResweep(uint32_t nowMs) {
    if (_retryAt && nowMs - _retryAt < 600000) {
        ESP_LOGI("mppt", "re-sweep capped (one per 10 min)");
        return;
    }
    _retryAt = nowMs | 1, _resweepAt = (nowMs + 60000) | 1;
    ESP_LOGI("mppt", "re-sweep in 60 s");
}

// core 0: learn from the last sweep; save a grown count once a flash write is safe
void MpptController::scanLf(bool flashSafe) {
    scan.learn();
    if (flashSafe) scanSave();
}

// core 0: one tracker.conf rewrite when the learned count grew; a value set by hand since then is kept
void MpptController::scanSave() {
    if (!scan.unsaved()) return;
    ConfFile conf{"/littlefs/conf/tracker.conf"};
    if (conf.getByte("pv_substrings", 0) != scan.saved) {
        scan.saved = scan.n;
        return;
    }
    char v[4];
    snprintf(v, sizeof v, "%u", scan.n);
    if (conf.update({{"pv_substrings", v}})) scan.saved = scan.n;
    else ESP_LOGW("mppt", "tracker.conf write failed, retrying");
}

void MpptController::rectCalLf() {
    if (!sensorPhysicalI) return;
    const auto nowS = (uint32_t) (wallClockUs() / 1000000);
    rectCal.lf(converter, nowS, sensorPhysicalI->ewm.avg.get() * sensorPhysicalU->ewm.avg.get(),
               sensors.Vin->ewm.avg.get() / sensors.Vout->ewm.avg.get());
    const auto t0 = (uint32_t) wallClockUs();
    const bool was = rectCal.unsaved();
    rectCal.saveNow(converter, nowS, [] { vTaskDelay(pdMS_TO_TICKS(2)); });
    if (was && !rectCal.unsaved())
        ESP_LOGI("rectcal", "save took %lu us", (unsigned long) ((uint32_t) wallClockUs() - t0));
}
