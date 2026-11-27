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
                         ? (pvSim.active ? min(limits.Iout_max, pvSim.model.isc * pvSim.shade.iscScale() * 1.1f)
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
    // from the last attempt: a failing calibration must not re-sweep right away
    const auto sinceSweepUs = nowUs - sampler.getCalibrationStartUs();
    const uint32_t nowMs = (uint32_t) (nowUs / 1000);
    const bool grew = maxPowerPoint.power >= sweepMinPower() && maxPowerPoint.power < 25 * sweepMinPower() &&
                      power_smooth > 2 * maxPowerPoint.power && sinceSweepUs > 120 * 1000000;
    if (_capV > 0) capResweep(nowMs);
    bool retry = _resweepAt && (int32_t) (nowMs - _resweepAt) >= 0;
    // landed brighter than the MPP: re-sweep once the light holds (input current up < 3 % in 5 s), 60 s at the latest
    if (retry && _svFlag == 2 && _svLand > maxPowerPoint.power && nowMs - _retryAt < 55000) {
        const float i = power_smooth / sensors.Vin->ewm.avg.get();
        if (i > 1.03f * _svI) _svI = i, _resweepAt = (nowMs + 5000) | 1, retry = false;
    }
    if (!g_app.psuMode() && !_sweeping && !batteryFull && !inBackoff() && !flashOpActive() &&
        (grew || retry || sinceSweepUs > (time_us) sweepInterval * 1000000)) {
        ESP_LOGI("mppt", "%s sweep & sensor calibration", grew ? "power-doubled" : retry ? "retry" : "periodic");
        g_app.maxLoopLag = 0; // restart the lag window so telemetry tracks per-sweep peak, not all-time
        if (retry) _resweepAt = 0;
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
    float limitingControlValue = std::numeric_limits<float>::infinity(), outerCv = limitingControlValue;
    float powerCv = limitingControlValue;

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

        if (&c == &controlValues.back()) powerCv = std::fmin(cv, powerCv);
        else if (&c != controlValues.data() && cv < outerCv) outerCv = cv;
        if (cv < limitingControlValue) {
            limitingControlValue = cv;
            limitingControl = &c;
        }
    }

    //auto limitingControl = std::min_element(controlValues.begin(), controlValues.end(),
    //                                        [](const CVP &a, const CVP &b) { return a.second < b.second; });

    MpptControlMode controlMode = MpptControlMode::None;
    float controlValue = 0, capCv = limitingControlValue;
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
            _paceV = sensors.Vin->ewm.avg.get(), _paceVo = sensors.Vout->ewm.avg.get();
            _paceD = converter.getCtrlOnPwmCnt(), _pace = 1, _paceS = 0, _paceCap = 0, _paceT = (uint32_t) nowUs;
            // the filters lag the ramp: it moves at most 1/64 of the duty range per filter delay
            _lagF = sweepFiltDelay();
            _paceMax = std::clamp(1 / (64 * sweepSpeed * kCtrlSlewLimit * _lagF), 1.f, sweepPace);
        }

        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::Sweep;
            // sweep_speed scales both the per-tick cap and the limit-tracking gain,
            // so it actually controls speed near P/I/V limits, not just far from them.
            // The Vin floor does not slow it: crossing the floor ends the sweep. The pace speed-up leaves the
            // limit-tracking terms as they are, except the power limiter's (slow everywhere), up to half its limit.
            const float k = sweepSpeed * (0.25f / 5.0f), pp = powerCv;
            const float pace = 1 + (_pace - 1) * std::clamp(pp * (2 * 0.25f / 5.0f) - 1, 0.f, 1.f);
            controlValue = std::min(std::min(sweepSpeed, pp * k) * pace, outerCv * k);
            capCv = std::min(outerCv, pp * pace);
            sweepPaceUpdate(power_smooth);

            // capture MPP during sweep, this will be our target afterward, without the input caps' discharge (the
            // Vin EWM is as delayed as the power). Ignore sub-sweepMinPower() samples so a marginal-light sweep can't
            // "peak" at a near-max-duty phantom (dawn cold-start) and strand the converter there.
            const float pPv = power_smooth - _paceCap;
            scan.sweepSample(sensors.Vin->ewm.avg.get(), pPv);
            if (pPv > maxPowerPoint.power && pPv >= sweepMinPower()) {
                maxPowerPoint.power = pPv;
                maxPowerPoint.dutyCycle = converter.getCtrlOnPwmCnt();
                maxPowerPoint.voltage = sensors.Vin->ewm.avg.get();
                maxPowerPoint.slope = _paceS;
            }

            sweepPlot.add(sensors.Vin->ewm.avg.get(), converter.getDutyCycle(), pPv, limits.Vin_max);

            // ascending ramp only: the move back to the MPP runs after _stopSweep
            if (!converter.disabled() && !converter.boost() && !g_app.psuMode() && !sensors.Iout->isVirtual &&
                bflow.state())
                outZ.add(sensors.Vout->med3.get(), sensors.Iout->med3.get(), (uint32_t) nowUs);
            rtcount("mppt.update.sweeping");
        } else {
            // the limiter only when one is active this tick (the smallest response is always set)
            CVP *lim = limitingControlValue < 0 ? limitingControl : nullptr;
            _stopSweep(controlMode, lim ? int(lim - controlValues.begin()) : -1, lim);
            rtcount("mppt.update.stopSweep");
            if (converter.disabled()) { // no-MPP backoff
                ctrlState.mode = MpptControlMode::None;
                return;
            }
        }
    } else if (_svPhase == 3) {
        // return pass: steered like a scan; a limiter ends it unevaluated, as it stops a fade (CV only while it
        // lowers the duty: the sweep ends at a limit)
        const uint16_t d = converter.getCtrlOnPwmCnt();
        float t = NAN;
        if (controlMode == MpptControlMode::None || controlMode == MpptControlMode::MPPT || controlMode == CV)
            t = scan.update(nowMs, sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get(), power, d, 0, 0,
                            [this] { return powerSettleMs(); }, converter.lsFading() && !converter.forcedPwm_());
        if (controlMode != MpptControlMode::None && controlMode != MpptControlMode::MPPT && !(t < (float) d))
            t = NAN, scan.abort(), scan.ended = 0;
        if (!std::isnan(t)) {
            controlMode = MpptControlMode::Sweep;
            controlValue = constrain(t - (float) d, -(float) (converter.pwmCounts() / 64) - 1,
                                     (float) (converter.pwmCounts() / 128) + 1);
        } else if (scan.ended == 6) {
            scan.ended = 0, _svPhase = 0;
            tracker.resetDirection(false, sensors.Vin->ewm.avg.get()), tracker.resume(scan.rLand);
            const float dl = scan.rLand / scan.rBest - 1;
            ESP_LOGI("mppt", "sweep return: landed %.1f V %.1f W, best probe %.1f W", sensors.Vin->ewm.avg.get(),
                     scan.rLand, scan.rBest);
            // the probes ranked the maxima afresh; a scan only if the light changed during the return pass too
            if (_svLandTrim & 2)
                sweepJudge(scan.rLand, std::fabs(dl) > std::fabs(scan.rDev) ? dl : scan.rDev, std::fabs(dl) > sweepVerify);
        } else {
            if (scan.ended == 4) targetDutyCycle = scan.d0;
            scan.ended = 0;
            sweepVerdict(nowMs, 0, false);
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
    // landing assessment holds the duty; a limiter taking over ends it unevaluated
    if (_svPhase == 2 && ctrlState._limiting) sweepVerdict(nowMs, 0, false);
    if (_svPhase == 2 && nowMs - _svT > std::clamp(powerSettleMs(), 1500.f, 3000.f))
        sweepVerdict(nowMs, power_smooth, !batteryFull && !ctrlState._limiting && !converter.disabled() && !g_app.psuMode());


    float currentThreshold = limits.reverse_current_paranoia
                                 ? (bflow.state() ? 0.05f : 0.2f)
                                 : (bflow.state() ? 0.0f : 0.1f); // hysteresis; // hysteresis
    float I_phys_smooth_min = I_phys_smooth; //std::min(I_phys_smooth, sensorPhysicalI->med3.get());
    bool aboveThres = (I_phys_smooth_min > currentThreshold
                       || (I_phys_smooth_min > 0.05 && converter.getDutyCycle() > 0.3f)
    );

    if (controlMode == MpptControlMode::None && !_sweeping && !targetDutyCycle && !_svPhase && !g_app.psuMode() &&
        !batteryFull && !converter.boost()) {
        const uint16_t d = converter.getCtrlOnPwmCnt();
        const float t = scan.update(nowMs, sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get(), power, d,
                                    1.05f * vinFloor(true), // a start runs from flash
                                    flashOpActive() ? std::numeric_limits<float>::max() : 25 * sweepMinPower(),
                                    [this] { return powerSettleMs(); },
                                    converter.lsFading() && !converter.forcedPwm_());
        if (!std::isnan(t)) {
            // the tracker pauses; back at the start point it carries on, on another peak it restarts
            controlMode = MpptControlMode::Sweep, keepTracker = true;
            controlValue = constrain(t - (float) d, -(float) (converter.pwmCounts() / 64) - 1,
                                     (float) (converter.pwmCounts() / 128) + 1);
        } else if (scan.ended) {
            if (scan.ended == 1 || scan.ended == 3) tracker.resume(scan.pRef);
            else tracker.resetDirection(false, sensors.Vin->ewm.avg.get()), tracker.maxPowerPoint.power = 0; // probed below the peak
            if (scan.ended == 4) targetDutyCycle = scan.d0;
            scan.ended = 0;
        }
    } else if (scan.active() && !_sweeping && _svPhase != 3) {
        scan.abort();
        if (!targetDutyCycle && !g_app.psuMode()) targetDutyCycle = scan.d0;
    }

    if (controlMode == MpptControlMode::None) {
        if (g_app.psuMode()) {
            controlMode = MpptControlMode::CV;
            controlValue = limitingControlValue;
        } else if (_svPhase == 2) {
            controlMode = MpptControlMode::MPPT;
            controlValue = 0;
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
    controlValue = std::min(controlValue, capCv);
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
    float vTgt = std::isfinite(iF) ? pvSim.shade.voltage(pvSim.model, iF) : pvSim.model.voc;
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

    const bool hold = inBackoff() || charger.wireGuard.tripped() || charger.params.awaitingDetect() ||
                      (converter.disabled() && tempStartBlock());
    if (hold && !converter.disabled()) converter.disable();
    const float mcZ = mcBfZero.load(std::memory_order_acquire);
    if (mcZ == mcZ && !converter.boost()) { // boost: the task closes BF (battery feeds the coil)
        if (converter.disabled()) {
            if (bflow.state()) bflow.enable(false);
        } else if (!bflow.state()) {
            // lower bound of protect()'s reverse-current filter: fails whenever that opened BF (and when cancelled)
            const auto &s = *sensorPhysicalI;
            if (fminf(s.ewm.avg.get(), std::min(s.last, s.previous)) - mcZ > 0.1f) {
                bflow.enable(true);
                if (bflow.state() && mcSyncRect.load(std::memory_order_relaxed)) converter.enableSyncRect(true, true);
            }
        }
    }
    if (hold) return;

    if (bfTest.mode.load(std::memory_order_relaxed)) bfTestRt();
    const uint16_t target = mcZ == INFINITY ? 0 : manualTarget.load(std::memory_order_relaxed);
    if (target == 0) {
        if (!converter.disabled()) {
            if (converter.getCtrlOnPwmCnt() <= converter.pwmCtrlMin) {
                converter.disable();
                if (mcZ == mcZ && !converter.boost()) bflow.enable(false);
            } else
                converter.pwmPerturb(-rampStep);
        }
    } else {
        if (converter.disabled()) {
            if (bfTest.mode.load(std::memory_order_relaxed) & BfStop) return;
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
    bfBat = boardConf.getByte("bflow_bat", 0);
    hsClampConf = boardConf.getByte("hs_clamp", 0);

    pdLoadGains(converterConf, VinController, "vin");
    pdLoadGains(converterConf, VoutController, "vout");
    pdLoadGains(converterConf, IinController, "iin");
    pdLoadGains(converterConf, IoutCurrentController, "iout");
    pdLoadGains(converterConf, powerController, "power");

    // tracker.conf::target_duty_cycle boots into manual PWM at that fixed duty (0-1 of
    // pwmMaxDriver). An EXPLICIT 0 boots into manual PWM at duty 0 -- converter off, as after
    // `dc 0` -- so a bench board stays still on power-up; only an ABSENT key boots into MPPT.
    // (Before, 0 read as "unset" and the board swept on every power-up.)
    const float frac = trackerConf.getFloat("target_duty_cycle", NAN);
    const bool fixedDuty = std::isfinite(frac) && frac >= 0.0f && frac <= 1.0f;
    targetPwmCnt = fixedDuty ? (uint16_t) std::round(frac * (float) converter.pwmMaxDriver()) : 0;
    const bool bootManualZero = fixedDuty && !targetPwmCnt;
    if (!fixedDuty && !std::isnan(frac))
        ESP_LOGE("mppt", "tracker.conf::target_duty_cycle=%.3f outside [0,1], ignored (MPPT)", frac);

    if (targetPwmCnt) {
        g_app.opMode = OpMode::Manual;
        manualTarget.store(std::min(targetPwmCnt, converter.pwmCtrlMax),
                           std::memory_order_relaxed);
        ESP_LOGW("mppt", "target duty cycle PWM=%hu, manual mode (fixed duty), pwmMaxDriver=%u",
                 targetPwmCnt, (unsigned) converter.pwmMaxDriver());
    } else if (bootManualZero) {
        g_app.opMode = OpMode::Manual;
        manualTarget.store(0, std::memory_order_relaxed);
        ESP_LOGW("mppt", "target_duty_cycle=0: manual mode at duty 0 (converter off, no sweep)");
    }

    auto mode = converterConf.getString("mode", "");
    bool psuBootQueued = false;
    if ((targetPwmCnt || bootManualZero) && (mode == "psu" || mode == "pv")) {
        ESP_LOGE("mppt", "tracker.conf::target_duty_cycle overrides converter.conf mode=%s",
                 mode.c_str());
#if WITH_PSU
    } else if (mode == "psu") {
        float vout = converterConf.getFloat("psu_vout", 0.0f);
        if (!requestPsuSetpoint(vout, true)) {
            ESP_LOGE("mppt", "PSU mode but psu_vout invalid (%.2f), disabling", vout);
            g_app.setupFail("converter.conf psu_vout invalid");
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
        PvShade sh;
        if (!sh.parse(converterConf.getString("pv_shade", "").c_str())) {
            ESP_LOGE("mppt", "converter.conf pv_shade invalid, disabling");
            g_app.setupFail("converter.conf pv_shade invalid");
        } else if (!queuePvCurve(isc, voc, k, true, true, sh)) {
            ESP_LOGE("mppt", "PV mode but curve invalid (Isc=%.2f Voc=%.2f k=%.2f), disabling",
                     isc, voc, k);
            g_app.setupFail("converter.conf pv curve invalid");
        } else {
            ESP_LOGI("mppt", "PV-sim Isc=%.2fA Voc=%.2fV k=%.2f queued until fresh telemetry",
                     isc, voc, k);
            psuBootQueued = true;
        }
#else
    } else if (mode == "psu" || mode == "pv") {
        ESP_LOGE("mppt", "converter.conf mode=%s needs CONFIG_FUGU_WITH_PSU, disabling", mode.c_str());
        g_app.setupFail("converter.conf mode=%s needs WITH_PSU", mode.c_str());
#endif
    } else if (!mode.empty() && mode != "mppt" && !targetPwmCnt && !bootManualZero) {
        ESP_LOGE("mppt", "Unknown converter.conf mode '%s', disabling", mode.c_str());
        g_app.setupFail("converter.conf mode '%s' unknown", mode.c_str());
    }

    psuSlewDownVps = std::max(0.1f, converterConf.getFloat("psu_slew_down", 2.0f));
    psuSlewUpVps = std::max(0.f, converterConf.getFloat("psu_slew_up", converter.boost() ? 5.0f : 0.f));
    cIn = converterConf.getFloat("c_in", 1e-3f);
    if (!(cIn >= 0 && cIn <= 20e-3f)) {
        ESP_LOGE("mppt", "converter.conf::c_in %g F invalid (0..0.02), using 1e-3", (double) cIn);
        cIn = 1e-3f;
    }
    sweepSpeed = std::max(0.1f, trackerConf.getFloat("sweep_speed", 4.0f));
    sweepVerify = std::max(0.f, trackerConf.getFloat("sweep_verify", 0.1f));
    sweepPace = std::clamp(trackerConf.getFloat("sweep_pace", 4.0f), 1.f, 8.f);
    sweepInterval = (uint16_t) std::clamp(trackerConf.getLong("sweep_interval", 1800), 60L, 65535L);
    scan.n = scan.saved = std::min<uint8_t>(6, trackerConf.getByte("pv_substrings", 0)); // learned, 0 = learn again
    scan.interval = (uint16_t) std::clamp(trackerConf.getLong("scan_interval", 300), 0L, 65535L);
    scan.trigger = trackerConf.getByte("scan_trigger", 1);
    scan.nearDv = (uint8_t) std::clamp(std::lround(10 * trackerConf.getFloat("scan_near_v", 2.f)), 0L, 255L);

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
    // a boost's output follows Vin with no pack; a backflow switch may sit between the Vout sense and the pack
    assert_throw(!charger.params.vbatAuto || (!converter.boost() && (!bflow || bfBat)),
                 "vout_max=auto needs a buck with Vout sensed at the pack: no backflow switch, or bflow_bat=1");
    if ((bfBat || hsClampConf) && !hsIdleCheck())
        ESP_LOGW("mppt", "board.conf bflow_bat/hs_clamp ignored: needs a backflow switch, a buck and Iout sensed");
    else if (hsClampConf && !batIBipolar())
        ESP_LOGW("mppt", "board.conf hs_clamp ignored: the Iout sensor cannot read reverse current");
    lvPgood.init(boardConf);
    meter.load();
    if (targetPwmCnt) {
        if (!limits.reverse_current_paranoia) {
            converter.enableSyncRect(true);
            bflow.enable(true);
        }
    } else if (bootManualZero) {
        // Nothing switches, so this is the zero-current calibration the sweep would have run.
        // Sync rect and back-flow stay off until a `dc N` / `sync` asks for them.
        sampler.startCalibration();
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
        if (const uint8_t hs = hsFault.load(std::memory_order_relaxed)) point.addField("hs_short", (int) hs);
        if (bfFail.load(std::memory_order_relaxed)) point.addField("bf_fail", 1);
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

void MpptController::hsShortRt(uint32_t nowMs) {
    if (hsClearReq.exchange(false, std::memory_order_acq_rel)) hsClearRt(); // also a latch restored on any topology
    if (unlikely(g_hwOvTrip)) { // main.cpp keeps the alert while the clamp is pending, ramping or held
        if (const uint8_t f = hsFault.load(std::memory_order_relaxed); f && f != HsOff)
            hsGatesOffRt("OV alert");
    }
    if (converter.boost()) return;
    auto &g = hsGuard;
    if (unlikely(bfTest.mode.load(std::memory_order_relaxed) && !g_app.manualPwm())) RT_COLD(
        // the mode changed under the self-test: drop its exemptions and pinned LS
        bfTest.mode.store(0);
        converter.setManualRect(-1);
        bflow.lockOff.fetch_and((uint8_t) ~BackflowDriver::LockSelftest);
    );

    const bool idle = hsIdleCheck();
    if (idle) {
        const Sensor &s = *sensorPhysicalI;
        const auto &cc = s.params.calibrationConstraints;
        const float ofs = cc.calibrateOffset && s.calibrationComplete ? s.calibrationAvg : 0.f;
        const float iMin = std::max(.3f, .2f * cc.maxAbsValue), iRaw = s.ewm.avg.get() + ofs;
        if (g.update(nowMs, converter.disabled(), sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), iRaw, iMin,
                     std::max(2.f * iMin, 2.f * cc.maxAbsValue), g_app.psuMode() ? NAN : computeOvThreshold(),
                     !bflow.state()))
            hsLatchRt(nowMs, g.current(iRaw), g.byVoltage ? 'V' : 'I');
        const uint8_t f = hsFault.load(std::memory_order_relaxed);
        const float i = g.current(s.med3.get() + ofs);
        if (f == HsClamped || f == HsRamp) {
            // the clamp parks the switch node at ground: battery current through it means a conducting BF
            if (std::isfinite(i)) hsRise = i >= -.5f && i <= std::max(hsI0, 0.f) + .5f ? 0 : hsRise + 1; // NaN holds
            if (hsRise >= 2) RT_COLD(
                converter.clampLs(false);
                bflow.enable(false);
                hsFault.store(HsOff);
                hsClampUnsafe.store(true); // persisted by core 0, until a fresh blocks pass
                ESP_LOGE("hs-short", "I %.2f A under the LS %s (%.2f A before): released, all gates off", i,
                         f == HsRamp ? "ramp" : "clamp", hsI0);
            );
            else if (f == HsRamp) hsRampRt(nowMs);
            else if (nowMs - hsClampMs >= 250) {
                hsClampMs = nowMs;
                if (hsClampAllowed(hsWhy.load(std::memory_order_relaxed) == 'V')) converter.clampLs(true); // after a force-off elsewhere
                else hsGatesOffRt("clamp no longer armed");
            }
        } else if (f == HsPending) hsPendingRt(nowMs, i);
    }
    const bool inh = idle ? g.blocks() : g.latched;
    if (inh != converter.startInhibited(SynchronousConverter::InhibitHs))
        converter.inhibitStart(inh, SynchronousConverter::InhibitHs);
}

__attribute__((cold, noinline)) void MpptController::hsClearRt() {
    auto &g = hsGuard;
    if (converter.lsClamped() || hsFault.load(std::memory_order_relaxed) == HsRamp) converter.clampLs(false);
    bflow.lockOff.fetch_and((uint8_t) ~BackflowDriver::LockHsShort);
    g.latched = g.suspect = g.clean = false, g.needCheck = true;
    hsWhy.store(0);
    if (converter.boost()) converter.inhibitStart(false, SynchronousConverter::InhibitHs); // nothing else syncs it
    hsFault.store(0);
    ESP_LOGW("hs-short", "cleared%s", hsIdleCheck() ? ", start blocked until checked again" : "");
}

// The clamp is only safe for the LS at night: by day the input caps dump through the shorted HS and the LS.
__attribute__((cold, noinline)) void MpptController::hsLatchRt(uint32_t nowMs, float i, char why) {
    hsGuard.latched = true;
    bflow.lockOff.fetch_or(BackflowDriver::LockHsShort);
    bflow.enable(false);
    converter.inhibitStart(true, SynchronousConverter::InhibitHs);
    const bool clamp = hsClampAllowed(why == 'V');
    hsI0 = i, hsClampMs = nowMs, hsRise = 0, hsArm.low = false;
    hsWhy.store(why);
    hsFault.store(clamp ? HsPending : HsOff);
    converter.disable();
    ESP_LOGE("hs-short", "HS short (%c): %.2f A, Vin %.2f Vout %.2f; start blocked, all gates off%s", why, i,
             sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), clamp ? ", clamp pending Vin<9V" : "");
}

__attribute__((cold, noinline)) void MpptController::hsGatesOffRt(const char *why) {
    converter.clampLs(false);
    hsFault.store(HsOff);
    ESP_LOGW("hs-short", "%s: all gates off", why);
}

// RT, each sample while the clamp is pending: arms on Vin below VinMax for HoldMs, re-checks the arming every 250 ms
void MpptController::hsPendingRt(uint32_t nowMs, float i) {
    const bool low = hsArm.update(nowMs, sensors.Vin->ewm.avg.get());
    if (!low && nowMs - hsClampMs < 250) return;
    hsClampMs = nowMs;
    if (!hsClampAllowed(hsWhy.load(std::memory_order_relaxed) == 'V')) return hsGatesOffRt("clamp no longer armed");
    if (!low || std::isnan(i)) return;
    RT_COLD(
        if (i < -.5f) { // battery current into the short: the BF conducts, a clamp would short the battery
            hsClampUnsafe.store(true);
            ESP_LOGE("hs-short", "I %.2f A before the LS ramp: no clamp", i);
            return hsGatesOffRt("BF conducts");
        }
        hsI0 = i, hsRise = 0;
        hsRampUs = (uint32_t) wallClockUs(), hsRampPos = 0;
        hsFault.store(HsRamp);
        hsRampCnt = converter.lsClamps(); // before the pulse: a disable() during it ends the ramp
        if (!converter.lsPulse((uint32_t) std::ceil(HsClampArm::StartNs * 1e-9f * converter.getPwmTickRate()))
            || converter.lsClamps() != hsRampCnt)
            return hsGatesOffRt("LS ramp refused");
        ESP_LOGW("hs-short", "Vin %.2f V below %.0f V for %lu s: LS ramp", sensors.Vin->ewm.avg.get(),
                 HsClampArm::VinMax, (unsigned long) (HsClampArm::HoldMs / 1000));
    );
}

// RT, each sample of the LS ramp: a disable() (a protection) or an OV alert ends it with all gates off
void MpptController::hsRampRt(uint32_t nowMs) {
    if (converter.lsClamps() != hsRampCnt || g_hwOvTrip || !hsClampAllowed(hsWhy.load(std::memory_order_relaxed) == 'V'))
        return hsGatesOffRt("LS ramp stopped");
    // a stalled loop must not skip the ramp: one step advances it by at most StepMaxUs
    const auto now = (uint32_t) wallClockUs();
    hsRampPos += std::min(now - hsRampUs, HsClampArm::StepMaxUs), hsRampUs = now;
    if (hsRampPos < HsClampArm::RampUs) {
        const float rate = converter.getPwmTickRate();
        converter.lsPulse(HsClampArm::rampTicks(hsRampPos, (uint32_t) std::ceil(HsClampArm::StartNs * 1e-9f * rate),
                                                converter.pwmMaxDriver()));
        // a disable() (core 0) between the check above and the step: undo the step
        if (converter.lsClamps() != hsRampCnt) hsGatesOffRt("LS ramp stopped");
        return;
    }
    RT_COLD(
        hsClampMs = nowMs;
        hsFault.store(HsClamped);
        converter.clampLs(true);
        ESP_LOGW("hs-short", "LS clamped on");
    );
}

// RT, manual mode, each new sample
void MpptController::bfTestRt() {
    const uint8_t m = bfTest.mode.load(std::memory_order_acquire);
    if (converter.disabled()) return;
    if (m & BfStop) return bfTestStopRt();
    if (m & BfToggle) return bfToggleRt(m);
    if (m & BfConducts) bflow.enable(true);
    const float i = sensorPhysicalI->med3.get();
    if (!(m & BfEngage)) {
        if ((m & BfConducts) && !(i <= bfTest.iMax)) RT_COLD( // the CCM search's envelope; NaN stops too
            bfTest.why = std::isnan(i) ? 0 : 5, bfTest.i = i;
            bfTestStopRt();
        );
        return;
    }
    converter.setManualRect(0x7fff); // complementary LS, follows the HS count
    if (i < bfTest.iMin) bfTest.iMin = i;
    if (!(i >= ((m & BfBlocks) ? -.3f : -.5f))) { // NaN stops without a record
        if (!std::isnan(i)) {
            bfTest.i = i, bfTest.vin = sensors.Vin->med3.get(), bfTest.vout = sensors.Vout->med3.get();
            bfTest.d = converter.dutyCtrlEff();
            bfTest.tripped.store(true, std::memory_order_release);
        }
        bfTestStopRt();
    }
}

// the toggle variant's switch and its guards; the current sensor may not see reverse current
void MpptController::bfToggleRt(uint8_t m) {
    const bool on = m & BfOn;
    if (on != bflow.state()) {
        bflow.enable(on);
        bfTest.floorHold = bfTest.holdN, bfTest.iSeen = (uint8_t) sensorPhysicalI->numSamples;
    }
    if (!(m & BfEngage)) {
        if (!(m & BfNoBat)) return;
        converter.setManualRect(0); // LS at its minimum: an asynchronous buck, no reverse path
    } else converter.setManualRect(0x7fff);
    const float i = sensorPhysicalI->med3.get(), d = converter.dutyCtrlEff(),
                y = d * sensors.Vin->med3.get() - sensors.Vout->med3.get();
    uint8_t why = 0;
    if (m & BfNoBat) { // raw samples, two in a row
        const auto &vi = *sensors.Vin, &vo = *sensors.Vout;
        why = !(i >= -.5f && i <= .5f) ? 3
              : (m & BfEngage) && on && !(d * vi.last - vo.last >= -1.f) && !(d * vi.previous - vo.previous >= -1.f) ? 2
              : !(vi.last <= bfTest.vinMax) && !(vi.previous <= bfTest.vinMax) ? 4 : 0;
        // the output follows: forced PWM before the diode mode pumps it further (the switch is off)
        if (!why && !(m & BfEngage) && vo.last >= bfTest.vProof && vo.previous >= bfTest.vProof)
            bfTest.mode.fetch_or(BfEngage, std::memory_order_relaxed);
    } else if (!(m & BfFloor)) return;
    else if (!(y >= -.3f)) why = 2;
    else if (on) {
        if (const auto n = (uint8_t) sensorPhysicalI->numSamples; n != bfTest.iSeen && bfTest.floorHold)
            bfTest.iSeen = n, --bfTest.floorHold;
        if (!bfTest.floorHold && !(i >= bfTest.iFloor)) why = 1;
    }
    if (!why) return;
    RT_COLD(
        bfTest.why = why, bfTest.i = i, bfTest.d = y;
        bfTest.tripped.store(true, std::memory_order_release);
        bfTestStopRt();
    );
}

void MpptController::bfTestStopRt() {
    bfTest.mode.fetch_or(BfStop);
    converter.disable();
    setManualTarget(0);
    converter.setManualRect(-1);
    bflow.enable(false);
}

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

// RT: a sweep whose landing power misses its MPP saw the light change; it does not learn. One maximum and no substring
// count: one re-sweep per 10 min. !ok: not evaluable (limiter, fade stopped), no verdict
// First the landing trim: Vin lags the ramp (C_in V^2 / 2P, ~0.3 s in dim light), so the captured duty
// lies past the MPP, the captured Vin (filtered like the power) does not. Half a step on the sweep's Vin slope onto it.
__attribute__((cold, noinline)) void MpptController::sweepVerdict(uint32_t nowMs, float p, bool ok) {
    _svPhase = 0;
    const float v = sensors.Vin->ewm.avg.get(), dv = v - maxPowerPoint.voltage;
    const float mpp = maxPowerPoint.power, d = mpp > 0 ? p / mpp - 1 : 0;
    if (ok) ESP_LOGI("mppt", "sweep landed %.1f V %.1f W, MPP %.1f V %.1f W", v, p, maxPowerPoint.voltage, mpp);
    // a verdict sweep is bright: its lag is short, a landing off by more than sweep_verify is the light
    if (ok && (_svLandTrim & 1) && !(_svLandTrim & 2 && std::fabs(d) > sweepVerify) && maxPowerPoint.slope > 0 &&
        std::fabs(dv) > 0.03f * maxPowerPoint.voltage) {
        const float d0 = converter.getCtrlOnPwmCnt(), m = (float) (converter.pwmCtrlMax / 64);
        targetDutyCycle = (uint16_t) std::clamp(d0 + std::clamp(dv / (2 * maxPowerPoint.slope), -m, m),
                                                (float) converter.pwmCtrlMin + 1, (float) converter.pwmCtrlMax - 1);
        _svLandTrim &= ~1, _svPhase = 1;
        ESP_LOGI("mppt", "sweep landing trim %.0f -> %hu", d0, targetDutyCycle);
        return;
    }
    // P&O starts afresh, towards the captured Vin
    if (ok) tracker.resetDirection(v > maxPowerPoint.voltage, v), tracker.resume(p);
    if (!(_svLandTrim & 2)) return;
    if (!ok) { // not evaluable: treated as clean, the maxima still teach
        scan.release();
        ESP_LOGI("mppt", "sweep not verified (limiter)");
        return;
    }
    sweepJudge(p, d, true);
    if (_svFlag == 2 && scan.nc <= 1 && !scan.n) _svI = p / v, scheduleResweep(nowMs, d > 0 ? 5000 : 60000);
}

// RT: a sweep cut by the Vout cap held without BMS cell data sweeps again once the data lifts the cap
__attribute__((cold, noinline)) void MpptController::capNote(float cap) {
    _capV = !g_app.psuMode() && !charger.batSt.haveValidCellVoltage() ? cap : 0;
}

__attribute__((cold, noinline)) void MpptController::capResweep(uint32_t nowMs) {
    if (!charger.terminationDecided() || !charger.batSt.haveValidCellVoltage()) return;
    if (!charger.chargeHold() && charger.Vout_max() > _capV + 0.2f) {
        ESP_LOGI("mppt", "sweep cap %.2f V lifted to %.2f V", _capV, charger.Vout_max());
        scheduleResweep(nowMs, 3000);
    }
    _capV = 0;
}

__attribute__((cold, noinline)) void MpptController::scheduleResweep(uint32_t nowMs, uint32_t inMs) {
    if (_retryAt && nowMs - _retryAt < 600000) {
        ESP_LOGI("mppt", "re-sweep capped (one per 10 min)");
        return;
    }
    _retryAt = nowMs | 1, _resweepAt = (nowMs + inMs) | 1;
    ESP_LOGI("mppt", "re-sweep in %u s", (unsigned) (inMs / 1000));
}

void MpptController::batDetectLf() {
    auto &p = charger.params;
    if (!p.vbatAuto) return;
    static PackDetect pd;
    static time_us logUs = 0;
    static const char *logWhy = nullptr;
    if (p.haveVbatMax()) {
        const float v = sensors.Vout->ewm.avg.get();
        if (!pd.left(v, converter.disabled(), p.n_cells, p.cv_eoc)) return;
        ESP_LOGW("charger", "pack left the %u-cell band (Vout %.2f V, %s), detecting again", (unsigned) p.n_cells, v,
                 pd.why);
        p.clearCells();
        charger.releaseVoutPinning("pack change");
        logWhy = nullptr;
        return;
    }
    const auto &bs = charger.batSt;
    const bool bms = bs.haveValidCellVoltage();
    const uint32_t now = (uint32_t) wallClockUs(), fresh = BatteryState::VCELL_EXPIRATION_TIME_SEC * 1000000u;
    const float v = sensors.Vout->ewm.avg.get();
    uint8_t n = 0;
    if (charger.hasBmsCellSource() && !bms && wallClockUs() < 30000000) pd.why = "waiting for BMS";
    else
        n = pd.step(v, sensors.Vin->ewm.avg.get(), converter.disabled(), p.cv_eoc, limits.Vout_max,
                    bms ? bs.vcell_high : NAN, bms && now - bs.vpack_t < fresh ? bs.vpack : NAN,
                    bms && now - bs.cell_delta_t < fresh ? bs.cell_delta : NAN);
    if (n) {
        p.setCells(n);
        ESP_LOGI("charger", "pack detected: %u cells at %.2f V -> vout_max %.2f V, fallback %.2f V%s", (unsigned) n, v,
                 p.Vbat_max, p.Vbat_fallback, bms ? ", BMS agrees" : "");
        return;
    }
    const time_us t = wallClockUs();
    if (!logWhy || t - logUs > 60000000 || (pd.why != logWhy && t - logUs > 15000000)) {
        logWhy = pd.why, logUs = t;
        ESP_LOGW("charger", "vout_max=auto: no pack yet (Vout %.2f V): %s", v, pd.why);
    }
}

// RT, off in a no-MPP backoff: a battery coming back (Vout outside the battery band, then 1..2 s inside it again) ends
// the backoff and restarts its doubling. A battery present all along (dawn) never leaves the band. After a sweep the
// Vout cap cut, the cap rising above it and Vout sweeps at once: the start path's BMS wait would end on its timeout.
__attribute__((cold, noinline)) void MpptController::noMppBatCheck() {
    if (!inBackoff()) return;
    const float v = sensors.Vout->ewm.avg.get();
    if (_capV > 0 && !g_app.psuMode() && charger.Vout_max() > std::max(_capV, v) + 0.2f) {
        ESP_LOGI("mppt", "Vout cap %.2f V lifted to %.2f V, no-MPP backoff cleared", _capV, charger.Vout_max());
        clearBackoff();
        startSweep();
        return;
    }
    if (!charger.params.haveVbatMax()) return;
    const bool tick = (wallClockUs() >> 20) & 1, flip = tick != _noMpp.tick;
    _noMpp.tick = tick;
    if (v < charger.params.n_cells * 2.5f || v > charger.params.Vbat_max * 1.03f) {
        _noMpp.open = true, _noMpp.inBand = 0;
        return;
    }
    if (!_noMpp.open || !flip || ++_noMpp.inBand < 2) return;
    _noMpp = {};
    clearBackoff();
    ESP_LOGI("mppt", "battery back (Vout %.1f V), no-MPP backoff cleared", v);
}

// RT: d, the landing or the worst return-pass probe against the down-pass, beyond sweep_verify flags the sweep. A sky
// change can hide the dips between maxima from the sweep: with a known substring count a scan probes them (rescan).
__attribute__((cold, noinline)) void MpptController::sweepJudge(float p, float d, bool rescan) {
    _svLand = p, _svFlag = std::fabs(d) > sweepVerify ? 2 : 1;
    if (_svFlag == 1) scan.release(true);
    else scan.drop(), scan.force = rescan && scan.n > 1 && scan.voc > 0;
    ESP_LOGI("mppt", "sweep %s: landed %.0f W, off by %+.0f %%%s", _svFlag == 1 ? "verified" : "flagged", p, 100 * d,
             scan.force ? ", scan now" : "");
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
