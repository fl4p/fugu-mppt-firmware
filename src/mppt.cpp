#include "mppt.h"

#include "app_state.h"        // g_app.maxLoopLag (peak RT-loop lag for telemetry)
#include "tele/telemetry.h"   // makeTelePoint / TelePoint (text or binary wire, build-time)
#ifdef WITH_BLE_TELE
#include "tele/tele_ble.h"    // teleBleStreaming gate
#endif


constexpr auto withDebugFields = false;

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

    if (converter.disabled() && !startCondition()) {
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


    float ntcTemp = ntc.last();
    if (ucTemp.last() > ntcTemp) ntcTemp = ucTemp.last();

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

    // charge current
    float Iout_max = g_app.psuMode() ? limits.Iout_max : min(limits.Iout_max, charger.Iout_max());

    // periodic sweep / scan
    // Skip while the battery is full / output-voltage-limited (CV): there's no MPP to find,
    // and ramping duty from 0 would just dump a charge pulse into a full pack (recharge-after-full).
    // Also wait out any pending backoff — a scheduled re-sweep firing into an active trip
    // timer would stall under the same livelock that startCondition() guards against.
    bool batteryFull = bool(charger.termCond) || ctrlState.mode == MpptControlMode::CV;
    if (!g_app.psuMode() && !_sweeping && !batteryFull && !inBackoff() && (nowUs - sampler.getTimeLastCalibrationUs()) > (30 * 60000000)) {
        ESP_LOGI("mppt", "periodic sweep & sensor calibration");
        g_app.maxLoopLag = 0; // restart the lag window so telemetry tracks per-sweep peak, not all-time
        startSweep();
        rtcount("mppt.update.startSweep");
        return;
    }


    constexpr auto CV = MpptControlMode::CV, CC = MpptControlMode::CC, CP = MpptControlMode::CP;

    std::array<CVP, 5> controlValues{
        CVP{CV, VinController, {sensors.Vin->med3.get(), limits.Vin_min}},
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

    for (auto &c: controlValues) {
        auto cv = c.crtl.update(c.actual, c.target, dtCtrl);

        if (!isfinite(cv) && !converter.disabled() && converter.getDutyCycle() > 0.01f) {
            ESP_LOGW("mppt", "Control value %f not finite act=%.3f tgt=%.3f idx=%i", cv, c.actual, c.target,
                     int(&c -controlValues.begin()));
            shutdownDcdc("ctrl-nan");
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

    if (limitingControlValue < 0) {
        // limit condition
        controlMode = limitingControl->mode;
        controlValue = limitingControlValue;

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
            controlMode = MpptControlMode::MPPT;
            controlValue = limitingControlValue;
        }
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
            converter.pwmPerturb(1);
        }

        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::Sweep;
            // sweep_speed scales both the per-tick cap and the limit-tracking gain,
            // so it actually controls speed near P/I/V limits, not just far from them
            controlValue = std::min(limitingControlValue * (sweepSpeed * 0.25f / 5.0f), sweepSpeed);

            // capture MPP during sweep, this will be our target afterward.
            // Ignore sub-SweepMinPower samples so a marginal-light sweep can't "peak" at a
            // near-max-duty phantom (dawn cold-start) and strand the converter there.
            if (power_smooth > maxPowerPoint.power && power_smooth >= SweepMinPower) {
                maxPowerPoint.power = power_smooth;
                maxPowerPoint.dutyCycle = converter.getCtrlOnPwmCnt();
                maxPowerPoint.voltage = sensors.Vin->med3.get();
            }

            auto u = sensors.Vin->med3.get();
            sweepPlot.pointsU.add(u, power, limits.Vin_max);

            float d = converter.getDutyCycle();
            sweepPlot.pointsD.add(d, power, 1.0f);
            rtcount("mppt.update.sweeping");
        } else {
            _stopSweep(controlMode, limitingControl ? int(limitingControl - controlValues.begin()) : -1,
                       limitingControl);
            rtcount("mppt.update.stopSweep");
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
            }
        } else {
            ESP_LOGI("mppt", "PWM fade to %i stopped at controlMode %s", (int) targetDutyCycle,
                     MpptState2String[(int) controlMode]);
            targetDutyCycle = 0;
        }
    }


    float currentThreshold = limits.reverse_current_paranoia
                                 ? (bflow.state() ? 0.05f : 0.2f)
                                 : (bflow.state() ? 0.0f : 0.1f); // hysteresis; // hysteresis
    float I_phys_smooth_min = I_phys_smooth; //std::min(I_phys_smooth, sensorPhysicalI->med3.get());
    bool aboveThres = (I_phys_smooth_min > currentThreshold
                       || (I_phys_smooth_min > 0.05 && converter.getDutyCycle() > 0.3f)
    );

    if (controlMode == MpptControlMode::None) {
        if (g_app.psuMode()) {
            controlMode = MpptControlMode::CV;
            controlValue = limitingControlValue;
        } else {
            controlMode = MpptControlMode::MPPT;
            controlValue = tracker.update(power, converter.getCtrlOnPwmCnt(), sensors.Vin->ewm.avg.get());
            controlValue *= speedScale;
        }
    } else {
        // tracker.resetTracker(power_smooth, controlValue > 0);
        tracker.resetDirection(controlValue > 0);
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
        fp = constrain(fp, -(float) converter.getCtrlOnPwmCnt(), 16.0f * (float) converter.pwmCtrlMax / 2000.f);
        converter.pwmPerturbFractional(fp);

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

    if (converter.syncRectEnabled_() != aboveThres)
        UART_LOG_ASYNC("Current %s threshold %.2f (pwm=%hu)", aboveThres ? "above" : "below", I_phys_smooth_min,
                       converter.getCtrlOnPwmCnt());
    bflow.enable((aboveThres || converter.boost() || g_app.psuMode()) && !(sensorPhysicalI->ewm.avg.get() < -0.05f && limits.reverse_current_paranoia));
    converter.enableSyncRect(aboveThres);

    rtcount("mppt.update.en");

    if (ledPinSimple != 255) {
        bool ledState = (I_phys_smooth > 0.2f && controlMode == MpptControlMode::MPPT && controlValue > 0);
        digitalWrite(ledPinSimple, ledState);
        rtcount("mppt.update.led");
    }
}

void MpptController::updateManual() {
    lastUs = wallClockUs();
    ctrlState.mode = MpptControlMode::None;

    const int16_t rampStep = std::max(1, (int) std::lround((float) converter.pwmMaxDriver() / 512.f));

    if (inBackoff()) {
        if (!converter.disabled()) converter.disable();
        return;
    }

    if (manualTarget == 0) {
        if (!converter.disabled()) {
            if (converter.getCtrlOnPwmCnt() <= converter.pwmCtrlMin)
                converter.disable();
            else
                converter.pwmPerturb(-rampStep);
        }
    } else {
        if (converter.disabled()) {
            if (!limits.reverse_current_paranoia) {
                converter.enableSyncRect(true);
                bflow.enable(true);
            }
        }
        int16_t step = constrain((int32_t)manualTarget - (int32_t)converter.getCtrlOnPwmCnt(), -rampStep, rampStep);
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
        manualTarget = std::min(targetPwmCnt, converter.pwmCtrlMax);
        ESP_LOGW("mppt", "target duty cycle PWM=%hu, manual mode (fixed duty), pwmMaxDriver=%u",
                 targetPwmCnt, (unsigned) converter.pwmMaxDriver());
    }

    auto mode = converterConf.getString("mode", "");
    if (mode == "psu" && !targetPwmCnt) {
        float vout = converterConf.getFloat("psu_vout", 0.0f);
        if (std::isfinite(vout) && vout > 0 && vout <= limits.Vout_max) {
            psuVsetpoint = vout;
            VoutController.reset();
            g_app.opMode = OpMode::Psu;
            ESP_LOGI("mppt", "PSU mode, vset=%.2fV", vout);
        } else {
            ESP_LOGE("mppt", "PSU mode but psu_vout invalid (%.2f), disabling", vout);
            g_app.setupErr = true;
        }
    } else if (!mode.empty() && mode != "mppt" && !targetPwmCnt) {
        ESP_LOGE("mppt", "Unknown converter.conf mode '%s', disabling", mode.c_str());
        g_app.setupErr = true;
    }

    sweepSpeed = std::max(0.1f, trackerConf.getFloat("sweep_speed", 4.0f));

    if (tele.influxdbHost) {
        ESP_LOGI("main", "Influxdb telemetry to host %s", tele.influxdbHost.toString().c_str());
        // sampler.onNewSample = dcdcDataChanged;  // runs on the RT sampler task — would cross
        // tasks into teleBleEnqueue (single-task invariant, see tele_ble.cpp) if re-enabled
    }
    //flags.noPanelSwitch = boardConf


    ledPinSimple = boardConf.getByte("led_simple", 255);
    if (ledPinSimple != 255) {
        pinMode(ledPinSimple, OUTPUT);
        digitalWrite(ledPinSimple, false);
    }

    fan.init(boardConf);

    bflow.init(boardConf);
    meter.load();
    if (targetPwmCnt) {
        if (!limits.reverse_current_paranoia) {
            converter.enableSyncRect(true);
            bflow.enable(true);
        }
    } else if (g_app.psuMode()) {
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
    if ((!netReady && !bleReady) || sampler.halted)
        return;

    if (wallClockUs() - _lastPointWrite < 20'000) {
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

    if ((_teleNumPoints % 20) == 0) {
        point.addField("pwm_dir_f", cntrlValue, 2);
        point.addField("mppt_state", int(ctrlState.mode));
    }

    if ((_teleNumPoints % 40) == 0) {
        point.addField("mcu_temp", ucTemp.last(), 1); // TODO to frequent
        point.addField("ntc_temp", ntc.last(), 1); // TODO to frequent
        point.addField("lag", (int) g_app.maxLoopLag); // peak RT-loop lag (µs), resets on rt-stats
    }

    point.addField("pwm_duty", converter.getCtrlOnPwmCnt());
    if (!converter.disabled() && (_teleNumPoints % 10) == 0) {
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
