#include "mppt.h"

#include "Point.h"
#include "WritePrecision.h"


/**
 * - Energy counter
 * - voltage and current control
 * - calls mpp tracker
 */
void MpptController::update() {
    if (targetPwmCnt) {
        updateCV();
        return;
    }

    //auto nowMs = wallClockMs();
    auto &nowUs = wallClockUs();

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
    float Iout_max = min(limits.Iout_max, charger.Iout_max());

    // periodic sweep / scan
    if (!_sweeping /*&& power_smooth < 30*/ && (nowUs - sampler.getTimeLastCalibrationUs()) > (30 * 60000000)) {
        ESP_LOGI("mppt", "periodic sweep & sensor calibration");
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
                charger.Vout_max()
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
        auto cv = c.crtl.update(c.actual, c.target);

        if (!isfinite(cv) && !converter.disabled() && converter.getDutyCycle() > 0.01f) {
            ESP_LOGW("mppt", "Control value %f not finite act=%.3f tgt=%.3f idx=%i", cv, c.actual, c.target,
                     int(&c -controlValues.begin()));
            shutdownDcdc();
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
        if (converter.disabled()) converter.pwmPerturb(1);

        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::Sweep;
            controlValue = std::min(limitingControlValue / 5.0f, 4.0f); // sweep speed
            //controlValue = std::min(limitingControlValue / 8.0f, 2.0f); // sweep speed

            // capture MPP during sweep, this will be our target afterward
            if (power_smooth > maxPowerPoint.power) {
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
            controlValue = (float) constrain(targetDutyCycle - converter.getCtrlOnPwmCnt(), -8, 2);
            if (std::fabs(controlValue) <= 1) {
                ESP_LOGI("mppt", "Reached target duty cycle %hu", targetDutyCycle);
                targetDutyCycle = 0;
            }
        } else {
            ESP_LOGI("mppt", "PWM fade to %i stopped at controlMode %s", (int) targetDutyCycle,
                     MpptState2String[(int) controlMode].c_str());
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
        controlMode = MpptControlMode::MPPT;
        controlValue = tracker.update(power, converter.getCtrlOnPwmCnt(), sensors.Vin->ewm.avg.get());
        controlValue *= speedScale;
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
        auto dt_us = nowUs - lastUs;
        auto fp = controlValue * (1.f / 2000.f) * (float) converter.pwmCtrlMax * (float) dt_us * 1e-6f * 25.f * 2.f;
        if (!_sweeping && converter.getCtrlOnPwmCnt() < converter.pwmCtrlMin * 2) {
            // slow-down control loop for low duty cycles (low-load condition)
            // TODO does this makes sense? the aim here is to stabilize Vout in low/no-load condition
            // can also slow-down the VoutCNTRL
            //fp *= 0.2f;
        }

        // constrain the buck step, this will slow down control for lower loop rates:
        // this causes very slow load response time, but works well when battery is connected
        fp = constrain(fp, -(float) converter.getCtrlOnPwmCnt(), 16.0f);
        converter.pwmPerturbFractional(fp);

        if (controlValue < -80 and fp < -0.01 and converter.getCtrlOnPwmCnt() > converter.getCtrlOnPwmMin()) {
            UART_LOG_ASYNC(
                "Limiting! Control value %.2f => perturbation %.2f (to %hu), mode=%s, idx=%i (act=%.3f, tgt=%.3f)",
                controlValue, fp, converter.getCtrlOnPwmCnt(),
                MpptState2String[(int) controlMode].c_str(), ctrlState.limIdx, limitingControl->actual,
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
    bflow.enable(aboveThres || converter.boost()); // the backflow switch is only useful in buck topology
    converter.enableSyncRect(aboveThres);

    rtcount("mppt.update.en");

    if (ledPinSimple != 255) {
        bool ledState = (I_phys_smooth > 0.2f && controlMode == MpptControlMode::MPPT && controlValue > 0);
        digitalWrite(ledPinSimple, ledState);
        rtcount("mppt.update.led");
    }
}

void MpptController::updateCV() {
    // TODO TODO
    // Temperature derating?
    auto &nowUs = wallClockUs();

    auto cv = VoutController.update(sensors.Vout->last, charger.Vout_max());
    ctrlState.mode = MpptControlMode::CV;
    cntrlValue = cv;

    if (!std::isfinite(cv)) {
        if (!converter.disabled()) {
            ESP_LOGW("mppt", "Control value %f not finite act=%.3f tgt=%.3f idx=%i", cv, sensors.Vout->last,
                     charger.Vout_max(), 0);
            shutdownDcdc();
        }
        return;
    }

    float currentThreshold = limits.reverse_current_paranoia
                                 ? (bflow.state() ? 0.05f : 0.2f)
                                 : (bflow.state() ? 0.0f : 0.1f); // hysteresis; // hysteresis
    float I_phys_smooth_min = sensorPhysicalI->ewm.avg.get();
    bool aboveThres = (I_phys_smooth_min > currentThreshold
                       || (I_phys_smooth_min > -0.01 && converter.getDutyCycle() > 0.3f)
    );

    /*
        if (targetPwmCnt) {
            // no tracking,
            controlMode = MpptControlMode::MPPT;
            auto cnt = converter.getCtrlOnPwmCnt();
            controlValue = cnt == targetPwmCnt ? 0 : (cnt > targetPwmCnt) ? -1 : std::min(
                    voutCtrlVal, (float)targetPwmCnt - cnt);
        } */


    if (lastUs) {
        // normalize the control value to pwmMax and scale it with update rate to fix buck slope rate
        auto dt_us = nowUs - lastUs;
        auto fp = cv * (1.f / 10000.f) * (float) converter.pwmCtrlMax * (float) dt_us * 1e-6f * 25.f * 2.f;

        if (converter.getCtrlOnPwmCnt() < 160) {
            fp *= 0.01f;
        } else if (converter.getCtrlOnPwmCnt() < 200) {
            fp *= 0.04f;
        } else {
            fp *= 10.0f;
        }

        if ((fp + (float) converter.getCtrlOnPwmCnt() > (float) targetPwmCnt)) {
            fp = (float) targetPwmCnt - (float) converter.getCtrlOnPwmCnt();
        }

        // no pwm jitter near target, "lock-in"
        if (targetPwmCnt && absdiff(converter.getCtrlOnPwmCnt(), targetPwmCnt) < converter.pwmCtrlMax / 512) {
            //if (fp < 0 or fp > 100)
            //    ESP_LOGI("mppt", "near tgt, fp=%.4f pwm=%hu, tgt=%hu", fp, converter.getCtrlOnPwmCnt(), targetPwmCnt);
            if ((int) fabsf(fp) < converter.pwmCtrlMax / 512)
                fp = (float) targetPwmCnt - (float) converter.getCtrlOnPwmCnt();
        } /*else         if (targetPwmCnt && absdiff(converter.getCtrlOnPwmCnt(), targetPwmCnt) < converter.pwmCtrlMax/128) {
            //if (fp < 0 or fp > 100)
            //    ESP_LOGI("mppt", "near tgt,-5 fp=%.4f pwm=%hu, tgt=%hu", fp, converter.getCtrlOnPwmCnt(), targetPwmCnt);
            if ((int)fabsf(fp) < converter.pwmCtrlMax/512)
                fp = 0;
        }*/

        fp = constrain(fp, -(float) converter.getCtrlOnPwmCnt(), 16.0f);
        converter.pwmPerturbFractional(fp);

        rtcount("mppt.update.pwm");
    }
    lastUs = nowUs;

    if (bflow.state() != aboveThres)
        UART_LOG_ASYNC("Current %s threshold %.2f (pwm=%hu)", aboveThres ? "above" : "below", I_phys_smooth_min,
                       converter.getCtrlOnPwmCnt());

    bflow.enable(aboveThres);
    converter.enableSyncRect(aboveThres);

    rtcount("mppt.update.en");
}


void MpptController::updateManual() {
    if (targetDutyCycle) {
        int16_t step = constrain(targetDutyCycle - converter.getCtrlOnPwmCnt(), -4, 4);
        if (step == 0) {
            ESP_LOGI("mppt", "Reached target duty cycle %hu", targetDutyCycle);
            targetDutyCycle = 0;
        } else {
            converter.pwmPerturb(step);
        }
    }

}


void MpptController::begin(const ConfFile &trackerConf, const ConfFile &boardConf, const Limits &limits_,
                           const TeleConf &tele_) {
    limits = limits_;
    tele = tele_;

    targetPwmCnt = (uint16_t) std::round(
        trackerConf.getFloat("target_duty_cycle", 0.0f) * (float) converter.pwmMaxDriver());

    if (targetPwmCnt) {
        ESP_LOGW("mppt", "target duty cycle PWM=%.2hu, not performing tracking!", targetPwmCnt);
    }

    if (tele.influxdbHost) {
        ESP_LOGI("main", "Influxdb telemetry to host %s", tele.influxdbHost.toString().c_str());
        // sampler.onNewSample = dcdcDataChanged;
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
    startSweep();
}

void MpptController::telemetry() {
    if (!WiFi.isConnected() || !tele.influxdbHost || !timeSynced)
        return;

    if (wallClockUs() - _lastPointWrite < 20000) {
        return;
    }

    auto I_phys_smooth = (sensorPhysicalI->ewm.avg.get());
    auto V_phys_smooth = (sensorPhysicalU->ewm.avg.get());
    //auto Vout(sensors.Vout->ewm.avg.get());
    float power_smooth = I_phys_smooth * V_phys_smooth;
    float power = sensorPhysicalI->med3.get() * sensorPhysicalU->med3.get();


    Point point("mppt");
    point.addTag("device", getHostname().c_str());
    point.addField("I", sensorPhysicalI->med3.get(), 3);
    point.addField("Ui", sensors.Vin->med3.get(), 2);
    point.addField("Uo", sensors.Vout->med3.get(), 2);
    //point.addField("U", V_phys_smooth, 2);
    point.addField("P", power, 2);
    point.addField("P_smooth", power_smooth, 2);
    //point.addField("U_out", Vout, 2);


    point.addField("E", meter.totalEnergy.get(), 1);
    point.addField("E_today", meter.dailyEnergyMeter.today.energyYield, 1);


    point.addField("pwm_dir_f", cntrlValue, 2);
    point.addField("mppt_state", int(ctrlState.mode));
    point.addField("mcu_temp", ucTemp.last(), 1);
    point.addField("ntc_temp", ntc.last(), 1);

    point.addField("pwm_duty", converter.getCtrlOnPwmCnt());
    if (!converter.disabled()) {
        point.addField("pwm_ls_duty", converter.getRectOnPwmCnt());
        point.addField("pwm_ls_max", converter.getRectOnPwmMax());
        point.addField("pwm_dcm", converter.inDCM());
    }

    if (ctrlState.mode == MpptControlMode::MPPT) {
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

    if (!limIdxSampled.empty()) {
        point.addField("cv_lim_idx", limIdxSampled.pop());
    }

    point.setTime(WritePrecision::MS);

    telemetryAddPoint(point, 80);
    _lastPointWrite = wallClockUs();
}
