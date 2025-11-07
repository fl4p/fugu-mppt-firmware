#include "mppt.h"

void MpptController::updateCV() {
    // TODO TODO
    // Temperature derating?
    auto &nowUs = wallClockUs();

    auto cv = VoutController.update(sensors.Vout->last, charger.Vbat_max());
    ctrlState.mode = MpptControlMode::CV;
    cntrlValue = cv;


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
        if (targetPwmCnt && absdiff(converter.getCtrlOnPwmCnt(), targetPwmCnt) < converter.pwmCtrlMax/512) {
            //if (fp < 0 or fp > 100)
            //    ESP_LOGI("mppt", "near tgt, fp=%.4f pwm=%hu, tgt=%hu", fp, converter.getCtrlOnPwmCnt(), targetPwmCnt);
            if ((int)fabsf(fp) < converter.pwmCtrlMax/512)
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


void MpptController::begin(const ConfFile &trackerConf, const ConfFile &pinConf, const Limits &limits_,
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
    //flags.noPanelSwitch = pinConf


    ledPinSimple = pinConf.getByte("led_simple", 255);
    if (ledPinSimple != 255) {
        pinMode(ledPinSimple, OUTPUT);
        digitalWrite(ledPinSimple, false);
    }

    fan.init(pinConf);

    bflow.init(pinConf);
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
    point.addField("pwm_ls_duty", converter.getRectOnPwmCnt());
    point.addField("pwm_ls_max", converter.getRectOnPwmMax());
    point.addField("pwm_dcm", converter.inDCM());

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
