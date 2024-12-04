#pragma once

#include "adc/sampling.h"

#include "telemetry.h"
#include "util.h"

#include "temperature.h"
#include "cooling.h"

#include "battery.h"
#include "lcd.h"
#include "tracker.h"
#include "pd_control.h"
#include "store.h"
#include "metering.h"
#include "charger.h"
#include "plot.h"

struct Limits {
    const float Vin_max{};
    const float Vin_min{};

    const float Vout_max{};

    const float Iin_max{};
    const float Iout_max{};

    const float P_max{};

    const float Temp_max{};

    const float Temp_derate{};


    explicit Limits(const ConfFile &limits)
            : Vin_max(limits.getFloat("vin_max")), Vin_min(limits.getFloat("vin_min")),
              Vout_max(limits.getFloat("vout_max")),
              Iin_max(limits.getFloat("iin_max")), Iout_max(limits.getFloat("iout_max")),
              P_max(limits.getFloat("p_max")), Temp_max(limits.getFloat("temp_max")),
              Temp_derate(limits.getFloat("temp_derate")) {
        assert(Vin_max > Vin_min);
        assert(Vin_max * Iin_max > P_max);
        assert(Vin_max * Iin_max < P_max * 4);
        assert(Temp_derate < Temp_max);
        assert(20 < Temp_max and Temp_max < 120);
    }

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

struct MpptParams : public BatChargerParams {
    float Vin_max = 80.f;
    float Vin_min = 10.5f;
    float Iin_max = 30.f;
    float P_max = 800.f;
};

enum class MpptControlMode : uint8_t {
    None = 0,
    CV,
    CC,
    CP,
    MPPT,
    Sweep,
    Max,
};


static const std::array<std::string, (size_t) MpptControlMode::Max> MpptState2String{
        "N/A",
        "CV",
        "CC",
        "CP",
        "MPPT",
        "SWEEP"
};


struct TopologyConfig {
    bool backflowAtHV = false;//backflow switch is at solar input
};


/**
 * Implements
 * - Protection of DCDC converter
 * - Voltage and current control loop
 * - MPP global scan
 * - Telemetry
 */
class MpptController {
    ADC_Sampler &dcdcPwr;
    SynchronousBuck &buck;
    LCD &lcd;


    bool autoDetectVout_max = true;

    MpptControlMode state = MpptControlMode::None;
    bool _limiting = false;// control limited (no MPPT)
    bool _sweeping = false;// global scan
    uint16_t _targetDutyCycle = 0;// MPP from global scan

    struct {
        float power = 0;
        float voltage = 0;
        uint16_t dutyCycle = 0;
    } maxPowerPoint;// MPP during sweep

    Plot sweepPlot{};

    unsigned long lastTimeProtectPassed = 0;
    unsigned long _lastPointWrite = 0;

    VIinVout<const ADC_Sampler::Sensor *> sensors{};
    const ADC_Sampler::Sensor *sensorPhysicalI{nullptr};
    const ADC_Sampler::Sensor *sensorPhysicalU{nullptr};

    PD_Control VinController{-100, -200, true}; // Vin under-voltage
    PD_Control VoutController{5000, 12000, true}; // Vout over-voltage
    PD_Control IinController{100, 400, true}; // Iin over-current
    PD_Control_SmoothSetpoint IoutCurrentController{200, 800, 200}; // Iout over-current // TODO PID?
    PD_Control_SmoothSetpoint powerController{20, 40, 200}; // over-power // TODO PID?
    //PD_Control LoadRegulationCTRL{5, -200, true}; //

public:
    Tracker tracker{};

private:
    TopologyConfig topologyConfig;

    uint8_t ledPinSimple = 255;
public:
    //MpptParams params;
    Limits limits{};
    TeleConf tele{};
    BatteryCharger charger;
    BackflowDriver bflow{};
    SolarEnergyMeter meter{};
    TempSensorGPIO_NTC ntc;
    Esp32TempSensor ucTemp;

    float speedScale = 1;

    explicit MpptController(ADC_Sampler &dcdcPwr, SynchronousBuck &pwm, LCD &lcd)
            : dcdcPwr(dcdcPwr), buck(pwm), lcd(lcd),
              charger{} {
    }

    void setSensors(const VIinVout<const ADC_Sampler::Sensor *> &channels_) {
        sensors = channels_;
        if (sensors.Iout->isVirtual) {
            ESP_LOGI("mppt", "Iout sensor is virtual, using Iin");
            sensorPhysicalI = ::sensors.Iin;
            sensorPhysicalU = ::sensors.Vin;
        } else {
            // use Iout by default
            sensorPhysicalI = ::sensors.Iout;
            sensorPhysicalU = ::sensors.Vout;
        }
        if (sensorPhysicalI->isVirtual) throw std::runtime_error("no physical I sensor");
        if (sensorPhysicalU->isVirtual) throw std::runtime_error("no physical U sensor");
    }

    void initSensors(const ConfFile &pinConf) {
        ntc.begin(pinConf);
        ucTemp.begin();
        ucTemp.read();
    }

    void begin(const ConfFile &pinConf, const Limits &limits_, const TeleConf &tele_) {
        limits = limits_;
        tele = tele_;


        ledPinSimple = pinConf.getByte("led_simple", 255);
        if (ledPinSimple != 255) {
            pinMode(ledPinSimple, OUTPUT);
            digitalWrite(ledPinSimple, false);
        }

        fanInit(pinConf);

        bflow.init(pinConf);
        meter.load();
        startSweep();
    }

    [[nodiscard]] MpptControlMode getState() const { return state; }

    void shutdownDcdc() {
        if (topologyConfig.backflowAtHV) {
            buck.disable();
            bflow.enable(false);
        } else {
            // disabling the backflow switch first to avoid any battery current into the converter
            // solar current is usually not harmful, so shutting down the converter can wait
            bflow.enable(false); // this is very fast
            buck.disable();
        }
    }

    [[nodiscard]] float boardPowerSupplyVoltage() const {
        constexpr auto diodeFwdVoltage = 0.3f;
        return std::max(sensors.Vin->last, sensors.Vout->last) - diodeFwdVoltage;
    }

    [[nodiscard]] bool boardPowerSupplyUnderVoltage(bool start = false) const {
        return boardPowerSupplyVoltage() < (start ? 9.5f : 9.f);
    }

    [[nodiscard]] bool startCondition() const {
        return !(ntc.last() > limits.Temp_derate) && ucTemp.last() < limits.Temp_derate
               && sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 1
               && !boardPowerSupplyUnderVoltage(true) && !dcdcPwr.isCalibrating();
    }

    bool protectLf(bool ignoreUV) {
        //auto nowMs = loopWallClockMs();

        // power supply under-voltage shutdown
        if (boardPowerSupplyUnderVoltage() and not ignoreUV) {
            if (!buck.disabled())
                ESP_LOGW("mppt", "Supply under-voltage! Vin %.1f and Vout %.1f < 10", sensors.Vin->last,
                         sensors.Vout->last);
            shutdownDcdc();
            enqueue_task([&] { meter.commit(); });
            return false;
        }

        // detect battery voltage
        // TODO move this to charger ?
        if (std::isnan(charger.params.Vout_max)) {
            auto vout = sensors.Vout->calibrationAvg;
            float detectedVout_max = detectMaxBatteryVoltage(vout);
            if (std::isnan(detectedVout_max)) {
                ESP_LOGW("mppt", "Unable to detect battery voltage Vout=%.2fV", vout);
                buck.disable();
                enqueue_task([&] { dcdcPwr.startCalibration(); });
                return false;
            } else {
                ESP_LOGI("mppt", "Detected max battery voltage %.2fV (from Vout=%.2fV)", detectedVout_max, vout);
                charger.params.Vout_max = min(limits.Vout_max, detectedVout_max);
            }
        }

        if (ntc.last() > limits.Temp_max || ucTemp.last() > limits.Temp_max) {
            ESP_LOGE("MPPT", "Temp %.1f (or µC %.1f) > %.1f°C, shutdown", ntc.last(), ucTemp.last(), limits.Temp_max);
            return false;
        }

        return true;
    }

    bool protect(bool ignoreUV) {

        auto nowMs = loopWallClockMs();

        // input over-voltage
        if (sensors.Vin->last > limits.Vin_max) {
            // input over-voltage
            ESP_LOGW("mppt", "Vin %.1f > %.1f!", sensors.Vin->last, limits.Vin_max);
            shutdownDcdc();
            return false;
        }

        // output over-voltage
        auto ovTh = std::min(charger.params.Vout_max * 1.05f, limits.Vout_max);
        //if (adcSampler.med3.s.chVout.get() > ovTh) {
        if (sensors.Vout->last > ovTh) { //  && sensors.Vout->previous > ovTh * 0.9f
            bool wasDisabled = buck.disabled();
            shutdownDcdc();

            auto vout = std::max(sensors.Vout->last, sensors.Vout->previous);

            if (!wasDisabled)
                ESP_LOGW("mppt", "Vout %.1fV (prev=%.1fV,ewma=%.1fV,std=%.4f,buck=%hu) > %.1fV + 5pct!",
                         sensors.Vout->last, sensors.Vout->previous,
                         sensors.Vout->ewm.avg.get(), sensors.Vout->ewm.std.get(), buck.getBuckOnPwmCnt(),
                         charger.params.Vout_max
                );


            if (autoDetectVout_max && nowMs - lastTimeProtectPassed > 20000) {
                // if the OV condition persists for some seconds, auto-detect Vout_max
                charger.params.Vout_max = NAN;
                dcdcPwr.startCalibration();
            }

            enqueue_task([&] {
                lcd.displayMessageF("OV shutdown!\nVout=%.1fV max=%.1fV", 10000, vout, ovTh);
            });

            return false;
        }


        // input over current
        if (sensors.Iin->last > limits.Iin_max * 1.3f && !buck.disabled()) {
            shutdownDcdc();
            ESP_LOGW("mppt", "Input current %.1f > 1.5x limit (Iout=%.1f, Vin=%.2f, Iin=%.2f), shutdown",
                     sensors.Iin->last,
                     sensors.Iout->last, sensors.Vin->last, sensors.Iin->last);
            return false;
        }

        // output over current
        if ((sensors.Iout->last > limits.Iout_max * 1.25f or sensors.Iout->ewm.avg.get() > (limits.Iout_max + 5.f)
            ) and not buck.disabled()) {
            shutdownDcdc();
            ESP_LOGW("mppt", "Output Current %.2f above limit %.2f, shutdown", sensors.Iout->last, limits.Iout_max);
            return false;
        }

        if (sensorPhysicalI->last < -1 && sensorPhysicalI->previous < -1) {
            if (sensors.Iout->ewm.avg.get() > 10) {
                //buck.halfDutyCycle();
                shutdownDcdc();
                ESP_LOGE("MPPT", "Reverse current %.2f A, noise? High avg current, shutdown", sensorPhysicalI->last);
            } else {
                bflow.enable(false); // reverse current
                buck.lowSideMinDuty();
                ESP_LOGE("MPPT", "Reverse current %.2f A, noise? disable BFC and low-side FET", sensorPhysicalI->last);
            }
        }

        if (sensorPhysicalI->ewm.avg.get() < -1) {
            ESP_LOGE("MPPT", "Reverse avg current shutdown %.1f A", sensorPhysicalI->ewm.avg.get());
            shutdownDcdc();
            return false;
        }

        if (sensors.Vout->ewm.avg.get() > (sensors.Vin->ewm.avg.get() + 1.0f) * 1.25f) {
            if (!buck.disabled())
                ESP_LOGE("MPPT", "Vout %.1f > Vin %.1f, shutdown duty=%i", sensors.Vout->ewm.avg.get(),
                         sensors.Vin->ewm.avg.get(), (int) buck.getBuckOnPwmCnt());
            shutdownDcdc();
            return false;
        }

        if (sensors.Vout->last > (sensors.Vin->last + .5f) * 2) {
            ESP_LOGE("MPPT", "Vout %.1f > 2x Vin %.1f, shutdown", sensors.Vout->last, sensors.Vin->last);
            shutdownDcdc();
            return false;
        }

        // try to prevent voltage boost and disable low side for low currents
        auto currentFilt = fminf(sensorPhysicalI->ewm.avg.get(),
                                 std::max(sensorPhysicalI->last, sensorPhysicalI->previous));
        if (currentFilt < -0.05f) {
            if (buck.getBuckDutyCycleLS() > buck.getDutyCycleLSMax() / 2 &&
                buck.getBuckDutyCycleLS() > (buck.pwmMaxHS / 10)) {
                ESP_LOGW("MPPT", "Low current, set low-side min duty (ewm(Iin)=%.2f, max(Iin,Iin[-1])=%.2f)",
                         sensors.Iin->ewm.avg.get(), std::max(sensors.Iin->last, sensors.Iin->previous));
            }
            if (bflow.state())
                ESP_LOGW("MPPT", "Low current %.2f, disable backflow", currentFilt);
            buck.lowSideMinDuty();
            bflow.enable(false); // low current
        }


        if (sensorPhysicalI->ewm.avg.get() > 6 && !bflow.state()) {
            if (!buck.disabled())
                ESP_LOGE("MPPT", "High-current through open backflow switch!");
            shutdownDcdc();
            return false;
        }


        // TODO move this to control loop
        float vOut = fmaxf(sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get());
        float vIn = fminf(sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get());
        //float vOut = sensors.Vout->ewm.avg.get();
        //float vIn = sensors.Vin->ewm.avg.get();
        auto vr = buck.updateLowSideMaxDuty(vOut, vIn);

        auto iOutSmall = sensorPhysicalI->ewm.avg.get() < (limits.Iout_max * 0.01f);

        if (iOutSmall && buck.getBuckOnPwmCnt() > buck.pwmMinLS * 2 and
            (vOut < 1 or (buck.getBuckDutyCycle() * 0.9f) > vr)) {

            if (!buck.disabled())
                ESP_LOGE("MPPT",
                         "Buck running at D=%d%% but Vout (%.2f) and Iout (%.2f, last=%.2f) low! Sensor or half-bridge failure.",
                         100 * buck.getBuckOnPwmCnt() / buck.pwmMaxHS, vOut, sensors.Iout->ewm.avg.get(),
                         sensors.Iout->last
                );

            shutdownDcdc();
            return false;
        }

        lastTimeProtectPassed = nowMs;

        return true;
    }


    /**
     * Start a global MPPT scan.
     */
    void startSweep() {
        buck.disable();
        _limiting = false;
        _targetDutyCycle = 0;

        VinController.reset();
        VoutController.reset();
        IinController.reset();
        IoutCurrentController.reset();
        //LoadRegulationCTRL.reset();

        ESP_LOGI("mppt", "Start sweep");

        _sweeping = true;
        maxPowerPoint = {};

        dcdcPwr.startCalibration();

        enqueue_task([&] {
            rtcount_en = false;
            vTaskDelay(10);
            sweepPlot.reserve();
            meter.commit(); // not real-time safe
            lcd.periodicInit(); // not real-time safe
            rtcount_en = true;
        });
    }


    /**
     * Stops MPPT scan and set duty cycle to captured MPP
     * @param controlMode
     */
    void _stopSweep(MpptControlMode controlMode, int limIdx) {
        _sweeping = false;
        _targetDutyCycle = maxPowerPoint.dutyCycle;

        enqueue_task([&, controlMode, limIdx] {
            ESP_LOGI("mppt", "Stop sweep after %.2fs at controlMode=%s (limIdx=%i) PWM=%hu, MPP=(%.1fW,PWM=%hu,%.1fV)",
                     (loopWallClockUs() - dcdcPwr.getTimeLastCalibrationUs()) * 1e-6f,
                     MpptState2String[(uint8_t) controlMode].c_str(), limIdx,
                     buck.getBuckOnPwmCnt(), maxPowerPoint.power, maxPowerPoint.dutyCycle, maxPowerPoint.voltage
            );
            lcd.displayMessageF("MPP Scan done\n%.1fW @ %.1fV", 6000, maxPowerPoint.power, maxPowerPoint.voltage);
            sweepPlot.plot();
        });
    }

    void telemetry() {
        if (!WiFi.isConnected() || !tele.influxdbHost)
            return;

        auto I_phys_smooth = (sensorPhysicalI->ewm.avg.get());
        auto V_phys_smooth = (sensorPhysicalU->ewm.avg.get());
        //auto Vout(sensors.Vout->ewm.avg.get());
        float power_smooth = I_phys_smooth * V_phys_smooth;
        float power = sensorPhysicalI->last * sensorPhysicalU->last;

        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField("I", I_phys_smooth, 2);
        point.addField("U", V_phys_smooth, 2);
        point.addField("P", power, 2);
        point.addField("P_smooth", power_smooth, 2);
        //point.addField("U_out", Vout, 2);


        point.addField("E", meter.totalEnergy.get(), 1);
        point.addField("E_today", meter.dailyEnergyMeter.today.energyYield, 1);

        point.addField("pwm_duty", buck.getBuckOnPwmCnt());
        point.addField("pwm_ls_duty", buck.getBuckDutyCycleLS());
        point.addField("pwm_ls_max", buck.getDutyCycleLSMax());

        auto nowMs = loopWallClockMs();

        point.setTime(WritePrecision::MS);

        if (nowMs - _lastPointWrite > 10) {
            telemetryAddPoint(point, 80);
            _lastPointWrite = nowMs;
        }
    }

    /* // sensor API now computes this through virtual sensors
     * float getIoutSmooth(float conversionEff = 0.97f) const {
        if (boardPowerSupplyUnderVoltage())
            return 0;
        auto pin = sensors.Iin->ewm.avg.get() * sensors.Vin->ewm.avg.get();
        return pin * conversionEff / std::max(sensors.Vout->ewm.avg.get(), 2.f);
    } */


    unsigned long lastUs = 0;

    /**
     * - Energy counter
     * - voltage and current control
     * - calls mpp tracker
     */
    void update() {
        auto nowMs = loopWallClockMs();
        auto &nowUs = loopWallClockUs();

        if (buck.disabled() && !startCondition()) {
            bflow.enable(false);
            state = MpptControlMode::None;
            return;
        }

        auto I_phys_smooth = (sensorPhysicalI->ewm.avg.get());
        auto V_phys_smooth = (sensorPhysicalU->ewm.avg.get());
        //auto Vout(sensors.Vout->ewm.avg.get());
        float power_smooth = I_phys_smooth * V_phys_smooth;
        float power = sensorPhysicalI->med3.get() * sensorPhysicalU->med3.get();

        //avgIin.add(adcSampler.last.s.chIin);
        //avgVin.add(adcSampler.last.s.chVin);
        //float smoothPower = avgIin.get() * avgVin.get();


        meter.add(sensors.Iout->last * sensors.Vout->last, power_smooth, sensors.Vin->ewm.avg.get(),
                  sensors.Vout->ewm.avg.get(), nowUs);
        rtcount("mppt.update.meterAdd");


        float ntcTemp = ntc.last();
        if (ucTemp.last() > ntcTemp) ntcTemp = ucTemp.last();

        fanUpdateTemp(ntcTemp, power_smooth);
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

        // topping current
        float Iout_max = limits.Iout_max; //charger.getToppingCurrent(::sensors.Vout->ewm.avg.get());

        // periodic sweep / scan
        if (!_sweeping /*&& power_smooth < 30*/ && (nowUs - dcdcPwr.getTimeLastCalibrationUs()) > (30 * 60000000)) {
            ESP_LOGI("mppt", "periodic sweep & sensor calibration");
            startSweep();
            rtcount("mppt.update.startSweep");
            return;
        }


        struct CVP {
            MpptControlMode mode;
            PD_Control &crtl;
            struct {
                float actual, target;
            };
        };

        constexpr auto CV = MpptControlMode::CV, CC = MpptControlMode::CC, CP = MpptControlMode::CP;

        std::array<CVP, 5> controlValues{
                CVP{CV, VinController, {sensors.Vin->med3.get(), limits.Vin_min}},
                CVP{CV, VoutController, {sensors.Vout->last, charger.params.Vout_max}},
                CVP{CC, IinController, {sensors.Iin->med3.get(), limits.Iin_max}},
                CVP{CC, IoutCurrentController, {sensors.Iout->med3.get(), Iout_max}},
                CVP{CP, powerController, {power_smooth, powerLimit}},
                //CVP{CC, LoadRegulationCTRL, {sensors.Iout->last, Iout_max * 1.5f}},
        };

        // TODO sum negative values

        CVP *limitingControl = nullptr;
        float limitingControlValue = std::numeric_limits<float>::infinity();
        for (auto &c: controlValues) {
            auto cv = c.crtl.update(c.actual, c.target);

            if (!isfinite(cv)) {
                ESP_LOGW("mppt", "Control value %f not finite act=%.3f tgt=%.3f idx=%i", cv, c.actual, c.target,
                         int(&c -controlValues.begin()));
                shutdownDcdc();
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


            _limiting = true;
        } else {
            // no limit condition
            if (_limiting) {
                // recover from limit condition
                _limiting = false;
                controlMode = MpptControlMode::MPPT;
                controlValue = limitingControlValue;
            }
        }

        // bounce at pwm boundary
        if (buck.getBuckOnPwmCnt() == buck.pwmMaxHS) {
            controlMode = MpptControlMode::CV;
            controlValue = -1;
        } else if (buck.getBuckOnPwmCnt() == buck.pwmMinHS && !_sweeping) {
            controlMode = MpptControlMode::CV;
            controlValue = 1;
        }

        // THIS CAN FAIL:
        // assert((controlMode == MpptControlMode::None) == (controlValue == 0));

        rtcount("mppt.update.control");

        if (_sweeping && !dcdcPwr.isCalibrating()) {
            if (controlMode == MpptControlMode::None) {
                controlMode = MpptControlMode::Sweep;
                controlValue = 2; // sweep speed

                // capture MPP during sweep
                if (power_smooth > maxPowerPoint.power) {
                    maxPowerPoint.power = power;
                    maxPowerPoint.dutyCycle = buck.getBuckOnPwmCnt();
                    maxPowerPoint.voltage = sensors.Vin->med3.get();
                }

                auto u = sensors.Vin->med3.get();
                sweepPlot.pointsU.add(u, power, limits.Vin_max);

                float d = buck.getBuckOnPwmCnt() / (float) buck.pwmMaxHS;
                sweepPlot.pointsD.add(d, power, 1.0f);
                rtcount("mppt.update.sweeping");
            } else {
                _stopSweep(controlMode, limitingControl ? int(limitingControl - controlValues.begin()) : -1);
                rtcount("mppt.update.stopSweep");
            }
        } else if (_targetDutyCycle) {
            if (controlMode == MpptControlMode::None or
                (controlMode == MpptControlMode::CV && buck.getBuckOnPwmCnt() > _targetDutyCycle)) {
                controlMode = MpptControlMode::Sweep;
                controlValue = (float) constrain(_targetDutyCycle - buck.getBuckOnPwmCnt(), -8, 2);
                if (std::fabs(controlValue) <= 1) {
                    ESP_LOGI("mppt", "Reached target duty cycle %hu", _targetDutyCycle);
                    _targetDutyCycle = 0;
                }
            } else {
                ESP_LOGI("mppt", "PWM fade to %i stopped at controlMode %s", (int) _targetDutyCycle,
                         MpptState2String[(int) controlMode].c_str());
                _targetDutyCycle = 0;
            }
        }

        //
        if (controlMode == MpptControlMode::None) {
            controlMode = MpptControlMode::MPPT;
            controlValue = tracker.update(power, buck.getBuckOnPwmCnt());
            controlValue *= speedScale;
        } else {
            // tracker.resetTracker(power_smooth, controlValue > 0);
            tracker.resetDirection(controlValue > 0);
        }
        rtcount("mppt.update.tracker");

        //assert(controlValue != 0 && controlMode != MpptControlMode::None);

        // always cap control value
        // TODO instead of capping, use fade-to-target. the tracker might return big jumps
        controlValue = std::min(controlValue, limitingControlValue);
        this->state = controlMode;


        if (lastUs) {
            // normalize the control value to pwmMax and scale it with update rate to fix buck slope rate
            auto dt_us = nowUs - lastUs;
            auto fp = controlValue * (1.f / 2000.f) * (float) buck.pwmMaxHS * (float) dt_us * 1e-6f * 25.f;
            if (!_sweeping && buck.getBuckOnPwmCnt() < buck.pwmMinHS * 2) {
                // slow-down control loop for low duty cycles (low-load condition)
                // TODO does this makes sense? the aim here is to stabilize Vout in low/no-load condition
                // can also slow-down the VoutCNTRL
                fp *= 0.2f;
            }

            // constrain the buck step, this will slow down control for lower loop rates:
            fp = constrain(fp, -(float) buck.getBuckOnPwmCnt(), 1.0f);
            buck.pwmPerturbFractional(fp);

            if (controlValue < -80 and fp < -0.01) {
                auto limIdx = (int) (limitingControl - controlValues.begin());

                UART_LOG_ASYNC(
                        "Limiting! Control value %.2f => perturbation %.2f, mode=%s, idx=%i (act=%.3f, tgt=%.3f)",
                        controlValue, fp,
                        MpptState2String[(int) controlMode].c_str(), limIdx, limitingControl->actual,
                        limitingControl->target);

                if (controlMode == MpptControlMode::CC)
                    UART_LOG_ASYNC("Iout_max=%.2f powerLimit=%.2f", Iout_max, powerLimit);
            }
            rtcount("mppt.update.pwm");
        }
        lastUs = nowUs;

        float currentThreshold = bflow.state() ? 0.05f : 0.2f; // hysteresis
        float I_phys_smooth_min = I_phys_smooth; //std::min(I_phys_smooth, sensorPhysicalI->med3.get());
        bool aboveThres = (I_phys_smooth_min > currentThreshold);
        if (bflow.state() != aboveThres)
            UART_LOG_ASYNC("Current %s threshold %.2f", aboveThres ? "above" : "below", I_phys_smooth_min);
        bflow.enable(aboveThres);
        buck.enableLowSide(aboveThres);

        rtcount("mppt.update.en");

        if (ledPinSimple != 255) {
            bool ledState = (I_phys_smooth > 0.2f && controlMode == MpptControlMode::MPPT && controlValue > 0);
            digitalWrite(ledPinSimple, ledState);
            rtcount("mppt.update.led");
        }

        // tele disabled: sps=275, enabled: sps=165
        if (tele.influxdbHost and WiFi.isConnected()) {

            Point point("mppt");

            point.addTag("device", "fugu_" + String(getChipId()));
            point.addField("I", I_phys_smooth, 2);
            point.addField("U", V_phys_smooth, 2);
            //point.addField("U_out", sensors.Vout->ewm.avg.get(), 2);
            point.addField("P", power, 2);
            point.addField("P_smooth", power_smooth, 2);
            point.addField("E", meter.totalEnergy.get(), 1);


            point.addField("pwm_dir_f", controlValue, 2);
            point.addField("mppt_state", int(controlMode));
            // point.addField("mcu_temp", mcu_temp.last(), 1);
            point.addField("ntc_temp", ntcTemp, 1);
            point.addField("pwm_duty", buck.getBuckOnPwmCnt());
            point.addField("pwm_ls_duty", buck.getBuckDutyCycleLS());
            point.addField("pwm_ls_max", buck.getDutyCycleLSMax());


            if (controlMode == MpptControlMode::MPPT) {
                auto dP = tracker.dP;
                point.addField("P_prev", tracker._lastPower, 2);
                point.addField("dP", dP, 2);
                //point.addField("P_filt", tracker.pwmPowerTable[buck.getBuckDutyCycle()].get(), 1);
                point.addField("P_filt", tracker._powerBuf.getMean(), 1);
                if (std::abs(dP) < tracker.minPowerStep) {
                    point.addField("dP_thres", 0.0f, 2);
                } else {
                    point.addField("dP_thres", dP, 2);
                }
            }

            if (limitingControl) {
                auto limIdx = (int) (limitingControl - controlValues.begin());
                point.addField("cv_lim_idx", limIdx);
            }

            point.setTime(WritePrecision::MS);

            if (nowMs - _lastPointWrite > 10) {
                telemetryAddPoint(point, 80);
                _lastPointWrite = nowMs;
            }

            rtcount("mppt.update.tele");
        }
    }


};