#include "sensor_setup.h"

#include <Arduino.h>
#include <esp_heap_caps.h>

#include <string>
#include <unordered_map>

#include "adc.h"
#include "ads.h"
#include "adc_esp32_cont.h"
#include "mock.h"
#include "ina226.h"
#include "sampling.h"
#if WITH_VCONV
#include "adc/vconv.h"
#include "sim/vconv.h"
#endif
#include "../mppt.h"
#include "../conf.h"
#include "../util.h"

// Globals owned by main.cpp.
extern ADC_Sampler adcSampler;
extern VIinVout<const Sensor *> sensors;
extern MpptController mppt;
extern float conversionEfficiency;

#include "../app_state.h"

static AsyncADC<float> *createAdcInstance(const std::string &adcName, const ConfFile &boardConf,
                                          const ConfFile &sensConf, const std::string &chnDebug) {
    AsyncADC<float> *adc;
    if (adcName == "ads1115" or adcName == "ads1015") {
        adc = new ADC_ADS(adcName == "ads1115");
    } else if (adcName == "ina226") {
        adc = new ADC_INA226();
    } else if (adcName == "esp32adc1") {
        // assert_throw(ntc_ch== 255, "adc1 conflicts ntc impl TODO fix");
        adc = new ADC_ESP32_Cont(sensConf);
        ((ADC_ESP32_Cont *) adc)->setNtcCh(sensConf.getByte("ntc_ch", 255));
    } else if (adcName == "fake") {
        adc = new ADC_Fake();
#if WITH_VCONV
    } else if (adcName == "vconv") {
        adc = new ADC_VConv();
#endif
    } else {
        throw std::runtime_error("unknown ADC '" + adcName + "'" + " " + chnDebug);
    }

    if (!adc->init(boardConf)) {
        delete adc;
        throw std::runtime_error("failed to initialize ADC " + adcName);
    }

    return adc;
}

#if WITH_VCONV
static void configureVirtualConverter() {
    ConfFile vc{"/littlefs/conf/vconv.conf"};
    if (!vc) {
        ESP_LOGW("vconv", "no vconv.conf found, using defaults");
    }
    float isc = vc.f("isc", 8.0f);
    float voc = vc.f("voc", 40.0f);
    float k   = vc.f("pv_k", 0.8f);
    g_vconv.setPv(isc, voc, k);

    const float vBat = vc.f("v_bat", 28.0f);
    const float rBat = vc.f("r_bat", 0.05f);
    g_vconv.setBat(vBat, rBat);
    g_vconv.setBatRipple(vc.f("vbat_ac_amp", 0.0f), vc.f("vbat_ac_freq", 100.0f),
                         (int) vc.getByte("vbat_ac_shape", 0));

    ConfFile coil{"/littlefs/conf/coil.conf"};
    float L0 = coil.f("L0", 50e-6f);
    g_vconv.setPassives(vc.f("c_in", 470e-6f), vc.f("c_out", 470e-6f), L0);

    // Topology comes from converter.conf, not vconv.conf: the plant must match whatever buck.h was
    // told, or the Ctrl/Rect gate roles and the sim disagree. We run before converter.init(), so we
    // cannot ask the converter — reject an unrecognised value the same way it does rather than
    // silently modelling a buck while the firmware drives a boost.
    ConfFile conv{"/littlefs/conf/converter.conf"};
    const std::string topo = conv.getString("topo", "buck");
    assert_throw(topo == "buck" || topo == "boost", "vconv: converter.conf::topo must be buck|boost");
    const bool boost = topo == "boost";
    g_vconv.setBoost(boost);

    // Seed cap voltages near steady-state so the model doesn't start in the
    // sub-MinRatioVoltage relaxation branch. In boost V_out sits ABOVE V_in, so seeding it
    // at v_bat with V_in at voc/2 would start the model below its own passthrough floor.
    g_vconv.setVin(voc * 0.5f);
    g_vconv.setVout((boost && vBat < voc * 0.5f) ? voc * 0.5f : vBat);

    ESP_LOGI("vconv", "%s PV Isc=%.2fA Voc=%.2fV k=%.2f  Bat=%.2fV/%.3fΩ  L0=%.1fµH",
             boost ? "boost" : "buck", isc, voc, k, vBat, rBat, L0 * 1e6f);
}
#endif

void setupSensors(const ConfFile &boardConf, const Limits &lim) {
    loopWallClockUs_ = esp_timer_get_time();
    heap_caps_check_integrity_all(true);

#if WITH_VCONV
    configureVirtualConverter();
#endif

    /// compute voltage factor for a resistor divider network with 2 parallel R_low (Rh+(Rl+Ra))
    auto adcVDiv = [](float Rh, float Rl, float Ra) {
        auto rl = 1 / (1 / Rl + 1 / Ra);
        return LinearTransform{(Rh + rl) / rl, 0.f};
    };

    ConfFile sensConf{"/littlefs/conf/sensor.conf"};

    if (!sensConf) {
        throw std::runtime_error("no sensor conf");
    }

    g_app.loopRateMin = sensConf.getByte("expected_hz", 0);
    conversionEfficiency = sensConf.f("power_conversion_eff", 0.95f);
    assert_throw(conversionEfficiency > 0.5f and conversionEfficiency < 1.0f, "");

    struct p {
        SensorParams params;
        AsyncADC<float> *adc;
        uint16_t filtLen;
    };
    std::unordered_map<std::string, p> params;

    auto defAdcName = sensConf.getString("adc", "");
    std::unordered_map<std::string, AsyncADC<float> *> adcs{};
    for (auto chn_: {"ntc", "vin", "iin", "iout", "vout",}) {
        auto chn = std::string(chn_);
        auto chNum = sensConf.getByte(chn + '_' + "ch", 255);
        auto an = sensConf.getString(chn + '_' + "adc", defAdcName);

        if (chNum != 255 && adcs.find(an) == adcs.end()) {
            adcs[an] = createAdcInstance(an, boardConf, sensConf, chn);
        }
        auto adc = chNum != 255 ? adcs[an] : nullptr;

        LinearTransform lt{1.f, 0.f};

        if (chNum != 255) {
            if (chn[0] == 'v') {
                lt = adcVDiv(
                    sensConf.f(chn + '_' + "rh"),
                    sensConf.getFloat(chn + '_' + "rl"),
                    adc->getInputImpedance(chNum)
                );
            } else if (chn[0] == 'i') {
                lt = {
                    sensConf.f(chn + '_' + "factor", 1.f),
                    sensConf.f(chn + '_' + "midpoint", 0.f)
                };
            }
        }

        auto constexpr DEFAULT_FILT_LEN = 10;
        auto filtLen = (uint16_t) sensConf.getLong(chn + '_' + "filt_len", DEFAULT_FILT_LEN);

        params.emplace(chn, p{
                           .params = SensorParams{
                               .adcCh = chNum,
                               .transform = lt,
                               .calibrationConstraints = {},
                               .teleName = chn,
                               .unit = ' ',
                               .rawTelemetry = false,
                           },
                           .adc = adc,
                           .filtLen = filtLen,
                       });

        //ESP_LOGI("main", "Initialized ADC %s (V/I)(i/o)_ch = (%i %i %i %i) , exp.LoopRate=%hu", adcName.c_str(), Vin_ch,
        //         Iin_ch,
        //         Vout_ch, Iout_ch, loopRateMin);
    }

    //Vin_transform.factor *= sensConf.f("vin_calib", 1.f);
    //Vout_transform.factor *= sensConf.f("vout_calib", 1.f);

    params.find("vin")->second.params.calibrationConstraints = {lim.Vin_max, 1.8f, false};
    params.find("vin")->second.params.unit = 'V';
    params.find("vin")->second.params.rawTelemetry = true;
    sensors.Vin = adcSampler.addSensor(params.find("vin")->second.adc,
                                       params.find("vin")->second.params,
                                       lim.Vin_max,
                                       params.find("vin")->second.filtLen); // filtLen = 60


    {
        auto &iin(params.find("iin")->second);

        if (iin.params.adcCh != 255) {
            iin.params.calibrationConstraints = {lim.Iin_max * 0.05f, .1f, true};
            iin.params.unit = 'A';
            sensors.Iin = adcSampler.addSensor(
                iin.adc,
                iin.params,
                lim.Iin_max,
                iin.filtLen);
        } else {
            sensors.Iin = adcSampler.addVirtualSensor(
                [&]() {
                    // Sensor.reset() leaves last=NaN until the first post-reset sample arrives
                    // (calibration phase, ADC reInit, etc.). |NaN|<x is false, so without this
                    // guard the division below would propagate NaN into power/MPPT.
                    if (!std::isfinite(sensors.Iout->last) || !std::isfinite(sensors.Vin->last) ||
                        !std::isfinite(sensors.Vout->last))
                        return 0.f;
                    if (std::abs(sensors.Iout->last) < .01f or sensors.Vin->last < 0.1f)
                        return 0.f;
                    return sensors.Iout->last * sensors.Vout->last / sensors.Vin->last / conversionEfficiency;
                },
                iin.filtLen,
                iin.params.teleName.c_str(),
                iin.params.unit);
        }
    }


    {
        auto &iout(params.find("iout")->second);

        if (iout.params.adcCh != 255) {
            iout.params.calibrationConstraints = {lim.Iout_max * 0.05f, .1f, true};
            iout.params.unit = 'A';
            iout.params.rawTelemetry = true;
            sensors.Iout = adcSampler.addSensor(
                iout.adc,
                iout.params,
                lim.Iout_max,
                iout.filtLen);
        } else {
            sensors.Iout = adcSampler.addVirtualSensor(
                [&]() {
                    if (!std::isfinite(sensors.Iin->last) || !std::isfinite(sensors.Vin->last) ||
                        !std::isfinite(sensors.Vout->last))
                        return 0.f;
                    if (std::abs(sensors.Iin->last) < .05f or sensors.Vout->last < 0.1f)
                        return 0.f;
                    return sensors.Iin->last * sensors.Vin->last / sensors.Vout->last * conversionEfficiency;
                },
                iout.filtLen,
                iout.params.teleName.c_str(),
                iout.params.unit
            );
        }
    }

    {
        auto &ntc(params.find("ntc")->second);
        if (ntc.params.adcCh != 255) {
            ntc.params.calibrationConstraints = {2.8f, .1f, false};
            ntc.params.unit = 'C';
            auto sense = adcSampler.addSensor(ntc.adc, ntc.params, 2.8f, 50);
            mppt.ntc.setValueRef(sense->ewm.avg.get());
        }
    }


    {
        // notice that Vout should be the last sensor for lowest latency
        auto &vout(params.find("vout")->second);
        vout.params.calibrationConstraints = {lim.Vout_max, .7f, false};
        vout.params.unit = 'V';
        sensors.Vout = adcSampler.addSensor(vout.adc, vout.params, lim.Vout_max, vout.filtLen); // 60
    }

    // Inverter-ripple notch: adaptive auto-tunes to the measured 2x-line tone on Vout; with
    // notch_adaptive=0 the notch stays fixed at notch_freq (100 Hz = 50 Hz mains by default).
    adcSampler.configureNotch(
        sensConf.getByte("notch_adaptive", 1) != 0,
        sensConf.f("notch_freq", 100.f),
        sensConf.f("notch_q", 20.f),
        -30.f);
    adcSampler.setRippleSource(sensors.Vout);

    // Glitch-safe median: pass dense load pulses through (unbiased current) but still clip impulse
    // glitches. despike = outlier threshold (0 = off -> legacy unconditional median; ~8 to enable).
    adcSampler.configureDespike(sensConf.f("despike", 0.f));

    adcSampler.ignoreCalibrationConstraints = sensConf.getByte("ignore_calibration_constraints", 0);
    if (adcSampler.ignoreCalibrationConstraints)
        ESP_LOGW("main", "Skipping ADC range and noise checks.");
    heap_caps_check_integrity_all(true);
}
