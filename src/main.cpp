#include <Arduino.h>

#include "adc/adc.h"
#include "adc/ads.h"
#include "adc/adc_esp32.h"

#include "adc/sampling.h"
#include "buck.h"
#include "mppt.h"
#include "util.h"
#include "telemetry.h"

#include <Wire.h>
#include <hal/uart_types.h>

#include "pinconfig.h"
#include "version.h"
#include "lcd.h"

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


ADC_ADS adc_ads; // ADS1x15
ADC_ESP32 adc_esp32; // ESP32 internal ADC

ADC_Sampler adcSampler{}; // schedules async ADC reading
LCD lcd;
HalfBridgePwm pwm;

MpptController mppt{adcSampler, pwm, lcd};

VIinVout<const ADC_Sampler::Sensor *> sensors{};


bool disableWifi = false;

unsigned long lastLoopTime = 0, maxLoopLag = 0;
unsigned long lastTimeOut = 0;
uint32_t lastNSamples = 0;
unsigned long lastMpptUpdateNumSamples = 0;
bool manualPwm = false;
bool charging = false;


void uartInit(int port_num);

void setup() {
    Serial.begin(115200);

#if CONFIG_IDF_TARGET_ESP32S3 and !CONFIG_ESP_CONSOLE_UART_DEFAULT
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
#endif

    ESP_LOGI("main", "*** Fugu Firmware Version %s (" __DATE__ " " __TIME__ ")", FIRMWARE_VERSION);

    if (!Wire.begin((uint8_t) PinConfig::I2C_SDA, (uint8_t) PinConfig::I2C_SCL, 800000UL)) {
        ESP_LOGE("main", "Failed to initialize Wire");
    }

    if (!lcd.init()) {
        ESP_LOGE("main", "Failed to init LCD");
    }

    lcd.displayMessage("Fugu FW v" FIRMWARE_VERSION "\n" __DATE__ " " __TIME__, 2000);

#ifdef NO_WIFI
    disableWifi = true;
#endif

    if (!disableWifi) {
        connect_wifi_async();
        bool res = wait_for_wifi();
        lcd.displayMessage(
                res ? ("WiFi connected.\n" + std::string(WiFi.localIP().toString().c_str())) : "WiFi timeout.", 2000);
    }

    if (!mountLFS()) {
        ESP_LOGE("main", "Error mounting LittleFS partition!");
    }

    AsyncADC<float> *adc;
    uint8_t Vin_ch, Vout_ch, Iin_ch;
    int ewmaSpan;
    if (adc_ads.init()) {
        adc = &adc_ads;
        ewmaSpan = 8;
        Vin_ch = 3;
        Vout_ch = 1;
        Iin_ch = 2;
    } else if (adc_esp32.init()) {
        ESP_LOGW("main", "Failed to initialize external ADS1x15 ADC, using internal");
        adc = &adc_esp32;
        ewmaSpan = 80;
        Vin_ch = (uint8_t) PinConfig::ADC_Vin;
        Vout_ch = (uint8_t) PinConfig::ADC_Vout;
        Iin_ch = (uint8_t) PinConfig::ADC_Iin;
    } else {
        scan_i2c();
        ESP_LOGE("main", "Failed to initialize any ADC");
        while (1) {}
    }

    adcSampler.setADC(adc);

    constexpr float acs712_30_sensitivity = 1. / 0.066;

    LinearTransform Vin_transform = {(200 + FUGU_HV_DIV) / FUGU_HV_DIV, 0};
    LinearTransform Vout_transform = {(47. / 2 + 1) / 1, 0};
    LinearTransform Iin_transform = {
            -acs712_30_sensitivity * (10 + 3.3) / 10. * 1.03734586, //-20.9
            2.5 * 10. / (10 + 3.3) - 0.0117, // midpoint 1.88V
    };

    sensors.Vin = adcSampler.addSensor(
            {
                    Vin_ch,
                    Vin_transform,
                    {80, 0.1f, false},
                    "U_in_raw",
                    false},
            80.f);
    sensors.Iin = adcSampler.addSensor(
            {
                    Iin_ch,
                    Iin_transform,
                    {.5f, .1f, true},
                    "I_raw",
                    false},
            30.f);
    sensors.Vout = adcSampler.addSensor(
            {Vout_ch,
             Vout_transform,
             {60, .05f, false},
             "U_out_raw",
             true},
            60.f);

    adcSampler.begin(ewmaSpan);

    if (!pwm.init()) {
        ESP_LOGE("main", "Failed to init half bridge");
    }

    if (!disableWifi)
        adcSampler.onNewSample = dcdcDataChanged;

    mppt.setSensors(sensors);
    mppt.begin();

    ESP_LOGI("main", "setup() done.");
}

void loop() {
    auto nowMs = millis();

    if (!adcSampler.update())
        return; // no new data -> don't do anythings. this prevents unne

    mppt.ntc.read();

    // cap control update rate to sensor sampling rate (see below). rate for all 3 sensors are equal.
    // we choose Vout here because this is the most critical control value (react fast to prevent OV)
    auto nSamples = sensors.Vout->numSamples;


    // auto range current sense ADC channel TODO add hysteresis
    /* float adcRangeBound = 2.0f;
    adcSampler.adc->setMaxExpectedVoltage(
            sensors.Iin->adcCh,
            (std::max(sensors.Iin->last, sensors.Iin->ewm.avg.get()) < sensors.Iin->transform.apply(adcRangeBound))
            ? adcRangeBound
            : sensors.Iin->transform.apply_inverse(30.f)
    ); */


    if (unlikely(adcSampler.isCalibrating())) {
        pwm.disable();
    } else {
        if (charging) {
            bool mppt_ok = mppt.protect();
            if (mppt_ok) {
                if ((nSamples - lastMpptUpdateNumSamples) > 0) {
                    if (!manualPwm)
                        mppt.update();
                    else
                        mppt.telemetry();
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                charging = false;
            }
        } else if (mppt.startCondition()) {
            mppt.startSweep();
            charging = true;
        }
    }

    if ((nowMs - lastTimeOut) >= 3000) {
        auto sps = (lastNSamples < nSamples ? (nSamples - lastNSamples) : 0) * 1000u /
                   (uint32_t) (nowMs - lastTimeOut);

        if (sps < 100 && !pwm.disabled() && nSamples > 1000) { //(nowMs - adcSampler.getTimeLastCalibration()) > 6000)
            ESP_LOGE("main", "Loop latency too high! shutdown");
            pwm.disable();
            charging = false;
        }

        UART_LOG(
                "Vi/o=%4.1f/%4.1f Iin=%4.1fA Pin=%4.1fW %.0f°C %2usps %2ukbps PWM(H|L|Lm)=%4hu|%4hu|%4hu MPPT(st=%5s,%i) lag=%.1fms N=%u",
                sensors.Vin->last,
                sensors.Vout->last,
                sensors.Iin->last,
                sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
                //ewm.chIin.std.get() * 1000.f, σIin=%.2fm
                mppt.ntc.last(),
                sps,
                (uint32_t) (bytesSent /*/ 1000u * 1000u*/ / millis()),
                pwm.getBuckDutyCycle(), pwm.getBuckDutyCycleLS(), pwm.getDutyCycleLSMax(),
                //mppt.getPower()
                manualPwm ? "MANU"
                          : (!charging && !mppt.startCondition()
                             ? "START"
                             : MpptState2String[(uint8_t) mppt.getState()].c_str()),
                (int) charging,
                maxLoopLag * 1e-3f,
                nSamples
        );
        lastTimeOut = nowMs;
        lastNSamples = nSamples;

        if (!charging)
            mppt.meter.update(); // always update the meter
    }


    lcd.updateValues(LcdValues{
            .Vin = sensors.Vin->ewm.avg.get(),
            .Vout = sensors.Vout->ewm.avg.get(),
            .Iin = sensors.Iin->ewm.avg.get(),
            .Iout = mppt.getIoutSmooth(),
            .Temp = mppt.ntc.last(),
    });

    if (manualPwm) {
        pwm.pwmPerturb(0); // this will increase LS duty cycle if possible
        pwm.enableBackflowMosfet(true);
        // notice that mppt::protect() calls updateLowSideMaxDuty()
        delay(10);
    }

    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly

    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    char data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    length = uart_read_bytes(uart_num, data, 127, 0);
    if (length) {
        data[length] = 0;
        uart_write_bytes(uart_num, data, length);

        String inp(data);

        ESP_LOGI("main", "received serial command: %s", inp.c_str());
        inp.trim();
        if (inp.length() > 0) {
            if ((inp[0] == '+' or inp[0] == '-') && !adcSampler.isCalibrating() && inp.length() < 6 &&
                inp.toInt() != 0 &&
                std::abs(inp.toInt()) < pwm.pwmMax) {
                int pwmStep = inp.toInt();
                ESP_LOGI("main", "Manual PWM step %i", pwmStep);
                //manualPwm = true;
                pwm.pwmPerturb((int16_t) pwmStep);
            } else if (inp == "restart" or inp == "reset") {
                pwm.disable();
                Serial.println("Restart, delay 1s");
                delay(200);
                ESP.restart();
            } else if (inp == "mppt" && manualPwm) {
                ESP_LOGI("main", "MPPT re-enabled");
                manualPwm = false;
            } else if (inp.startsWith("dc ") && !adcSampler.isCalibrating() && inp.length() <= 8
                       && inp.substring(3).toInt() > 0 && inp.substring(3).toInt() < pwm.pwmMax) {
                manualPwm = true;
                pwm.pwmPerturb(inp.substring(3).toInt() - pwm.getBuckDutyCycle());
            } else if (inp.startsWith("speed ") && inp.length() <= 12) {
                float speedScale = inp.substring(6).toFloat();
                if (speedScale >= 0 && speedScale < 10) {
                    mppt.speedScale = speedScale;
                    ESP_LOGI("main", "Set tracker speed scale %.4f", speedScale);
                }
            } else if (inp.startsWith("fan ")) {
                fanSet(inp.substring(4).toFloat() * 0.01f);
            } else if (inp == "sweep") {
                mppt.startSweep();
            } else if (inp == "reset-lag") {
                maxLoopLag = 0;
            } else if (inp == "wifi on") {
                disableWifi = false;
                timeSynced = false;
                connect_wifi_async();
            } else if (inp == "wifi off") {
                WiFi.disconnect(true);
                disableWifi = true;
            } else if (inp == "scan-i2c") {
                scan_i2c();
            } else {
                ESP_LOGI("main", "unknown command");
            }
        }
    }

    if (!disableWifi) {
        /* only connect with disabled power conversion
         * ESP32's wifi can cause latency issues otherwise
         */
        wifiLoop(pwm.disabled());
    }
    auto now = micros();
    auto lag = now - lastLoopTime;
    if (lastLoopTime && lag > maxLoopLag && !pwm.disabled()) maxLoopLag = lag;
    lastLoopTime = now;
}
