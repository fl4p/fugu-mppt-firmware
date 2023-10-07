#include <Arduino.h>

#include "adc/adc.h"
#include "adc/ads.h"
#include "adc/adc_esp32.h"
#include "adc/ina226.h"

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
#include "led.h"

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

ADC_Sampler adcSampler{}; // schedules async ADC reading
SynchronousBuck pwm;

LedIndicator led;
LCD lcd;

MpptController mppt{adcSampler, pwm, lcd};
VIinVout<const ADC_Sampler::Sensor *> sensors{};

bool disableWifi = false;

unsigned long lastLoopTime = 0, maxLoopLag = 0;
unsigned long lastTimeOut = 0;
uint32_t lastNSamples = 0;
unsigned long lastMpptUpdateNumSamples = 0;
bool manualPwm = false;
bool charging = false;

uint8_t loopRateMin = 0;


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

    if (!mountLFS()) {
        ESP_LOGE("main", "Error mounting LittleFS partition!");
    }

    ConfFile pinConf{"conf/pins"};

    if (!Wire.begin(
            (uint8_t) pinConf.getLong("i2c_sda"),
            (uint8_t) pinConf.getLong("i2c_scl"),
            pinConf.getLong("i2c_freq", 800000UL)
    )) {
        ESP_LOGE("main", "Failed to initialize Wire");
    }

    led.begin();
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
        led.setHexShort(res ? 0x565 : 0x200);
        lcd.displayMessage(
                res ? ("WiFi connected.\n" + std::string(WiFi.localIP().toString().c_str())) : "WiFi timeout.", 2000);
    }

    auto adcVDiv = [](float rH, float rL, float rA) {
        auto rl = 1 / (1 / rL + 1 / rA);
        return LinearTransform{(rH + rl) / rl, 0.f};
    };

    ConfFile sensConf{"conf/sensors"};

    AsyncADC<float> *adc;


    auto adcName = sensConf.getString("adc");
    if (adcName == "ads1115" or adcName == "ads1015") {
        adc = new ADC_ADS(adcName == "ads1115");
    } else if (adcName == "ina226") {
        adc = new ADC_INA226();
    } else {
        ESP_LOGE("main", "Unknown ADC %s", adcName.c_str());
        assert(false);
    }

    loopRateMin = sensConf.getByte("expected_hz", 0);

    auto Vin_ch = sensConf.getByte("vin_ch");
    auto Vout_ch = sensConf.getByte("vout_ch");
    auto Iin_ch = sensConf.getByte("iin_ch", 255);
    auto Iout_ch = sensConf.getByte("iout_ch", 255);


    LinearTransform Vin_transform = adcVDiv(
            sensConf.f("vin_rh"),
            sensConf.getFloat("vin_rl"),
            adc->getInputImpedance(Vin_ch)
    );
    Vin_transform.factor *= sensConf.f("vin_calib");

    LinearTransform Vout_transform = adcVDiv(
            sensConf.f("vout_rh"),
            sensConf.getFloat("vout_rl"),
            adc->getInputImpedance(Vout_ch)
    );
    Vout_transform.factor *= sensConf.f("vout_calib");

    LinearTransform Iin_transform = {1, 0};
    if (Iin_ch != 255)
        Iin_transform = {sensConf.f("iin_factor"), sensConf.f("iin_midpoint")};


    LinearTransform Iout_transform{1, 0};

    if (!adc->init()) {
        ESP_LOGE("main", "Failed to initialize ADC %s", adcName.c_str());
        scan_i2c();
        while (1) {} // trap
    }

    adcSampler.setADC(adc);

    uint16_t iinFiltLen = sensConf.getLong("iin_filt_len");
    uint16_t ioutFiltLen = sensConf.getLong("iout_filt_len");
    float conversionEfficiency = sensConf.f("conversion_eff");

    sensors.Vin = adcSampler.addSensor(
            {
                    Vin_ch,
                    Vin_transform,

                    {80, 1.8f, false},
                    "U_in_raw",
                    true},
            80.f, 60);


    sensors.Iin = (Iin_ch != 255)
                  ? adcSampler.addSensor(
                    {
                            Iin_ch,
                            Iin_transform,

                            {.5f, .1f, true},
                            "I_raw",
                            false},
                    30.f, iinFiltLen)
                  : adcSampler.addVirtualSensor([&]() {
                if (std::abs(sensors.Iout->last) < .05f or sensors.Vin->last < 0.1f)
                    return 0.f;
                return sensors.Iout->last * sensors.Vout->last / sensors.Vin->last / conversionEfficiency;
            }, iinFiltLen);


    sensors.Iout = (Iout_ch != 255)
                   ? adcSampler.addSensor(
                    {
                            Iout_ch,
                            Iout_transform,
                            {2.0f, .1f, true},
                            "Io",
                            true},
                    30.f, ioutFiltLen)
                   : adcSampler.addVirtualSensor([&]() {
                if (std::abs(sensors.Iin->last) < .05f or sensors.Vout->last < 0.1f)
                    return 0.f;
                return sensors.Iin->last * sensors.Vin->last / sensors.Vout->last * conversionEfficiency;
            }, ioutFiltLen);

    // note that Vout should be the last sensor for lowest latency
    sensors.Vout = adcSampler.addSensor(
            {Vout_ch,
             Vout_transform,
             {60, .7f, false},
             "U_out_raw",
             true},
            60.f, 60);

    adcSampler.begin();

    if (!pwm.init()) {
        ESP_LOGE("main", "Failed to init half bridge");
    }

    if (!disableWifi) adcSampler.onNewSample = dcdcDataChanged;

    mppt.setSensors(sensors);
    mppt.begin();

    ESP_LOGI("main", "setup() done.");

    /*manualPwm = true;
    adcSampler.cancelCalibration();
    pwm.pwmPerturb(1327); // this can destroy the BF switch
    pwm.enableLowSide(true);
    mppt.bflow.enable(true); */
}

unsigned long timeLastSampler = 0;

unsigned long delayStartUntil = 0;

bool handleCommand(const String &inp);

void loop() {
    auto nowMs = millis();

    flush_async_uart_log();

    mppt.ntc.read();

    if (!adcSampler.update()) {
        if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
            ESP_LOGW("main", "Board power supply UV!");
            adcSampler.cancelCalibration();
        }

        if (timeLastSampler && nowMs - timeLastSampler > 200 && !pwm.disabled()) {
            pwm.disable();
            ESP_LOGE("main", "Timeout waiting for new ADC sample, shutdown!");
        }

        if (!timeLastSampler and nowMs > 20000) {
            pwm.disable();
            ESP_LOGE("main", "Never got a sample! Please check ADC");
            delay(5000);
        }

        yield();
        return; // no new data -> don't do anythings. this prevents unnecessary cpu time
    }

    // cap control update rate to sensor sampling rate (see below). rate for all 3 sensors are equal.
    // we choose Vout here because this is the most critical control value (react fast to prevent OV)
    auto nSamples = sensors.Vout->numSamples;

    bool haveNewSample = (nSamples - lastMpptUpdateNumSamples) > 0;

    if (haveNewSample)
        timeLastSampler = nowMs;

    // auto range current sense ADC channel TODO add hysteresis
    /* float adcRangeBound = 2.0f;
    adcSampler.adc->setMaxExpectedVoltage(
            sensors.Iin->adcCh,
            (std::max(sensors.Iin->last, sensors.Iin->ewm.avg.get()) < sensors.Iin->transform.apply(adcRangeBound))
            ? adcRangeBound
            : sensors.Iin->transform.apply_inverse(30.f)
    ); */


    if (unlikely(adcSampler.isCalibrating())) {
        mppt.shutdownDcdc();
    } else {
        if (charging or manualPwm) {
            bool mppt_ok = mppt.protect();
            if (mppt_ok) {
                if (haveNewSample) {
                    if (!manualPwm)
                        mppt.update();
                    else
                        mppt.telemetry();
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                charging = false;
                delayStartUntil = nowMs + 2000;
            }
        } else if (nowMs > delayStartUntil && mppt.startCondition()) {
            if (!manualPwm) mppt.startSweep();
            charging = true;
        }
    }

    if ((nowMs - lastTimeOut) >= 3000) {
        auto sps = (lastNSamples < nSamples ? (nSamples - lastNSamples) : 0) * 1000u /
                   (uint32_t) (nowMs - lastTimeOut);

        if (sps < loopRateMin && !pwm.disabled() && nSamples > 1000 &&
            !manualPwm) { //(nowMs - adcSampler.getTimeLastCalibration()) > 6000)
            ESP_LOGE("main", "Loop latency too high (%i < %hhu Hz), shutdown!", sps, loopRateMin);
            mppt.shutdownDcdc();
            charging = false;
        }

        UART_LOG(
                "Vi/o=%5.2f/%5.2f Ii/o=%4.1f/%4.1fA Pin=%5.1fW %.0f°C %2usps %2ukbps PWM(H|L|Lm)=%4hu|%4hu|%4hu MPPT(st=%5s,%i) lag=%.1fms N=%u",
                sensors.Vin->last,
                sensors.Vout->last,
                sensors.Iin->last,
                sensors.Iout->last,
                sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
                //ewm.chIin.std.get() * 1000.f, σIin=%.2fm
                mppt.ntc.last(),
                sps,
                (uint32_t) (bytesSent /*/ 1000u * 1000u*/ / millis()),
                pwm.getBuckDutyCycle(), pwm.getBuckDutyCycleLS(), pwm.getDutyCycleLSMax(),
                //mppt.getPower()
                manualPwm ? "MANU"
                          : (!charging && !mppt.startCondition()
                             ? (mppt.boardPowerSupplyUnderVoltage() ? "UV" : "START")
                             : MpptState2String[(uint8_t) mppt.getState()].c_str()),
                (int) charging,
                maxLoopLag * 1e-3f,
                nSamples
        );
        lastTimeOut = nowMs;
        lastNSamples = nSamples;

        if (!charging)
            mppt.meter.update(); // always update the meter

        if (manualPwm) {
            uint8_t i = constrain((sensors.Vout->last * sensors.Iout->last) / mppt.params.P_max * 255, 1, 255);
            led.setRGB(0, i, i);
        } else if (!charging) {
            led.setHexShort(sensors.Vout->last > sensors.Vin->last ? 0x100 : 0x300);
        } else {
            switch (mppt.getState()) {
                case MpptControlMode::Sweep:
                    led.setHexShort(0x303); // purple
                    break;
                case MpptControlMode::MPPT:
                    if (sensors.Iout->ewm.avg.get() > 0.2f)
                        led.setHexShort(0x230);
                    else
                        led.setHexShort(0x111);
                    break;
                case MpptControlMode::CV:
                    led.setHexShort(0x033);
                    break;
                default:
                    // CV/CC/CP, topping
                    led.setHexShort(0x310);
                    break;
            }
        }

    }


    lcd.updateValues(LcdValues{
            .Vin = sensors.Vin->ewm.avg.get(),
            .Vout = sensors.Vout->ewm.avg.get(),
            .Iin = sensors.Iin->ewm.avg.get(),
            .Iout = sensors.Iout->ewm.avg.get(),
            .Temp = mppt.ntc.last(),
    });

    if (manualPwm) {
        if (!pwm.disabled())
            pwm.pwmPerturb(0); // this will increase LS duty cycle if possible
        //mppt.bflow.enable(true);
        // notice that mppt::protect() calls updateLowSideMaxDuty()
        // delay(1); // why?
    }

    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly

    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    static char buf[128];
    static uint8_t buf_pos = 0;
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    length = uart_read_bytes(uart_num, &buf[buf_pos], 128 - buf_pos, 0);
    if (length) {
        if (buf_pos == 0) uart_write_bytes(uart_num, "> ", 2);
        uart_write_bytes(uart_num, &buf[buf_pos], length); // echo
        lastTimeOut = nowMs; // stop logging during user input
        buf_pos += length;
        while (buf_pos > 0 && buf[buf_pos - 1] == '\b') {
            --buf_pos;
            buf[buf_pos] = 0;
        }
        if (buf[buf_pos - 1] == '\r' or buf[buf_pos - 1] == '\n') {
            buf[buf_pos] = 0;
            String inp(buf);
            inp.trim();
            if (inp.length() > 0)
                handleCommand(inp);
            buf_pos = 0;
        } else if (buf_pos == 128) {
            buf[buf_pos] = 0;
            ESP_LOGW("main", "discarding command buffer %s", buf);
            buf_pos = 0;
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

    //yield();
    //esp_task_wdt_reset();
    vTaskDelay(1); // this resets the Watchdog Timer (WDT) for some reason
}


bool handleCommand(const String &inp) {
    ESP_LOGI("main", "received serial command: '%s'", inp.c_str());


    if ((inp[0] == '+' or inp[0] == '-') && !adcSampler.isCalibrating() && inp.length() < 6 &&
        inp.toInt() != 0 && std::abs(inp.toInt()) < pwm.pwmMaxHS) {
        int pwmStep = inp.toInt();
        //manualPwm = true; // don't switch to manual pwm here!
        pwm.pwmPerturb((int16_t) pwmStep);
        ESP_LOGI("main", "Manual PWM step %i -> %i", pwmStep, (int) pwm.getBuckDutyCycle());
    } else if (manualPwm && (inp == "ls-disable" or inp == "ls-enable")) {
        pwm.enableLowSide(inp == "ls-enable");
    } else if (manualPwm && (inp == "bf-disable" or inp == "bf-enable")) {
        auto newState = inp == "bf-enable";
        if (mppt.bflow.state() != newState)
            ESP_LOGI("main", "Set bflow state %i", newState);
        mppt.bflow.enable(newState);
    } else if (inp == "restart" or inp == "reset") {
        pwm.disable();
        Serial.println("Restart, delay 1s");
        delay(200);
        ESP.restart();
    } else if (inp == "mppt" && manualPwm) {
        ESP_LOGI("main", "MPPT re-enabled");
        manualPwm = false;
    } else if (inp.startsWith("dc ") && !adcSampler.isCalibrating() && inp.length() <= 8
               && inp.substring(3).toInt() > 0 && inp.substring(3).toInt() < pwm.pwmMaxHS) {
        if (!manualPwm)
            ESP_LOGI("main", "Switched to manual PWM");
        manualPwm = true;
        pwm.pwmPerturb(inp.substring(3).toInt() - pwm.getBuckDutyCycle());
        // pwm.enableLowSide(true);
    } else if (inp.startsWith("speed ") && inp.length() <= 12) {
        float speedScale = inp.substring(6).toFloat();
        if (speedScale >= 0 && speedScale < 10) {
            mppt.speedScale = speedScale;
            ESP_LOGI("main", "Set tracker speed scale %.4f", speedScale);
        }
    } else if (inp.startsWith("fan ")) {
        fanSet(inp.substring(4).toFloat() * 0.01f);
    } else if (inp.startsWith("led ")) {
        led.setRGB(inp.substring(4).c_str());
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
    } else if (inp.startsWith("wifi-set ")) {
        //inp.substring(9)
    } else if (inp == "scan-i2c") {
        scan_i2c();
    } else {
        ESP_LOGI("main", "unknown or unexpected command");
        return false;
    }

    ESP_LOGI("main", "OK: %s", inp.c_str());

    return true;
}