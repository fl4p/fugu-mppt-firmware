#include "logging.h"

#include <hal/uart_types.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include <Arduino.h>
#include <Wire.h>
#include <USB.h>
#include <esp_private/usb_console.h>

#include "adc/adc.h"
#include "adc/ads.h"
//#include "adc/adc_esp32.h"
#include "adc/adc_esp32_cont.h"
#include "adc/fake.h"
#include "adc/ina226.h"
#include "adc/sampling.h"
#include "buck.h"
#include "mppt.h"
#include "util.h"
#include "telemetry.h"
#include "version.h"
#include "lcd.h"
#include "led.h"
#include "ota.h"

#include "perf.h"
#include <sprofiler.h>

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED

#include <hal/usb_serial_jtag_ll.h>

#endif

#include <esp_task_wdt.h>
#include <esp_pm.h>


ADC_Sampler adcSampler{}; // schedules async ADC reading
SynchronousBuck pwm;

LedIndicator led;
LCD lcd;

MpptController mppt{adcSampler, pwm, lcd};
VIinVout<const ADC_Sampler::Sensor *> sensors{};

bool disableWifi = false;

static unsigned long loopWallClockUs_ = 0;

unsigned long lastLoopTime = 0, maxLoopLag = 0, maxLoopDT = 0;
unsigned long lastTimeOutUs = 0;
uint32_t lastNSamples = 0;
unsigned long lastMpptUpdateNumSamples = 0;
bool manualPwm = false;
bool charging = false;
float conversionEfficiency;

uint16_t loopRateMin = 0;

const unsigned long IRAM_ATTR &loopWallClockUs() { return loopWallClockUs_; }

unsigned long IRAM_ATTR loopWallClockMs() { return (unsigned long) (loopWallClockUs_ / 1000ULL); }


void uartInit(int port_num);

void loopNetwork_task(void *arg);

void loopCore0_LF(void *arg);

void loopRT(void *arg);

void setupSensors(const ConfFile &pinConf, const Limits &lim) {
    loopWallClockUs_ = micros();

    /// compute voltage factor for a resistor divider network with 2 parallel R_low (Rh+(Rl+Ra))
    auto adcVDiv = [](float Rh, float Rl, float Ra) {
        auto rl = 1 / (1 / Rl + 1 / Ra);
        return LinearTransform{(Rh + rl) / rl, 0.f};
    };

    ConfFile sensConf{"/littlefs/conf/sensor.conf"};

    if (!sensConf) {
        throw std::runtime_error("no sensor conf");
    }

    AsyncADC<float> *adc;


    auto adcName = sensConf.getString("adc");
    if (adcName == "ads1115" or adcName == "ads1015") {
        adc = new ADC_ADS(adcName == "ads1115");
    } else if (adcName == "ina226") {
        adc = new ADC_INA226();
    } else if (adcName == "esp32adc1") {
        adc = new ADC_ESP32_Cont();
    } else if (adcName == "fake") {
        adc = new ADC_Fake();
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
    //Vin_transform.factor *= sensConf.f("vin_calib", 1.f);

    LinearTransform Vout_transform = adcVDiv(
            sensConf.f("vout_rh"),
            sensConf.getFloat("vout_rl"),
            adc->getInputImpedance(Vout_ch)
    );
    //Vout_transform.factor *= sensConf.f("vout_calib", 1.f);

    LinearTransform Iin_transform = {sensConf.f("iin_factor", 1.f), sensConf.f("iin_midpoint", 0.f)};
    LinearTransform Iout_transform{sensConf.f("iout_factor", 1.f), sensConf.f("iout_midpoint", 0.f)};


    if (!adc->init(pinConf)) {
        ESP_LOGE("main", "Failed to initialize ADC %s", adcName.c_str());
        scan_i2c();
        //while (1) {} // trap
        adcSampler.setADC(nullptr);
        return;
    } else {
        ESP_LOGI("main", "Initialized ADC %s (V/I)(i/o)_ch = (%i %i %i %i) , exp.LoopRate=%hu", adcName.c_str(), Vin_ch,
                 Iin_ch,
                 Vout_ch, Iout_ch, loopRateMin);
    }

    adcSampler.setADC(adc);

    uint16_t iinFiltLen = sensConf.getLong("iin_filt_len");
    uint16_t ioutFiltLen = sensConf.getLong("iout_filt_len");
    conversionEfficiency = sensConf.f("conversion_eff");
    assert(conversionEfficiency > 0.5f and conversionEfficiency < 1.0f);

    sensors.Vin = adcSampler.addSensor(
            {
                    Vin_ch,
                    Vin_transform,

                    {lim.Vin_max, 1.8f, false},
                    "U_in_raw",
                    true},
            lim.Vin_max, 60);


    sensors.Iin = (Iin_ch != 255)
                  ? adcSampler.addSensor(
                    {
                            Iin_ch,
                            Iin_transform,

                            {lim.Iin_max * 0.05f, .1f, true},
                            "Iid",
                            false},
                    lim.Iin_max, iinFiltLen)
                  : adcSampler.addVirtualSensor([&]() {
                if (std::abs(sensors.Iout->last) < .01f or sensors.Vin->last < 0.1f)
                    return 0.f;
                return sensors.Iout->last * sensors.Vout->last / sensors.Vin->last / conversionEfficiency;
            }, iinFiltLen);


    sensors.Iout = (Iout_ch != 255)
                   ? adcSampler.addSensor(
                    {
                            Iout_ch,
                            Iout_transform,
                            {lim.Iout_max * 0.05f, .1f, true},
                            "Io",
                            true},
                    lim.Iout_max, ioutFiltLen)
                   : adcSampler.addVirtualSensor([&]() {
                if (std::abs(sensors.Iin->last) < .05f or sensors.Vout->last < 0.1f)
                    return 0.f;
                return sensors.Iin->last * sensors.Vin->last / sensors.Vout->last * conversionEfficiency;
            }, ioutFiltLen);

    // note that Vout should be the last sensor for lowest latency
    sensors.Vout = adcSampler.addSensor(
            {Vout_ch,
             Vout_transform,
             {lim.Vout_max, .7f, false},
             "U_out_raw",
             true},
            lim.Vout_max, 60);

    adcSampler.ignoreCalibrationConstraints = sensConf.getByte("ignore_calibration_constraints", 0);

    if (adcSampler.ignoreCalibrationConstraints)
        ESP_LOGW("main", "Skipping ADC range and noise checks.");

    mppt.setSensors(sensors);
}



void setup() {
    bool setupErr = false;

    Serial.begin(115200);
    //ESP_ERROR_CHECK(esp_usb_console_init()); // using JTAG

    rtcount_test_cycle_counter();

#ifndef CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    //usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
    //        .tx_buffer_size = 1024,
    //        .rx_buffer_size = 1024,
    //};
    //ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
#endif

#if CONFIG_IDF_TARGET_ESP32S3 and !CONFIG_ESP_CONSOLE_UART_DEFAULT
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
#endif

    ESP_LOGI("main", "*** Fugu Firmware Version %s (" __DATE__ " " __TIME__ ")", FIRMWARE_VERSION);

    if (!mountLFS()) {
        ESP_LOGE("main", "Error mounting LittleFS partition!");
        setupErr = true;
    }

    ConfFile pprofConf{"/littlefs/conf/pprof.conf", true};

    auto sprofHz = (uint32_t) pprofConf.getLong("sprofiler_hz", 0);
    if (sprofHz && esp_cpu_dbgr_is_attached()) {
        // only start the profiler with OpenOCD attached?
        //ESP_LOGI("main", "starting sprofiler with freq %lu (samples/bank=%i)", sprofHz, PROFILING_ITEMS_PER_BANK);
        //sprofiler_initialize(sprofHz);
    } else if (sprofHz) {
        ESP_LOGW("main", "sprofiler configured but not debugger attached");
    }


    ConfFile pinConf{"/littlefs/conf/pins.conf"};

    auto mcuStr = pinConf.getString("mcu", "");
    if (mcuStr != CONFIG_IDF_TARGET) {
        ESP_LOGE("main", "pins.conf expects MCU %s, but target is %s", mcuStr.c_str(), CONFIG_IDF_TARGET);
        setupErr = true;
    }

    if (pinConf) {
        // TODO
        /*
         * [0;32m[I][i2c.arduino:183]: Performing I2C bus recovery[0m
[1;31m[E][i2c.arduino:199]: Recovery failed: SCL is held LOW on the I2C bus
         */
        //*?
        auto i2c_freq = pinConf.getLong("i2c_freq", 100000);
        auto i2c_sda = pinConf.getByte("i2c_sda", 255);
        bool noI2C = i2c_sda == 255;
        if (!noI2C) {
            ESP_LOGI("main", "i2c pins SDA=%hi SCL=%hi freq=%lu", i2c_sda, pinConf.getByte("i2c_scl"), i2c_freq);
            if (!Wire.begin(i2c_sda, (uint8_t) pinConf.getLong("i2c_scl"), i2c_freq)) {
                ESP_LOGE("main", "Failed to initialize Wire");
                setupErr = true;
            }
        } else {
            ESP_LOGI("main", "no i2c_sda pin set");
        }


        led.begin(pinConf);

        if (!noI2C) {
            if (!lcd.init()) {
                ESP_LOGE("main", "Failed to init LCD");
            } else {
                lcd.displayMessage("Fugu FW v" FIRMWARE_VERSION "\n" __DATE__ " " __TIME__, 2000);
            }
        }
    }

    Limits lim{};
    try {
        lim = Limits{ConfFile{"/littlefs/conf/limits.conf"}};
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error reading limits.conf: %s", er.what());
        setupErr = true;
    }


#ifdef NO_WIFI
    disableWifi = true;
#endif

    TeleConf teleConf{};

    if (!disableWifi) {
        connect_wifi_async();
        bool res = wait_for_wifi();
        led.setHexShort(res ? 0x565 : 0x200);
        lcd.displayMessage(
                res ? ("WiFi connected.\n" + std::string(WiFi.localIP().toString().c_str())) : "WiFi timeout.", 2000);

        teleConf = ConfFile{"/littlefs/conf/tele.conf"};
    }

    if (teleConf.influxdbHost) {
        ESP_LOGI("main", "Influxdb telemetry to host %s", teleConf.influxdbHost.toString().c_str());
        adcSampler.onNewSample = dcdcDataChanged;
    }

    try {
        setupSensors(pinConf, lim);
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error during sensor setup: %s", er.what());
        //if(adcSampler.adc) delete adcSampler.adc;
        adcSampler.setADC(nullptr);
        setupErr = true;
    }

    if (setupErr) {
        ESP_LOGE("main", "Error during setup, adc-only mode, skip pwm init");
    } else {
        if (!pwm.init()) {
            ESP_LOGE("main", "Failed to init half bridge");
        }
    }

    try {
        mppt.initSensors(pinConf);
        if (adcSampler.adc && !setupErr) {
            mppt.begin(pinConf, lim, teleConf);
        }
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error during mppt setup: %s", er.what());
    }

    xTaskCreatePinnedToCore(loopRT, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, 1);
    //xTaskCreatePinnedToCore(loopNetwork_task, "netloop", 4096 * 4, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loopCore0_LF, "core0LF", 4096 * 1, NULL, 1, NULL, 0);


    // this will defer all logs, if abort() is called during setup we might never see relevant messages
    // so calls this after everything else has been setup
    enable_esp_log_to_telnet();

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

void loopNewData(unsigned long nowMs);

void loopUart(unsigned long nowMs);

void loop() {
    loopNetwork_task(nullptr);
}

static esp_err_t disable_cpu_power_saving(void) {
    esp_err_t ret = ESP_OK;

    static esp_pm_lock_handle_t s_cli_pm_lock = NULL;


    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "nopowersave", &s_cli_pm_lock);
    if (ret == ESP_OK) {
        esp_pm_lock_acquire(s_cli_pm_lock);
        ESP_LOGI("main", "Successfully created CLI pm lock");
    } else {
        if (s_cli_pm_lock != NULL) {
            esp_pm_lock_delete(s_cli_pm_lock);
            s_cli_pm_lock = NULL;
        }
        ESP_LOGI("main", " Failed to create CLI pm lock");
    }
    return ret;
}

void loopRT(void *arg) {
#define RT_CORE 1

#if CONFIG_ARDUINO_RUNNING_CORE == RT_CORE or CONFIG_ARDUINO_EVENT_RUNNING_CORE == RT_CORE or \
    CONFIG_ARDUINO_UDP_RUNNING_CORE == RT_CORE or CONFIG_ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE == RT_CORE
#error "arduino runtime is configured to run on RT_CORE"
#endif

# if CONFIG_ESP_TIMER_ISR_AFFINITY != RT_CORE or CONFIG_ESP_TIMER_TASK_AFFINITY == RT_CORE or CONFIG_LWIP_TCPIP_TASK_AFFINITY == RT_CORE \
 or CONFIG_PTHREAD_TASK_CORE_DEFAULT == RT_CORE or CONFIG_FMB_PORT_TASK_AFFINITY == RT_CORE or CONFIG_MDNS_TASK_AFFINITY == RT_CORE
#error "esp runtime is configured to run on RT_CORE"
#endif

    try {
        adcSampler.begin();
    } catch (const std::runtime_error &er) {
        ESP_LOGE("main", "error starting ADC sampler: %s", er.what());
        while (true) { vTaskDelay(10); }
    }

    disable_cpu_power_saving();

    assert(xPortGetCoreID() == 1);
    vTaskPrioritySet(NULL, 20); // highest priority (24)
    // ^ see https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/api-guides/performance/speed.html#choosing-task-priorities-of-the-application

    ESP_LOGI("main", "Loop running on core %i", (int) xPortGetCoreID());

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    ESP_LOGW("main", "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS enabled!");
            delay(1000);
#endif

    while (true) {
        rtcount("start");
        loopWallClockUs_ = micros();
        rtcount("micros");
        auto &nowUs(loopWallClockUs_);

        auto nowMs = (unsigned long) (nowUs / 1000ULL);


        rtcount("adc.update.pre");
        auto samplerRet = adcSampler.update();
        rtcount("adc.update");

        if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure) {
            mppt.shutdownDcdc();
            charging = false;
            delayStartUntil = nowMs + 4000;
        }

        if (samplerRet != ADC_Sampler::UpdateRet::NewData) {
            if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
                ESP_LOGW("main", "Board power supply UV %.2f!", mppt.boardPowerSupplyVoltage());
                adcSampler.cancelCalibration();
            }

            if (timeLastSampler && nowMs - timeLastSampler > 200) {
                if (!pwm.disabled()) {
                    pwm.disable();
                    charging = false;
                    ESP_LOGE("main", "Timeout waiting for new ADC sample, shutdown! numSamples %lu",
                             lastMpptUpdateNumSamples);
                }
            }

            if (!timeLastSampler and nowMs > 20000) {
                pwm.disable();
                ESP_LOGE("main", "Never got a sample! Please check ADC");
                if (nowMs > (1000 * 60 * 15)) {
                    ESP_LOGW("main", "Rebooting");
                    ESP.restart();
                }

                delay(5000);
            }
        } else {
            loopNewData(nowMs);
            rtcount("loopNewData");
        }


        auto now2 = micros();
        auto lag = now2 - lastLoopTime;
        if (lastLoopTime && lag > maxLoopLag && !pwm.disabled()) maxLoopLag = lag;
        lastLoopTime = now2;

        auto loopDT = now2 - nowUs;
        if (loopDT > maxLoopDT && !pwm.disabled())
            maxLoopDT = loopDT;

        // Not need to yield or call vTaskDelay here
        // waiting for adc values does the necessary blocking

        //vTaskDelay(0); // this resets the Watchdog Timer (WDT) for some reason
        //vTaskDelay(1);
        //yield();
    }
}

static bool usbConnected = false;

std::string mpptStateStr() {
    std::string arrow;
    if (mppt.getState() == MpptControlMode::MPPT) {
        if (mppt.tracker.slowMode) {
            arrow = mppt.tracker._direction ? "⇡" : "⇣";
        } else {
            arrow = mppt.tracker._direction ? "↑" : "↓";
        }
    }
    return arrow + MpptState2String[(uint8_t) mppt.getState()];
}

int console_write_usb(const char *buf, size_t len);

void loopLF(const uint32_t &nSamples, const unsigned long &nowUs) {
    uint32_t sps = (uint64_t) (nSamples - lastNSamples) * 1000000llu / (uint64_t) (nowUs - lastTimeOutUs);

    if (sps < loopRateMin && !pwm.disabled() && nSamples > max(loopRateMin * 5, 200) &&
        !manualPwm && lastTimeOutUs && (nowUs - adcSampler.getTimeLastCalibrationUs()) > 6000000) {
        mppt.shutdownDcdc();
        auto loopRunTime = (nowUs - adcSampler.getTimeLastCalibrationUs());
        ESP_LOGE("main", "Loop latency too high (%lu < %hu Hz), shutdown! (nSamples=%lu, D=%u, loopRunTime=%.1fs )",
                 sps, loopRateMin, nSamples, pwm.getBuckOnPwmCnt(), loopRunTime * 1e-6f);
        charging = false;
    }

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    usbConnected = usb_serial_jtag_is_connected();
#endif

    mppt.ntc.read();

    UART_LOG(
            "V=%5.2f/%5.2f I=%4.1f/%5.2fA %5.1fW %.0f℃%.0f℃ %2lusps %2lu㎅/s PWM(H|L|Lm)=%4hu|%4hu|%4hu"
            " st=%5s,%i lag=%lu㎲ lt=%lu㎲ N=%lu rssi=%hi",
            sensors.Vin->last,
            sensors.Vout->last,
            sensors.Iin->ewm.avg.get(), // sensors.Iin->last,
            sensors.Iout->ewm.avg.get(),
            sensors.Vin->ewm.avg.get() * sensors.Iin->ewm.avg.get(),
            //ewm.chIin.std.get() * 1000.f, σIin=%.2fm
            mppt.ntc.last(), mppt.ucTemp.last(),
            sps,
            (uint32_t) (bytesSent * 1000 / (nowUs - lastTimeOutUs)),
            pwm.getBuckOnPwmCnt(), pwm.getBuckDutyCycleLS(), pwm.getDutyCycleLSMax(),
            //mppt.getPower()
            manualPwm ? "MANU"
                      : (!charging && !mppt.startCondition()
                         ? (mppt.boardPowerSupplyUnderVoltage() ? "UV" : "START")
                         : mpptStateStr().c_str()),
            (int) charging,
            maxLoopLag,
            maxLoopDT,
            nSamples,
            WiFi.RSSI()
    );
    lastNSamples = nSamples;
    bytesSent = 0;

    if (!charging)
        mppt.meter.update(); // always update the meter

    if (manualPwm) {
        uint8_t i = constrain((sensors.Vout->last * sensors.Iout->last) / mppt.limits.P_max * 255, 1, 255);
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

void loopNewData(unsigned long nowMs) {
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
            rtcount("protect.pre");
            bool mppt_ok = mppt.protect(manualPwm);
            rtcount("protect");
            if (mppt_ok) {
                if (haveNewSample) {
                    if (!manualPwm) {
                        rtcount("mppt.update.pre");
                        mppt.update();
                        rtcount("mppt.update");
                    } else {
                        mppt.telemetry();
                        rtcount("mppt.telemetry");
                    }
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                charging = false;
                delayStartUntil = nowMs + 2000;
            }
        } else if (nowMs > delayStartUntil && mppt.startCondition()) {
            if (!manualPwm) {
                rtcount("mppt.startSweep.pre");
                mppt.startSweep();
                rtcount("mppt.startSweep");
            }
            charging = true;
        }
    }


    if (manualPwm) {
        if (!pwm.disabled())
            pwm.pwmPerturb(0); // this will increase LS duty cycle if possible
        //mppt.bflow.enable(true);
        // notice that mppt::protect() calls updateLowSideMaxDuty()
        // delay(1); // why?
    }
}


void loopConsole(int read(char *buf, size_t len), int write(const char *buf, size_t len), unsigned long nowMs) {
    constexpr uint8_t bufSiz = 128;
    static char buf[bufSiz];
    static uint8_t buf_pos = 0;

    int length = read(&buf[buf_pos], 128 - buf_pos);
    if (length > 0) {
        if (buf_pos == 0) write("> ", 2);
        if (length + buf_pos >= bufSiz - 1)
            length = bufSiz - 1 - buf_pos;
        write(&buf[buf_pos], length); // echo
        lastTimeOutUs = loopWallClockUs(); // stop logging during user input
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
        } else if (buf_pos == bufSiz - 1) {
            buf[buf_pos] = 0;
            ESP_LOGW("main", "discarding command buffer %s", buf);
            buf_pos = 0;
        }
    }
}

int uartRead(char *buf, size_t len) {
    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    if (length == 0) return 0;
    length = uart_read_bytes(uart_num, buf, len, 0);
    return length;
}

int uartWrite(const char *buf, size_t len) {
    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    return uart_write_bytes(uart_num, buf, len);
}


#include "../vfs/private_include/esp_vfs_private.h"


int console_read_usb(char *buf, size_t len) {
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    return esp_vfs_usb_serial_jtag_get_vfs()->read(0, buf, len);
#else
    return 0;
#endif
}


int console_write_usb(const char *buf, size_t len) {
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    auto r = esp_vfs_usb_serial_jtag_get_vfs()->write(0, buf, len);
    usb_serial_jtag_ll_txfifo_flush();
    return r;
#else
    return 0;
#endif
}

void loopUart(unsigned long nowMs) {
    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly

    loopConsole(uartRead, uartWrite, nowMs);


    if (usbConnected) {
        //loopConsole(esp_usb_console_read_buf, esp_usb_console_write_buf, nowMs);
        loopConsole(console_read_usb, console_write_usb, nowMs);
    }
}


void loopNetwork_task(void *arg) {
    //ESP_LOGI("main", "Net loop running on core %i", xPortGetCoreID());
    assert(xPortGetCoreID() == 0);

    auto nowMs(loopWallClockMs());

    loopUart(nowMs);
    flush_async_uart_log();
    process_queued_tasks();

    if (!disableWifi) {
        /* only connect with disabled power conversion
         * ESP32's wifi can cause latency issues otherwise
         */
        wifiLoop(pwm.disabled());
        ftpUpdate();
        telemetryFlushPointsQ();
    }

    auto &nSamples(sensors.Vout->numSamples);

    if ((loopWallClockUs() - lastTimeOutUs) >= 3000000) {
        loopLF(nSamples, loopWallClockUs());
        lastTimeOutUs = loopWallClockUs();
    }

    if (lcd && adcSampler.adc)
        lcd.updateValues(LcdValues{
                .Vin = sensors.Vin->ewm.avg.get(),
                .Vout = sensors.Vout->ewm.avg.get(),
                .Iin = sensors.Iin->ewm.avg.get(),
                .Iout = sensors.Iout->ewm.avg.get(),
                .Temp = mppt.ntc.last(),
        });

    vTaskDelay(pdMS_TO_TICKS(2));
}

void loopCore0_LF(void *arg) {
    // do everything with poor real-time performance

    while (1) {

        mppt.ucTemp.read();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool handleCommand(const String &inp) {
    ESP_LOGI("main", "received serial command: '%s'", inp.c_str());


    if ((inp[0] == '+' or inp[0] == '-') && !adcSampler.isCalibrating() && inp.length() < 6 &&
        inp.toInt() != 0 && std::abs(inp.toInt()) < pwm.pwmMaxHS) {
        int pwmStep = inp.toInt();
        //manualPwm = true; // don't switch to manual pwm here!
        pwm.pwmPerturb((int16_t) pwmStep);
        ESP_LOGI("main", "Manual PWM step %i -> %i", pwmStep, (int) pwm.getBuckOnPwmCnt());
    } else if (manualPwm && (inp == "ls-disable" or inp == "ls-enable")) {
        pwm.enableLowSide(inp == "ls-enable");
    } else if (manualPwm && (inp == "bf-disable" or inp == "bf-enable")) {
        auto newState = inp == "bf-enable";
        if (mppt.bflow.state() != newState)
            ESP_LOGI("main", "Set bflow state %i", newState);
        mppt.bflow.enable(newState);
    } else if (inp == "restart" or inp == "reset" or inp == "reboot") {
        pwm.disable();
        Serial.println("Restart, delay 200ms");
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
        pwm.pwmPerturb(inp.substring(3).toInt() - pwm.getBuckOnPwmCnt());
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
        maxLoopDT = 0;
        rtcount_print(true);
    } else if (inp == "wifi on") {
        disableWifi = false;
        timeSynced = false;
        connect_wifi_async();
    } else if (inp == "wifi off") {
        WiFi.disconnect(true);
        disableWifi = true;
    } else if (inp.startsWith("wifi-add ")) {
        auto ssidAndPw = inp.substring(9);
        auto i = ssidAndPw.indexOf(':');
        if (i > 0) {
            std::string ssid = ssidAndPw.substring(0, i).c_str();
            auto psk = ssidAndPw.substring(i + 1);
            ESP_LOGI("main", "adding wifi network %s (psk=%s)", ssid.c_str(), psk.c_str());
            add_ap(ssid, psk.c_str());
        }

    } else if (inp == "scan-i2c") {
        scan_i2c();
    } else if (inp == "ls") {
        //list_files();
        ESP_LOGE("main", "not impl");
        return false;
    } else if (inp.startsWith("ota ")) {
        auto url = inp.substring(4);
        doOta(url);
        return true;
    } else if (inp == "rt-stats") {
        xTaskCreatePinnedToCore(print_real_time_stats_1s_task, "rtstats", 4096, NULL, 1, NULL, 0);
    } else if (inp == "mem") {
        UART_LOG("Total heap:  %9ld\n", ESP.getHeapSize());
        UART_LOG("Free heap:   %9ld\n", ESP.getFreeHeap());
        UART_LOG("Total PSRAM: %9ld\n", ESP.getPsramSize());
        UART_LOG("Free PSRAM:  %9ld\n", ESP.getFreePsram());
    } else {
        ESP_LOGI("main", "unknown or unexpected command");
        return false;
    }

    ESP_LOGI("main", "OK: %s", inp.c_str());

    return true;
}

void esp_task_wdt_isr_user_handler() {
    //throw std::runtime_error("reboot");
    enqueue_task([] {
        pwm.disable();
        ESP_LOGE("main", "Restart after WDT trigger");
        vTaskDelay(1000);
        ESP.restart();
    });
}