#include <Arduino.h>

#include "adc.h"
#include "ads.h"
#include "adc_esp32.h"

#include "sampling.h"
#include "pwm.h"
#include "mppt.h"
#include "util.h"
#include "telemetry.h"

#include <Wire.h>
#include <hal/uart_types.h>

#include "pinconfig.h"
#include "version.h"
#include "lcd.h"

//LiquidCrystal_I2C lcd(0x27,16,2);   //SYSTEM PARAMETER  - Configure LCD RowCol Size and I2C Address
//Adafruit_ADS1115 ads;             //SYSTEM PARAMETER  - ADS1115 ADC Library (By: Adafruit) Kindly uncomment this if you are using ADS1115
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
//TaskHandle_t Core2;    //SYSTEM PARAMETER  - Used for the ESP32 dual core operation


//#define backflow_MOSFET 27  //SYSTEM PARAMETER - Backflow MOSFET
#define LED 2               //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN 16              //SYSTEM PARAMETER - Fan GPIO Pin
//#define ADC_ALERT 34        //SYSTEM PARAMETER - Fan GPIO Pin
#define TempSensor 35       //SYSTEM PARAMETER - Temperature Sensor GPIO Pin


//#if CONFIG_IDF_TARGET_ESP32S3
ADC_ADS adc_ads;
ADC_ESP32 adc_esp32;


DCDC_PowerSampler dcdcPwr{ThreeChannelUnion<ChannelAndFactor>{.s={
        .chVin = {3, (200 + FUGU_HV_DIV) / FUGU_HV_DIV, 0},
        .chVout = {1, (47. / 2 + 1) / 1, 0},
        .chIin = {2, -(1 / 0.066f) * (10 + 3.3) / 10. * (14.6f / 13.1f) * (12.1f / 13.f), //ACS712-30 sensitivity)
                  2.5 * 10. / (10 + 3.3) - 0.0117, // midpoint
        },
}}};

HalfBridgePwm pwm;
MpptSampler mppt{dcdcPwr, pwm};

LCD lcd;


bool disableWifi = false;


void uartInit(int port_num);

void setup() {
    Serial.begin(115200);
#if CONFIG_IDF_TARGET_ESP32S3
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
#endif

    ESP_LOGI("main", "*** Fugu Firmware Version %s (" __DATE__ " " __TIME__ ")", FIRMWARE_VERSION);


    if (!Wire.begin((uint8_t) PinConfig::I2C_SDA, (uint8_t) PinConfig::I2C_SCL, 400000UL)) {
        ESP_LOGE("main", "Failed to initialize Wire");
    }

    if (!lcd.init()) {
        ESP_LOGE("main", "Failed to init LCD");
    }

    lcd.displayMessage("Fugu FW v" FIRMWARE_VERSION, 2000);

#ifdef NO_WIFI
    disableWifi = true;
#endif
    if (!disableWifi)
        connect_wifi_async();

    AsyncADC<float> *adc = nullptr;
    if(adc_ads.init() ) {
        adc = &adc_ads;
    } else if(adc_esp32.init()){
        ESP_LOGW("main", "Failed to initialize external ADS1x15 ADC, using internal");
        adc = &adc_esp32;
    } else {
        scan_i2c();
        ESP_LOGE("main", "Failed to initialize any ADC");
        while(1) {};
    }

    adc->setMaxExpectedVoltage(dcdcPwr.channels.s.chVin.num, 2);
    adc->setMaxExpectedVoltage(dcdcPwr.channels.s.chVout.num, 2);
    adc->setMaxExpectedVoltage(dcdcPwr.channels.s.chIin.num, 2.8f); // todo 3.3


    dcdcPwr.begin(adc);

    if (!pwm.init()) {
        ESP_LOGE("main", "Failed to init half bridge");
    }

    if (!disableWifi)
        dcdcPwr.onDataChange = dcdcDataChanged;

    mppt.startSweep();

    ESP_LOGI("main", "setup() done.");
}

unsigned long lastLoopTime = 0, maxLoopLag = 0;
unsigned long lastTimeOut = 0;
uint32_t lastNSamples = 0;

unsigned long protectCoolDownUntil = 0;

//unsigned long lastTimeMpptUpdate = 0;
unsigned long lastMpptUpdateNumSamples = 0;

bool manualPwm = false;

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


void loop() {
    auto nowMs = millis();

    dcdcPwr.update();

    if (dcdcPwr.isCalibrating()) {
        pwm.disable();
    }

    if (nowMs > protectCoolDownUntil) {
        bool mppt_ok = mppt.protect();
        if (!mppt_ok) {
            protectCoolDownUntil = nowMs + 4000;
            mppt.startSweep();
        }

        auto nSamples = dcdcPwr.numSamples.s.chIin;
        if (
            //(nowMs - lastTimeMpptUpdate) > 40 &&
                (nSamples - lastMpptUpdateNumSamples) > 0) {
            mppt.update(!mppt_ok || manualPwm);
            //lastTimeMpptUpdate = nowMs;
            lastMpptUpdateNumSamples = nSamples;
        }
    } else {
        mppt.update(true);
    }


    if ((nowMs - lastTimeOut) >= 3000) {
        auto &ewm(dcdcPwr.ewm.s);
        ESP_LOGI("main",
                 "Vin=%4.1f Vout=%4.1f Iin=%5.2fA Pin=%3.0fW T=%.0f°C sps=%2u bps=%u, PWM(H|L)=%4hu|%4hu MPPT(state=%s) lag=%lu",
                 dcdcPwr.last.s.chVin,
                 dcdcPwr.last.s.chVout,
                 dcdcPwr.last.s.chIin,
                 ewm.chVin.avg.get() * ewm.chIin.avg.get(),
        //ewm.chIin.std.get() * 1000.f, σIin=%.2fm
                 mppt.ntc.last(),
                 (lastNSamples < dcdcPwr.numSamples[0] ? (dcdcPwr.numSamples[0] - lastNSamples) : 0) * 1000u /
                 (uint32_t) (nowMs - lastTimeOut),
                 (uint32_t) (bytesSent * 1000u / millis()),
                 pwm.getBuckDutyCycle(), pwm.getBuckDutyCycleLS(),
        //mppt.getPower()
                 MpptState2String[(uint8_t) mppt.getState()].c_str(),
                 maxLoopLag
        );
        lastTimeOut = nowMs;
        lastNSamples = dcdcPwr.numSamples[0];
    }


    lcd.updateValues(LcdValues{
            .Vin = dcdcPwr.last.s.chVin,
            .Vout = dcdcPwr.last.s.chVout,
            .Iin = dcdcPwr.last.s.chIin,
            .Iout = dcdcPwr.getIoutSmooth(),
            .Temp = std::isnan(mppt.ntc.last()) ? mppt.mcu_temp.last() : mppt.ntc.last(),
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
            if ((inp[0] == '+' or inp[0] == '-') && !dcdcPwr.isCalibrating()) {
                int pwmStep = inp.toInt();
                ESP_LOGI("main", "Manual PWM step %i", pwmStep);
                manualPwm = true;
                pwm.pwmPerturb((int16_t) pwmStep);
            } else if (inp == "restart" or inp == "reset") {
                pwm.disable();
                Serial.println("Restart, delay 1s");
                delay(200);
                ESP.restart();
            } else if (inp == "mppt" && manualPwm) {
                ESP_LOGI("main", "MPPT re-enabled");
                manualPwm = false;
            } else if (inp.startsWith("dc ") && !dcdcPwr.isCalibrating()) {
                manualPwm = true;
                pwm.pwmPerturb(inp.substring(3).toInt() - pwm.getBuckDutyCycle());
            } else if (inp.startsWith("fan ")) {
                fanSet(inp.substring(4).toFloat() * 0.01f);
            } else if (inp == "sweep") {
                mppt.startSweep();
            } else if (inp == "reset-lag") {
                maxLoopLag = 0;
            } else {
                ESP_LOGI("main", "unknown command");
            }
        }
    }

    if (!disableWifi)
        wifiLoop();

    auto now = micros();
    auto lag = now - lastLoopTime;
    if (lastLoopTime && lag > maxLoopLag) maxLoopLag = lag;
    lastLoopTime = now;
}


const int BUF_SIZE = 1024;
QueueHandle_t uart_queue;

void uartInit(int port_num) {

    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

// tx=34, rx=33, stack=2048


#if CONFIG_IDF_TARGET_ESP32S3
    //const int PIN_TX = 34, PIN_RX = 33;
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif

    ESP_ERROR_CHECK(uart_set_pin(port_num, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port_num, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, intr_alloc_flags));


/* uart_intr_config_t uart_intr = {
     .intr_enable_mask = (0x1 << 0) | (0x8 << 0),  // UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
     .rx_timeout_thresh = 1,
     .txfifo_empty_intr_thresh = 10,
     .rxfifo_full_thresh = 112,
};
uart_intr_config((uart_port_t) 0, &uart_intr);  // Zero is the UART number for Arduino Serial
*/
}