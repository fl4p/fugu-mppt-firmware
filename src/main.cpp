#include <Arduino.h>

#include "adc.h"
#include "ads.h"
#include "sampling.h"
#include "pwm.h"
#include "mppt.h"
#include "util.h"
#include "telemetry.h"

#include <Wire.h>
//#include <SPI.h>
//#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

#include "pinconfig.h"

//LiquidCrystal_I2C lcd(0x27,16,2);   //SYSTEM PARAMETER  - Configure LCD RowCol Size and I2C Address
//Adafruit_ADS1115 ads;             //SYSTEM PARAMETER  - ADS1115 ADC Library (By: Adafruit) Kindly uncomment this if you are using ADS1115
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
//TaskHandle_t Core2;    //SYSTEM PARAMETER  - Used for the ESP32 dual core operation


//#define backflow_MOSFET 27  //SYSTEM PARAMETER - Backflow MOSFET
#define LED 2               //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN 16              //SYSTEM PARAMETER - Fan GPIO Pin
//#define ADC_ALERT 34        //SYSTEM PARAMETER - Fan GPIO Pin
#define TempSensor 35       //SYSTEM PARAMETER - Temperature Sensor GPIO Pin
#define buttonLeft 18       //SYSTEM PARAMETER -
#define buttonRight 17      //SYSTEM PARAMETER -
#define buttonBack 19       //SYSTEM PARAMETER -
#define buttonSelect 23     //SYSTEM PARAMETER -


ADC_ADS adc;
DCDC_PowerSampler dcdcPwr{adc, ThreeChannelUnion<ChannelAndFactor>{.s={
        .chVin = {3, 204.7 / 4.7, 0},
        .chVout = {1, (47. / 2 + 1) / 1, 0},
        .chIin = {2, -1 / 0.066f * (10 + 3.3) / 10., //ACS712-30 sensitivity)
                /* TODO midpoint should be 2.5 (5/2)*/
                // 3.5 * 10. / (10 + 3.3) * 0.99788f
                  2.5 * 10. / (10 + 3.3) - 0.0117,
        }, // 2.5250f/1.3300f
}}};

HalfBridgePwm pwm;
MpptSampler mppt{dcdcPwr, pwm};


void ICACHE_RAM_ATTR NewDataReadyISR() {
    adc.alertNewDataFromISR();
}

bool disableWifi = false;

void setup() {
    if (!disableWifi)
        connect_wifi_async("^__^", "xxxxxxxx");
    //connect_wifi_async("mentha", "xxxxxxxx");
    
    Wire.setClock(400000UL);
    Wire.setPins((uint8_t) PinConfig::I2C_SDA, (uint8_t) PinConfig::I2C_SCL);

    if (!Wire.begin()) {
        ESP_LOGE("main", "Failed to initialize Wire");
    }

    adc.setChannelGain(dcdcPwr.channels.s.chVin.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chVout.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chIin.num, GAIN_ONE);

    auto r = adc.init();
    if (!r) {
        ESP_LOGE("main", "Failed to initialize ADC (%i)", r);
    }

    pinMode((uint8_t) PinConfig::ADC_ALERT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt((uint8_t) PinConfig::ADC_ALERT), NewDataReadyISR, FALLING); // TODO rising

    dcdcPwr.begin();

    if (!pwm.init()) {
        ESP_LOGE("main", "Failed to init half bridge");
    }

    if (!disableWifi)
        dcdcPwr.onDataChange = dcdcDataChanged;

    dcdcPwr.startCalibration();
    mppt.startSweep();

    ESP_LOGI("main", "setup() done.");
}


unsigned long lastTimeOut = 0;
uint32_t lastNSamples = 0;

unsigned long protectCoolDownUntil = 0;

unsigned long lastTimeMpptUpdate = 0;
unsigned long lastMpptUpdateNumSamples = 0;


void loop() {
    //scan_i2c();
    auto nowMs = millis();

    dcdcPwr.update();


    if (nowMs > protectCoolDownUntil) {
        bool mppt_ok = mppt.protect();
        if (!mppt_ok) {
            protectCoolDownUntil = nowMs + 3000;
            mppt.startSweep();
        }

        auto nSamples = dcdcPwr.numSamples.s.chIin;
        if (
             (nowMs - lastTimeMpptUpdate) > 10
            && (nSamples - lastMpptUpdateNumSamples) > 0) {
            mppt.update(!mppt_ok);
            lastTimeMpptUpdate = nowMs;
            lastMpptUpdateNumSamples = nSamples;
        }
    } else {
        mppt.update(true);
    }


    if ((nowMs - lastTimeOut) >= 2000) {
        auto &ewm(dcdcPwr.ewm.s);
        ESP_LOGI("main", "Vin=%5.1f Vout=%5.1f Iin=%5.3f Pin=%.1f σIin=%.2fm sps=%u PWM=%hu MPPT=(P=%.1f state=%s)",
                 dcdcPwr.last.s.chVin,
                 dcdcPwr.last.s.chVout,
                 dcdcPwr.last.s.chIin,
                 ewm.chVin.avg.get() * ewm.chIin.avg.get(),
                 ewm.chIin.std.get() * 1000.f,
                  (dcdcPwr.numSamples[0] - lastNSamples), pwm.getBuckDutyCycle(),
                 mppt.getPower(), MpptState2String[mppt.getState()].c_str());
        lastTimeOut = nowMs;
        lastNSamples = dcdcPwr.numSamples[0];
    }

    if (!disableWifi)
        wifiLoop();
}
