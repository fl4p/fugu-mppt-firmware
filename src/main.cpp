#include <Arduino.h>

#include "adc.h"
#include "ads.h"
#include "sampling.h"
#include "pwm.h"
#include "mppt.h"


#include <Wire.h>
//#include <SPI.h>
//#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <InfluxDbClient.h>

//LiquidCrystal_I2C lcd(0x27,16,2);   //SYSTEM PARAMETER  - Configure LCD RowCol Size and I2C Address
//Adafruit_ADS1115 ads;             //SYSTEM PARAMETER  - ADS1115 ADC Library (By: Adafruit) Kindly uncomment this if you are using ADS1115
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
//TaskHandle_t Core2;    //SYSTEM PARAMETER  - Used for the ESP32 dual core operation


#define backflow_MOSFET 27  //SYSTEM PARAMETER - Backflow MOSFET
#define LED 2               //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN 16              //SYSTEM PARAMETER - Fan GPIO Pin
#define ADC_ALERT 34        //SYSTEM PARAMETER - Fan GPIO Pin
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

void scan_i2c();

constexpr int READY_PIN = 34;

void ICACHE_RAM_ATTR NewDataReadyISR() {
    adc.alertNewDataFromISR();
}


void setup() {
    //Wire.end();
    Wire.setClock(400000UL);
    Wire.setPins(21, 22);
    if (!Wire.begin()) {
        ESP_LOGE("main", "Failed to initialize Wire");
        while (true) yield();
    }


    //scan_i2c();

    adc.setChannelGain(dcdcPwr.channels.s.chVin.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chVout.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chIin.num, GAIN_ONE);

    auto r = adc.init();
    if (!r) {
        ESP_LOGE("main", "Failed to initialize ADC %i", r);
    }

    pinMode(READY_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING); // TODO rising

    dcdcPwr.begin();

    if(!pwm.init()) {
        ESP_LOGE("main", "Failed to init half bridge");
    }

}

std::vector<Point> points_frame;

unsigned long lastTimeOut = 0;
uint32_t lastNSamples = 0;

unsigned long lastTimeMpptUpdate = 0;

void loop() {
    //scan_i2c();
    auto nowMs = millis();

    dcdcPwr.update();

    bool mppt_ok = mppt.protect();

    if(mppt_ok && (nowMs - lastTimeMpptUpdate) > 20) {
        mppt.update();
        lastTimeMpptUpdate = nowMs;
    }


    if ((nowMs - lastTimeOut) >= 1000) {
        auto &ewm(dcdcPwr.ewm.s);
        ESP_LOGI("main", "VIN=%5.2f VOUT=%5.2f IIN=%5.3f sps=%u PWM=%hu", dcdcPwr.last.s.chVin, ewm.chVout.avg.get(),
                 dcdcPwr.last.s.chIin, (dcdcPwr.numSamples[0] - lastNSamples), pwm.getBuckDutyCycle());
        lastTimeOut = nowMs;
        lastNSamples = dcdcPwr.numSamples[0];
    }


}


void scan_i2c() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000);           // wait 5 seconds for next scan
}
