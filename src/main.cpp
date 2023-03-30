#include <Arduino.h>

#include "adc.h"
#include "ads.h"
#include "statmath.h"
#include "sampling.h"

#include <type_traits>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>


//LiquidCrystal_I2C lcd(0x27,16,2);   //SYSTEM PARAMETER  - Configure LCD RowCol Size and I2C Address
//Adafruit_ADS1115 ads;             //SYSTEM PARAMETER  - ADS1115 ADC Library (By: Adafruit) Kindly uncomment this if you are using ADS1115
LiquidCrystal_I2C lcd(0x3F, 16, 2);
TaskHandle_t Core2;    //SYSTEM PARAMETER  - Used for the ESP32 dual core operation


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
        .chVin = {3, 204.7 / 4.7},
        .chVout = {1, (47. / 2 + 1) / 1},
        .chIin = {2, 1.0f} // todo offset
}}};

void setup() {

    adc.setChannelGain(dcdcPwr.channels.s.chVin.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chVout.num, GAIN_TWO);
    adc.setChannelGain(dcdcPwr.channels.s.chIin.num, GAIN_TWO);

    if (!adc.init()) {
        ESP_LOGI("main", "Failed to initialize ADC");
        while (true) yield();
    }
}

void loop() {
    dcdcPwr.update();

    ESP_LOGI("main", "VIN=%5.2f VOUT=%5.2f IIN=%5.2f", dcdcPwr.ewm.chVin.avg.get(), dcdcPwr.ewm.chVout.avg.get(),
             dcdcPwr.ewm.chIin.avg.get());
}