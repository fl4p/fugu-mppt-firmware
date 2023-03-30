#include <Arduino.h>

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
Adafruit_ADS1015 ads;  //SYSTEM PARAMETER  - ADS1015 ADC Library (By: Adafruit) Kindly delete this line if you are using ADS1115


#define backflow_MOSFET 27  //SYSTEM PARAMETER - Backflow MOSFET
#define buck_IN 33          //SYSTEM PARAMETER - Buck MOSFET Driver PWM Pin
#define buck_EN 32          //SYSTEM PARAMETER - Buck MOSFET Driver Enable Pin
#define LED 2               //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN 16              //SYSTEM PARAMETER - Fan GPIO Pin
#define ADC_ALERT 34        //SYSTEM PARAMETER - Fan GPIO Pin
#define TempSensor 35       //SYSTEM PARAMETER - Temperature Sensor GPIO Pin
#define buttonLeft 18       //SYSTEM PARAMETER -
#define buttonRight 17      //SYSTEM PARAMETER -
#define buttonBack 19       //SYSTEM PARAMETER -
#define buttonSelect 23     //SYSTEM PARAMETER -

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}