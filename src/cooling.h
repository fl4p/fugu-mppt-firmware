#pragma once

#include <Arduino.h>
#include "pinconfig.h"

#define FAN_PWM_CH 2
#define FAN_PWM_BITS 8

bool fanInit() {
    pinMode((uint8_t) PinConfig::Fan, OUTPUT);

    auto freq = ledcSetup(FAN_PWM_CH, 16000, FAN_PWM_BITS);
    if (!freq) {
        ESP_LOGE("fan", "ledcSetup failed");
        return false;
    }

    ledcAttachPin((uint8_t) PinConfig::Fan, FAN_PWM_CH);
    ledcWrite(FAN_PWM_CH, 0);

    return true;
}

/**
 *
 * @param duty Duty-cycle [0..1]
 */
void fanSet(float duty) {
    //if(duty > 1) duty = 1;
    //if(duty < 0) duty = 0;

    static uint16_t driverDC = 0;

    if (duty < 0.1) duty = 0;
    else if (duty <= 0.25) duty = 5;
    else if (duty <= 0.5) duty = 15;
    else if (duty <= 0.75) duty = 40;
    else duty = 100.f;

    auto dc = (uint16_t) (((2 << (FAN_PWM_BITS - 1)) - 1) * duty * 0.01f);
    if (driverDC != dc) {
        ledcWrite(FAN_PWM_CH, dc);
        driverDC = dc;
    }
}

void fanUpdateTemp(float temp, float power) {
    static unsigned long fanOnTime = 0;

    if (temp > 70) {
        fanSet(1.f);
    } else if (temp > 60) {
        fanSet(.75f);
    } else if (temp > 55) {
        fanSet(.5f);
    } else if (temp >= 49.5f) { // or power > 550
        fanSet(.25f);
        fanOnTime = millis();
    } else if (temp < (power > 100 ? 35.f : 45.f) && (millis() - fanOnTime > 30000)) {
        fanSet(0);
    }
}

float thermalPowerLimit(float ntcTemp) {
    float powerLimit = std::numeric_limits<float>::max();

    // if temp is below 5, there might be an issue with the sensor
    if (ntcTemp > 90) {
        powerLimit = 20;
    } else if (ntcTemp > 80) {
        powerLimit = 200;
    } else if (ntcTemp > 75 or std::isnan(ntcTemp) or ntcTemp < 5) {
        powerLimit = 300;
    } else if (ntcTemp > 65) {
        powerLimit = 700 - (ntcTemp - 65) * 40;
    }

    return powerLimit;
}

