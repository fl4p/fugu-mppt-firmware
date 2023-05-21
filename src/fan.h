#pragma once

#include <Arduino.h>
#include "pinconfig.h"

#define FAN_PWM_CH 2
#define FAN_PWM_BITS 8

bool fanInit() {
    pinMode((uint8_t)PinConfig::Fan, OUTPUT);

    auto freq = ledcSetup(FAN_PWM_CH, 16000, FAN_PWM_BITS);
    if(!freq) {
        ESP_LOGE("fan", "ledcSetup failed");
        return false;
    }

    ledcAttachPin((uint8_t)PinConfig::Fan, FAN_PWM_CH);
    ledcWrite(FAN_PWM_CH, 0);

    return true;
}

void fanSet(float duty) {
    //if(duty > 1) duty = 1;
    //if(duty < 0) duty = 0;

    if(duty < 0.1) duty = 0;
    else if(duty <= 0.5) duty = 0.06f;
    else if(duty <= 0.75) duty = 0.2f;
    else duty = 1.f;

    ledcWrite(FAN_PWM_CH, (uint16_t)(((2 << (FAN_PWM_BITS-1)) - 1) * duty));
}

void fanUpdateTemp(float temp, float power) {
    static unsigned long fanOnTime = 0;

    if(temp > 65) {
        fanSet(100.0f);
    } else if(temp > 60) {
        fanSet(75.0f);
    } else if(temp > 55) {
        fanSet(50.0f);
    } else if(temp > 48 ) { // or power > 550
        fanSet(25.0f);
        fanOnTime = millis();
    } else if( temp < (power > 100 ? 40.f : 45.f) && (millis() - fanOnTime > 30000) ) {
        fanSet(0);
    }
}

