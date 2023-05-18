#pragma once

#include<cmath>

enum BatteryChemistry {
    None = 0,
    LiFePo4,
    AGM,
    LeadAcid,
    Custom,
    Max,
};

float detectMaxBatteryVoltage(float idleVoltage) {
    float voltageStep = 14.6f; // lifepo4 4s (3.65V) [1,2,4]
    float minVoltage = 10.f;

    for (int n = 1; n < 4; n *= 2) {
        if(idleVoltage > minVoltage * (float)n && idleVoltage < voltageStep * (float)n ) {
            return voltageStep * (float)n;
        }
    }

    // default to 12V systems if voltage is low
    if(idleVoltage < minVoltage)
        return voltageStep;

    // not detectable
    return NAN;
}