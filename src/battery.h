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
    float voltageStep = 14.6f; // lifepo4 4s (3.65V)
    float minVoltage = 10.f;

    int n = std::ceil(idleVoltage / voltageStep);
    if (n > 1) {
        if (idleVoltage < n * minVoltage) {
            return NAN;
        }
    } else {
        n = 1;
    }

    return n * voltageStep;
}