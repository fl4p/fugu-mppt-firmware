#pragma once

#include<cmath>

enum CellChemistry {
    None = 0,
    LiFePo4, // LFP 2.5~3.65V

    NCM, // LiCoMn LiIon, LiPo, 3.0~4.2V. Pedelec, EV (no Tesla), Tesla Powerwall
    NCA, // 3.0~4.2V. highest 279 mAh/g, Tesla EV

    LTO, // Li4Ti5O12 1.8~2.65V (float 2.25V) https://www.lto-store.de/home/Yinlong-LTO66160K-2-3V-45Ah-A-grade-p542371477

    AGM,
    LeadAcid,// https://batteryuniversity.com/article/bu-403-charging-lead-acid

    SIB, // sodium-ion

    Custom,

    Max,
};

float detectMaxBatteryVoltage(float openCircuitVoltage) {
    auto constexpr CellVolt = 3.65;
    auto constexpr CellPack = 4;
    float voltageStep = CellPack * CellVolt; // lifepo4 4s (3.65V) [1,2,4]
    float minVoltage = 10.f;

    for (int n = 1; n < 4; n *= 2) {
        if (openCircuitVoltage > minVoltage * (float) n && openCircuitVoltage < voltageStep * (float) n) {
            return voltageStep * (float) n;
        }
    }

    // default to 12V systems if voltage is low
    if (openCircuitVoltage < minVoltage)
        return voltageStep;

    // not detectable
    return NAN;
}