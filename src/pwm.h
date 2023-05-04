#include <cstdint>
#include <cmath>
#include<algorithm>
#include <Arduino.h>

#include "pinconfig.h"


class HalfBridgePwm {

    static constexpr uint8_t pwmCh_IN = 0;
    static constexpr uint8_t pwmCh_EN = 1;

    static constexpr float MinDutyCycleLS = 0.06f; // to keep the HS bootstrap circuit running

    uint8_t pwmResolution = 10;

    uint16_t pwmHS = 0;
    uint16_t pwmLS = 0;

    uint16_t pwmMaxLS = 0;

    float outInVoltageRatio = 0;

public:

    const uint16_t pwmMax, pwmMaxHS, pwmMinLS;
    uint16_t pwmStartHS = 0;


    HalfBridgePwm()
            : pwmMax(pow(2, pwmResolution) - 1), pwmMaxHS(pwmMax * (1.0f - MinDutyCycleLS)),
              pwmMinLS(std::ceil(pwmMax * MinDutyCycleLS)) {}

    bool init() {

        uint32_t pwmFrequency = 39000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        float PPWM_margin = 99.5;          //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
        float PWM_MaxDC = 95.0;            //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good, 97 makes low-side turn-on too short for bootstrapping

        //uint16_t pwmMax = 0;
        //uint16_t pwmMaxLimited = 0;
        //uint16_t PWM = 0;

        //PWM INITIALIZATION
        auto freq = ledcSetup(pwmCh_IN, pwmFrequency, pwmResolution);
        if(!freq) {
            return false;
        }
        ledcAttachPin( (uint8_t )PinConfig::Bridge_IN, pwmCh_IN);
        ledcWrite(pwmCh_IN, 0);

        // EN PWM Init
        freq = ledcSetup(pwmCh_EN, pwmFrequency, pwmResolution);
        if(!freq) {
            return false;
        }
        ledcAttachPin((uint8_t )PinConfig::Bridge_EN, pwmCh_EN);
        ledcWrite(pwmCh_EN, 0);

        pinMode((uint8_t )PinConfig::Backflow_EN,OUTPUT);

        // pwmMax = pow(2, pwmResolution) - 1;                           //Get PWM Max Bit Ceiling
        //pwmMaxLimited =(PWM_MaxDC * pwmMax) / 100.0f;


        return true;
    }


    void pwmPerturb(int8_t direction) {

        if (pwmHS <= 1 && pwmHS < pwmStartHS)
            pwmHS = pwmStartHS;

        // compute ordinary pwm update within bounds:
        pwmHS = constrain(pwmHS + direction, 0, pwmMaxHS);// pwmMinLS


        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);

        // "fade-in" the low-side duty cycle
        pwmLS = constrain(pwmLS + 2, pwmMinLS, pwmMaxLS);

        ledcWrite(pwmCh_IN, pwmHS);
        ledcWrite(pwmCh_EN, pwmHS + pwmLS);
    }

    float directionFloatBuffer = 0.0f;

    void pwmPerturbFractional(float directionFloat) {
        assert(std::abs(directionFloat) < 128);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int8_t) (directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }

    uint16_t getBuckDutyCycle() const {
        return pwmHS;
    }

    uint16_t getBuckDutyCycleLS() const {
        return pwmLS;
    }

    void halfDutyCycle() {
        pwmHS /= 2;
        pwmLS /= 2;
        pwmPerturb(0);
    }

    void disable() {
        if (pwmHS > 0)
            ESP_LOGW("pwm", "PWM disabled");
        pwmHS = 0;
        pwmLS = 0;
        ledcWrite(pwmCh_IN, 0);
        ledcWrite(pwmCh_EN, 0);
        enableBackflowMosfet(false);
    }

    void lowSideMinDuty() {
        if (pwmLS > pwmMinLS + 10)
            ESP_LOGW("pwm", "set low-side PWM to minimum %hu -> %hu", pwmLS, pwmMinLS);
        pwmLS = pwmMinLS;
        ledcWrite(pwmCh_EN, pwmHS + pwmLS);
    }

    static uint16_t computePwmMaxLs(uint16_t pwmHS, uint16_t pwmMax, float voltageRatio) {
        // prevents boosting and "low side burning" due to excessive LS current
        // (non-diode behavior, negative reverse inductor current)
        const float margin = 0.99f;
        voltageRatio = std::max<float>(voltageRatio, 0.01f);
        auto pwmMaxLs = (uint16_t) ((1 / voltageRatio - 1) * margin * (float) pwmHS);
        return std::min<uint16_t>(pwmMaxLs, pwmMax - pwmHS);
    }

    void updateLowSideMaxDuty(float outInVoltageRatio_) {
        // voltageRatio = Vout/Vin

        outInVoltageRatio = outInVoltageRatio_;

        //auto idealBuckPwm = (uint16_t) ((float) pwmMax * (margin / 100.f) * std::min(1.f, voltageRatio));
        // idealBuckPwm / 4 is still too much for open circuit output
        pwmStartHS = pwmMinLS; // idealBuckPwm / 4; // k =.75

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmMax, outInVoltageRatio);

        if (pwmLS > pwmMaxLS) {
            pwmLS = pwmMaxLS;
            ledcWrite(pwmCh_EN, pwmHS + pwmLS); // instantly commit if limit decreases
        }
    }

    void enableBackflowMosfet(bool enable) {
        digitalWrite((uint8_t )PinConfig::Backflow_EN,enable);
    }

};