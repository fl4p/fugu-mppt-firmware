#include <cstdint>
#include <cmath>
#include<algorithm>
#include <Arduino.h>


class HalfBridgePwm {
    static constexpr uint8_t buck_IN = 33;
    static constexpr uint8_t buck_EN = 32;

    static constexpr uint8_t pwmCh_IN = 0;
    static constexpr uint8_t pwmCh_EN = 1;

    static constexpr float MinDutyCycleLS = 0.05f;

    uint8_t pwmResolution = 11;

    uint16_t pwmHS = 0;
    uint16_t pwmLS = 0;

    uint16_t pwmMaxLS = 0;

public:

    const uint16_t pwmMax, pwmMaxHS, pwmMinLS;
    uint16_t pwmStartHS = 0;


    HalfBridgePwm()
            : pwmMax(pow(2, pwmResolution) - 1), pwmMaxHS(pwmMax * (1.0f - MinDutyCycleLS)),
              pwmMinLS(pwmMax * MinDutyCycleLS) {}

    bool init() {

        uint32_t pwmFrequency = 39000;           //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
        float PPWM_margin = 99.5;          //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
        float PWM_MaxDC = 95.0;            //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good, 97 makes low-side turn-on too short for bootstrapping

        //uint16_t pwmMax = 0;
        //uint16_t pwmMaxLimited = 0;
        //uint16_t PWM = 0;

        //PWM INITIALIZATION
        ledcSetup(pwmCh_IN, pwmFrequency, pwmResolution);
        ledcAttachPin(buck_IN, pwmCh_IN);
        ledcWrite(pwmCh_IN, 0);

        // EN PWM Init
        ledcSetup(pwmCh_EN, pwmFrequency, pwmResolution);
        ledcAttachPin(buck_EN, pwmCh_EN);
        ledcWrite(pwmCh_EN, 0);

        // pwmMax = pow(2, pwmResolution) - 1;                           //Get PWM Max Bit Ceiling
        //pwmMaxLimited =(PWM_MaxDC * pwmMax) / 100.0f;


        return true;
    }

    void pwmPerturb(int8_t direction) {

        if (pwmHS <= 1)
            pwmHS = pwmStartHS;

        // compute ordinary pwm update within bounds:
        pwmHS = constrain(pwmHS + direction, 0, pwmMaxHS);// pwmMinLS


        // "fade-in" the low-side duty cycle
        pwmLS = constrain(pwmLS + 1, pwmMinLS, pwmMaxLS);

        ledcWrite(pwmCh_IN, pwmHS);
        ledcWrite(pwmCh_EN, std::min<uint16_t>(pwmHS + pwmLS, pwmMax));
    }

    uint16_t getBuckDutyCycle() const {
        return pwmHS;
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
    }

    void lowSideMinDuty() {
        pwmLS = pwmMinLS;
        ledcWrite(pwmCh_EN, pwmHS + pwmLS);
    }

    void updateLowSideMaxDuty(float voltageRatio) {
        // voltageRatio = Vout/Vin

        float margin = 99.5;
        auto idealBuckPwm = (uint16_t) ((float) pwmMax * (margin / 100.f) * std::min(1.f, voltageRatio));

        pwmStartHS = idealBuckPwm / 2; // k =.75

        // prevents boosting and "low side burning" due to excessive LS current (non-diode behavior?):
        pwmMaxLS = pwmMax - idealBuckPwm;
        if (pwmLS > pwmMaxLS) {
            pwmLS = pwmMaxLS;
            ledcWrite(pwmCh_EN, pwmHS + pwmLS); // instantly commit if limit decreases
        }
    }

};