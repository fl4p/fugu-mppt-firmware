#include <cstdint>
#include <Arduino.h>


class HalfBridgePwm {
    static const uint8_t buck_IN = 33;
    static const uint8_t buck_EN = 32;

    static const uint8_t pwmCh_IN = 0;
    static const uint8_t pwmCh_EN = 1;

    bool init() {

        uint8_t pwmResolution = 11;             //  USER PARAMETER - PWM Bit Resolution
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

    }


    void enable(bool en) {

    }
};