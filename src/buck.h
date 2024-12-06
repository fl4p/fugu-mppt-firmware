#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <Arduino.h>

#include "pinconfig.h"
#include "pwm/ledc.h"
#include "backflow.h"
#include "store.h"
#include "util.h"

class SynchronousBuck {
    static constexpr uint8_t pwmCh_A = 0; // IN, HI (HS signal)
    static constexpr uint8_t pwmCh_B = 1; // EN or LO, depending on `pwmEnLogic`
    static constexpr float MinDutyCycleLS = 0.06f; // to keep the HS bootstrap circuit running

    PWM_ESP32_ledc pwmDriver;

    bool pwmEnLogic = false;
    uint8_t pinSd = 255;
    bool dcmHysteresis = false;
    bool lowSideEnabled = false;

    uint16_t pwmHS = 0;
    uint16_t pwmLS = 0;
    uint16_t pwmMaxLS = 0;

    float outInVoltageRatio = 0; // M
    float directionFloatBuffer = 0.0f; // fractional perturbation buffer

public:

    uint16_t pwmMaxHS{}, pwmMinLS{}, pwmMinHS{};

public:
    SynchronousBuck() : pwmDriver() {}

    SynchronousBuck(const SynchronousBuck &) = delete;

    SynchronousBuck &operator=(SynchronousBuck const &) = delete;

    [[nodiscard]] bool disabled() const { return pwmHS == 0; }

    [[nodiscard]] uint16_t getBuckOnPwmCnt() const { return pwmHS; }

    [[nodiscard]] float getBuckDutyCycle() const { return (float) pwmHS / (float) pwmMaxHS; }

    [[nodiscard]] uint16_t getBuckDutyCycleLS() const { return pwmLS; }

    [[nodiscard]] uint16_t getDutyCycleLSMax() const { return pwmMaxLS; }

    [[nodiscard]] float voltageRatio() const { return outInVoltageRatio; }

    bool init(const ConfFile &pinConf) {
        uint32_t pwmFrequency = pinConf.getLong("pwm_freq"); //39000; // buck converter switching frequency
        assert_throw(pwmFrequency > 5000 && pwmFrequency < 500000, "");

        auto drvInpLogic = pinConf.getString("pwm_driver_logic"); // driver input logic "in,en", "hi,li" and en
        uint8_t pinA, pinB;

        try {
            if (drvInpLogic == "in,en") { // e.g. Infineon ir2814

                pwmEnLogic = true;

                pinA = pinConf.getByte("pwm_in");
                pinB = pinConf.getByte("pwm_en");
                assert_throw(pinA != pinB, "");

                if (!pinConf.getByte("skip_assert", 0)) {
                    // ti, infineon gate drivers: in pins pulled low, EN/SD pins pulled high
                    assertPinState(pinA, false, "pwm_in", true);
                    assertPinState(pinB, false, "pwm_en", true);
                }

            } else if (drvInpLogic == "hi,li") {  // e.g. TI UCC21330x with optional DIS pin (SD pin)

                pinA = pinConf.getByte("pwm_hi");
                pinB = pinConf.getByte("pwm_li");
                pinSd = pinConf.getByte("pwm_sd", 255); // DIS

                assert_throw(pinA != pinB, "");
                assert_throw(pinA != pinSd, "");
                assert_throw(pinB != pinSd, "");

                if (!pinConf.getByte("skip_assert", 0)) {
                    assertPinState(pinA, false, "pwm_hi", false);
                    assertPinState(pinB, false, "pwm_li", false);
                    if (pinSd != 255) assertPinState(pinSd, true, "pwm_sd", false);
                }

            } else {
                ESP_LOGE("pwm", "unrecognized pwm_driver_logic %s", drvInpLogic.c_str());
                return false;
            }
        } catch (const std::exception &ex) {
            ESP_LOGE("pwm", "error %s", ex.what());
            return false;
        }

        pwmDriver.init_pwm(pwmCh_A, pinA, pwmFrequency);
        pwmDriver.init_pwm(pwmCh_B, pinB, pwmFrequency);

        if (pinSd != 255) {
            pinMode(pinSd, OUTPUT);
            digitalWrite(pinSd, 1);
        }

        pwmMaxHS = (uint16_t) ((float) pwmDriver.pwmMax * (1.0f - MinDutyCycleLS));
        pwmMinLS = std::ceil((float) pwmDriver.pwmMax * MinDutyCycleLS); // keeping the bootstrap circuit powered
        pwmMinHS = (pwmMinLS / 4); // everything else is too much!
        // note that mosfets have different Vg(th) and switching times worst case is Vi/o=80/12
        // ^ set pwmMinHS a bit lower than pwmMinLS (might cause no-load output over-voltage otherwise)

        ESP_LOGI("buck", "pwmDriver.pwmMax=%hu, pwmMinLS=%hu, pwmMinHS=%hu, pwmMaxHS=%hu",
                 pwmDriver.pwmMax, pwmMinLS, pwmMinHS, pwmMaxHS);

        return true;
    }

    void pwmPerturb(int16_t direction) {

        if (unlikely(disabled() and direction > 0 and pinSd != 255))
            digitalWrite(pinSd, 0);

        pwmHS = constrain(pwmHS + direction, pwmMinHS, pwmMaxHS);

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmDriver.pwmMax, outInVoltageRatio, &dcmHysteresis);
        pwmMaxLS = std::max(pwmMinLS, pwmMaxLS);

        if (pwmLS - pwmMaxLS > (pwmDriver.pwmMax / 10)) {
            UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu\n", pwmLS, pwmMaxLS);
        }

        // "fade-in" the low-side duty cycle
        pwmLS = lowSideEnabled ? constrain(pwmLS + (pwmMaxLS - pwmLS) / 4, pwmMinLS, pwmMaxLS) : pwmMinLS;

        if (-direction > pwmDriver.pwmMax / 100) {
            /*
             * with larger decreases of duty cycle do a "safe" update
             */
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_B, 0);
                pwmDriver.update_pwm(pwmCh_A, pwmHS);
                pwmDriver.update_pwm(pwmCh_B, pwmHS + pwmLS);
            } else {
                if (pinSd != 255) digitalWrite(pinSd, 1);
                else pwmDriver.update_pwm(pwmCh_B, 0);
                pwmDriver.update_pwm(pwmCh_A, pwmHS);
                pwmDriver.update_pwm(pwmCh_B, pwmLS, pwmHS);
                if (pinSd != 255) digitalWrite(pinSd, 0);
            }
        } else if (direction < 0) {
            /*
             * update EN before IN
             * pwmHS will be smaller than before. after the EN update (we don't know the direction) pwmHS stays at the
             * old value (larger) causing the effective pwmLS to be less, which is ok
             */
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_B, pwmHS + pwmLS);
            } else {
                pwmDriver.update_pwm(pwmCh_B, pwmHS, pwmLS);
            }
            pwmDriver.update_pwm(pwmCh_A, pwmHS);
        } else {
            /* update IN before EN
             * we increase pwmHS here, so update it first
             */
            pwmDriver.update_pwm(pwmCh_A, pwmHS);
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_B, pwmHS + pwmLS);
            } else {
                pwmDriver.update_pwm(pwmCh_B, pwmHS, pwmLS);
            }
        }
    }


    /**
     * Perturb by a fractional step.
     * Instantly perturbs the integer part and accumulates the remainder in a buffer
     *
     * @param directionFloat
     */
    void pwmPerturbFractional(float directionFloat) {
        assert(std::abs(directionFloat) < pwmDriver.pwmMax);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int16_t) (directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }


    void disable() {
        if (pinSd != 255)digitalWrite(pinSd, 1);
        pwmDriver.update_pwm(pwmCh_B, 0); // disable EN before IN !
        pwmDriver.update_pwm(pwmCh_A, 0);

        if (pwmHS > pwmMinHS)
            UART_LOG("PWM disabled (duty cycle was %d)\n", (int) pwmHS);
        pwmHS = 0;
        pwmLS = 0;
        lowSideEnabled = false;
    }


    /**
     * Compute low-side switch duty cycle to emulate a diode for synchronous buck (sensor-less)
     * Prevents reverse current through the LS switch, which would lead to voltage boost and reverse current
     * and can eventually destroy the LS switch.
     * High-voltages at the input can destroy anything connected (including the board itself!)
     *
     * The function does some error estimation and decides whether we are in DCM or CCM mode and limits the LS duty cycle accordingly.
     * See https://www.ti.com/seclit/ug/slyu036/slyu036.pdf#page=19 for more info about sync buck modes and timings
     *
     * by Fabian Schlieper (fl4p) https://github.com/fl4p/fugu-mppt-firmware/
     *
     * @param pwmHS Duty cycle of the HS switch
     * @param pwmMax The maximum duty cycle value
     * @param voltageRatio Vout/Vin ratio, D (the greater, the safer but less efficient)
     * @return
     */
    static uint16_t computePwmMaxLs(uint16_t pwmHS, const uint16_t pwmMax, float voltageRatio, bool *dcmHysteresis) {

        const float coilEfficiency = 0.99f; // TODO what does this model or represent, remove? maybe coil losses?

        // the computation below is very sensitive to voltageRatio errors!
        // at VR=0.64 a -5% error causes a 13% deviation of pwmMaxLs !
        // so we do some proper error computation here
        constexpr float voltageMaxErr = 0.02f; // inc -> safer, less efficient

        // WCEF = worst case error factor
        // compute the worst case error (Vout estimated too low, Vin too high)
        constexpr float voltageRatioWCEF = (1.f - voltageMaxErr) / (1.f + voltageMaxErr); // < 1.0

        voltageRatio = constrain(voltageRatio, 1e-2f, 1.f - 1e-2f); // constrain to reasonable range

        // compute worst-case error at current working point
        const float pwmMaxLsWCEF = (1 / (voltageRatio * voltageRatioWCEF) - 1) / (1 / voltageRatio - 1); // > 1

        auto pwmMaxLs = (1 / voltageRatio - 1) * (float) pwmHS / pwmMaxLsWCEF; // the lower, the safer

        auto lsCCM = pwmMax - pwmHS;
        if (pwmMaxLs < (float) (lsCCM - ((*dcmHysteresis) ? 0u : (pwmMax / 100u)))) {
            // DCM (Discontinuous Conduction Mode)
            // this is when the coil current is still touching zero, it'll stop for higher HS duty cycles
            // Allowing higher duty cycles here would result a synchronous forced PWM. (reverse coil current, can be dangerous)
            // (forced PWM reduces output ripple?) TODO

            if (!*dcmHysteresis) {
                *dcmHysteresis = true;
                UART_LOG("buck: CCM -> DCM (vr=%.4f, pwmMaxLs=%.1f, lsCCM=%hu)", voltageRatio, pwmMaxLs, lsCCM);
            }

            pwmMaxLs = std::min<float>(pwmMaxLs, (float) pwmHS);

        } else {
            // CCM (Continuous Conduction Mode)
            // coil current never becomes zero

            if (*dcmHysteresis) {
                *dcmHysteresis = false;
                UART_LOG("buck: DCM -> CCM (vr=%.4f, pwmMaxLs=%.1f, lsCCM=%hu)\n", voltageRatio, pwmMaxLs, lsCCM);
            }

            pwmMaxLs = (float) (pwmMax - pwmHS);
        }

        //ESP_LOGI("dbg", "pwmMaxLs=%f, (float) (pwmMax - pwmHS)=%f, pwmMax=%hu, pwmHS=%hu", pwmMaxLs, (float) (pwmMax - pwmHS), pwmMax, pwmHS);

        // todo beyond pwmMaxLs < (pwmMax - pwmHS), can we reduce the worst-case error assumption to boost eff?
        // or just replace  with pwmMaxLs < (pwmMax - pwmHS) * pwmMaxLsWCEF and remove scaling pwmMaxLs by pwmMaxLsWCEF

        return (uint16_t) (pwmMaxLs * coilEfficiency);
    }

    const float &updateLowSideMaxDuty(float vout, float vin) {
        // voltageRatio = Vout/Vin

        if (vin > vout && vin > 0.1f) {
            outInVoltageRatio = vout / vin;
        } else {
            outInVoltageRatio = 1; // the safest (minimize LS duty cycle)
        }

        pwmMaxLS = computePwmMaxLs(pwmHS, pwmDriver.pwmMax, outInVoltageRatio, &dcmHysteresis);

        pwmMaxLS = std::max(pwmMinLS, pwmMaxLS);

        if (pwmLS > pwmMaxLS) {
            if (pwmLS - pwmMaxLS > (pwmDriver.pwmMax / 40)) {
                UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu (VR=%.3f)", pwmLS, pwmMaxLS, outInVoltageRatio);
            }
            pwmLS = pwmMaxLS;
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_B, pwmHS + pwmLS); // instantly commit if limit decreases
            } else {
                pwmDriver.update_pwm(pwmCh_B, pwmHS, pwmLS);
            }
        }

        return outInVoltageRatio;
    }


    /**
     * Permanently set enable/disable low-side switch
     * @param enable
     */
    void enableLowSide(bool enable) {
        if (enable != lowSideEnabled) {
            UART_LOG("Low-side switch %s", enable ? "enabled" : "disabled");
        }
        lowSideEnabled = enable;
        if (!enable)
            lowSideMinDuty();
    }

    /**
     * Set low-side duty cycle to minimum (e.g. disable power conduction, just keep the bootstrapping powered for HS drive)
     * Notice that this is only needed if no inductor is connected to the half-bridge, as the inductor will push the
     * switch node voltage until LS diode conducts.
     */
    void lowSideMinDuty() {
        if (pwmLS > pwmMinLS) {
            if (pwmLS > pwmMinLS + pwmMinLS / 2) {
                ESP_LOGW("buck", "set low-side PWM to minimum %hu -> %hu (vRatio=%.3f, pwmMaxLS=%hu)", pwmLS, pwmMinLS,
                         outInVoltageRatio, pwmMaxLS);
            }
            pwmLS = pwmMinLS;
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_B, pwmHS + pwmLS);
            } else {
                pwmDriver.update_pwm(pwmCh_B, pwmHS, pwmLS);
            }
        }
    }
};