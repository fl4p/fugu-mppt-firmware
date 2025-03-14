#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>


#ifndef MOCK

#include <Arduino.h>
//#include "pinconfig.h"
#include "pwm/ledc.h"

#else

#include "pwm/mock.h"

#endif

#include "conf.h"
#include "util.h"

/**
 * Synchronous buck or boost converter
 */
class SynchronousConverter {
    static constexpr uint8_t pwmCh_Ctrl = 0; // Ctrl FET; buck: IN, HI (HS signal)
    static constexpr uint8_t pwmCh_Rect = 1; // Rect FET; buck: EN or LO, depending on `pwmEnLogic`

    static constexpr float MinDutyCycleLS = 0.06f; // to keep the HS bootstrap circuit running

    /**
     * instead of the %µi-curve we simply use a constant drop factor
     * this appears to be sufficient for CCM/DCM decision
     * notice that the more accurate we model coil current, the higher the efficiency in DCM and near-DCM condition
     */
    static constexpr float InductivityDcBias = 0.95f;


#ifdef ARDUINO
    PWM_ESP32_ledc pwmDriver;
#else
    PWM_Mock pwmDriver;
#endif

    bool pwmEnLogic = false; // whether the driver has a IN/SD input logic (instead of HI/LO)
    uint8_t pinSd = 255;
    bool dcmHysteresis = false;
    bool syncRectEnabled = false;

    uint16_t pwmCtrl = 0; // buck: HS
    uint16_t pwmRect = 0; // buck: LS
    uint16_t pwmRectMax = 0;
    float pwmRectRatioDCM = 0; // t_onRect/t_onCtrl when in DCM

    float outInVoltageRatio = 0; // M
    float directionFloatBuffer = 0.0f; // fractional perturbation buffer

    bool isBoost = false;
    bool forcedPwm = false;

    float fL = NAN; // fsw * L

public:

    inline bool boost() const { return isBoost; }

    inline bool forcedPwm_() const { return forcedPwm; }

    uint16_t pwmCtrlMax{}, pwmRectMin{}, pwmCtrlMin{};

public:
    SynchronousConverter() : pwmDriver() {}

    SynchronousConverter(const SynchronousConverter &) = delete;

    SynchronousConverter &operator=(SynchronousConverter const &) = delete;

    [[nodiscard]] bool disabled() const { return pwmCtrl == 0; }

    [[nodiscard]] float getDutyCycle() const { return (float) pwmCtrl / (float) pwmCtrlMax; }

    [[nodiscard]] uint16_t getCtrlOnPwmCnt() const { return pwmCtrl; }

    [[nodiscard]] uint16_t getCtrlOnPwmMin() const { return pwmCtrlMin; }

    [[nodiscard]] uint16_t getRectOnPwmCnt() const { return pwmRect; }

    [[nodiscard]] uint16_t getLowSideOnPwmCnt() const { return isBoost ? pwmCh_Ctrl : pwmRect; }

    [[nodiscard]] uint16_t getLowSideMinPwmCnt() const { return isBoost ? 0 : pwmRectMin; }

    [[nodiscard]] uint16_t getRectOnPwmMax() const { return pwmRectMax; }

    [[nodiscard]] uint16_t getRectOnPwmMin() const { return pwmRectMin; }

    [[nodiscard]] float voltageRatio() const { return outInVoltageRatio; } // M

    void init(const ConfFile &converterConf, const ConfFile &pinConf, const ConfFile &coilConf) {

        isBoost = converterConf.getByte("boost", 0);
        forcedPwm = converterConf.getByte("forced_pwm", 0);

        if (forcedPwm)ESP_LOGW("converter", "%s", "forced_pwm");

        auto L0 = coilConf.getFloat("L0");

        ESP_LOGI("converter", "Coil L0=%.1f µH", L0 * 1e6f);

        uint32_t pwmFrequency = pinConf.getLong("pwm_freq"); //39000; //  converter switching frequency
        assert_throw(pwmFrequency > 5e3 && pwmFrequency < 5e5, "");

        fL = (float) pwmFrequency * L0 * InductivityDcBias; // for ripple current computation
        assert_throw(fL < 20, "pwmFreq*L0 out-of-range");
        assert_throw(fL > 1, "pwmFreq*L0 out-of-range");
        // Lo: https://www.ti.com/lit/ds/symlink/lm5163.pdf#page=18

        auto drvInpLogic = pinConf.getString("pwm_driver_logic"); // driver input logic "in,en", "hi,li" and en
        uint8_t pinCtrl, pinRect;


        if (drvInpLogic == "InEn") { // e.g. Infineon ir2814

            pwmEnLogic = true;
            auto pnCtrl = isBoost ? "pwm_en" : "pwm_in";
            auto pnRect = isBoost ? "pwm_in" : "pwm_en";

            pinCtrl = pinConf.getByte(pnCtrl);
            pinRect = pinConf.getByte(pnRect);
            assert_throw(pinCtrl != pinRect, "");

            if (!pinConf.getByte("skip_assert", 0)) {
                // ti, infineon gate drivers: in pins pulled low, EN/SD pins pulled high
                assertPinState(pinCtrl, false, pnCtrl, true);
                assertPinState(pinRect, false, pnRect, true);
            }

        } else if (drvInpLogic == "HiLi") {  // e.g. TI UCC21330x with optional DIS pin (SD pin)
            auto pnCtrl = isBoost ? "pwm_li" : "pwm_hi";
            auto pnRect = isBoost ? "pwm_hi" : "pwm_li";

            pinCtrl = pinConf.getByte(pnCtrl);
            pinRect = pinConf.getByte(pnRect);
            pinSd = pinConf.getByte("pwm_sd", 255); // DIS

            assert_throw(pinCtrl != pinRect, "");
            assert_throw(pinCtrl != pinSd, "");
            assert_throw(pinRect != pinSd, "");

            if (!pinConf.getByte("skip_assert", 0)) {
                assertPinState(pinCtrl, false, pnCtrl, false);
                assertPinState(pinRect, false, pnRect, false);
                if (pinSd != 255) assertPinState(pinSd, true, "pwm_sd", false);
            }

        } else {
            throw std::runtime_error("unrecognized pwm_driver_logic " + drvInpLogic);

        }

        pwmDriver.init_pwm(pwmCh_Ctrl, pinCtrl, pwmFrequency);
        pwmDriver.init_pwm(pwmCh_Rect, pinRect, pwmFrequency);

        if (pinSd != 255) {
            pinMode(pinSd, OUTPUT);
            digitalWrite(pinSd, 1);
        }

        pwmRectMin = std::ceil(
                (float) pwmDriver.pwmMax * (isBoost ? 0.f : MinDutyCycleLS)); // keeping the bootstrap circuit powered
        pwmCtrlMax = (uint16_t) (pwmDriver.pwmMax - pwmRectMin);
        pwmCtrlMin = 1; //isBoost ? 0 : 0;
        // note that mosfets have different Vg(th) and switching times worst case is Vi/o=80/12
        // ^ set pwmMinHS a bit lower than pwmMinLS (might cause no-load output over-voltage otherwise)

        ESP_LOGI("converter", "f=%lu, boost=%d, pwmDriver.pwmMax=%hu, pwmMinLS=%hu, pwmMinHS=%hu, pwmMaxHS=%hu",
                 pwmFrequency, isBoost, pwmDriver.pwmMax, pwmRectMin, pwmCtrlMin, pwmCtrlMax);
    }

    void computePwmRectMax() {
        // update pwmRectMax for DCM or DCM case
        if (dcmHysteresis) {
            // DCM
            pwmRectMax = (uint16_t) std::round((float) pwmCtrl * pwmRectRatioDCM);
        } else {
            // CCM
            pwmRectMax = pwmDriver.pwmMax - pwmCtrl;
        }
        pwmRectMax = std::min<uint16_t>(std::max(pwmRectMin, pwmRectMax), pwmDriver.pwmMax - pwmCtrl);
    }

    void pwmPerturb(int16_t direction) {

        if (unlikely(disabled() and direction > 0 and pinSd != 255)) {
            UART_LOG("Converter enabled");
            digitalWrite(pinSd, 0);
        }

        pwmCtrl = constrain(pwmCtrl + direction, pwmCtrlMin, pwmCtrlMax);


        bool largerDecrease = (-direction > pwmDriver.pwmMax / 50);

        // update pwmRect block
        {
            if (largerDecrease && !forcedPwm)
                // assume DCM as it will always give equal or less pwmRectMax
                dcmHysteresis = true;

            computePwmRectMax();

            if (largerDecrease && !forcedPwm) {
                // we don't know if we end up in CCM/DCM, so set rect duty cycle to min
                // let the converter and sensors converge
                if (pwmRect > pwmRectMin + (pwmDriver.pwmMax / 64))
                    UART_LOG("Set pwmLS %hu -> pwmRectMin=%hu\n", pwmRect, pwmRectMin);
                pwmRect = pwmRectMin;
            } else {
                if (pwmRect - pwmRectMax > (pwmDriver.pwmMax / 16)) {
                    // report if rect duty cycle is significantly above upper limit
                    UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu\n", pwmRect, pwmRectMax);
                }

                // "fade-in" the low-side duty cycle
                // start slowly, then quickly step towards pwmRectMax
                auto step = 1 + ((pwmRect > pwmRectMin + 32) ? (pwmRectMax - pwmRect) / 64 : 0);
                pwmRect = syncRectEnabled ? constrain(pwmRect + step, pwmRectMin, pwmRectMax)
                                          : pwmRectMin;
            }
        }


        if (largerDecrease) {
            /*
             * with larger decreases of duty cycle do a "safe" update
             */
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_Rect, 0);
                pwmDriver.update_pwm(pwmCh_Ctrl, pwmCtrl);
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            } else {
                if (pinSd != 255) digitalWrite(pinSd, 1);
                else pwmDriver.update_pwm(pwmCh_Rect, 0);
                pwmDriver.update_pwm(pwmCh_Ctrl, pwmCtrl);
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
                if (pinSd != 255) digitalWrite(pinSd, 0);
            }
        } else if (direction < 0) {
            /*
             * update EN before IN
             * pwmHS will be smaller than before. after the EN update (we don't know the direction) pwmHS stays at the
             * old value (larger) causing the effective pwmLS to be less, which is ok
             */
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            } else {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
            }
            pwmDriver.update_pwm(pwmCh_Ctrl, pwmCtrl);
        } else {
            /* update IN before EN
             * we increase pwmHS here, so update it first
             */
            pwmDriver.update_pwm(pwmCh_Ctrl, pwmCtrl);
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            } else {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
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
        assert(std::abs(directionFloat) <= pwmDriver.pwmMax);

        directionFloat += directionFloatBuffer;
        directionFloatBuffer = 0;
        auto directionInt = (int16_t) (directionFloat);
        if (directionInt != 0)
            pwmPerturb(directionInt);
        directionFloatBuffer += directionFloat - (float) directionInt;
    }


    void disable() {
        if (pinSd != 255)digitalWrite(pinSd, 1);
        //pwmDriver.update_pwm(pwmCh_Ctrl, 0);
        //pwmDriver.update_pwm(pwmCh_Rect, 0);

        pwmDriver.stop(pwmCh_Ctrl, 0);
        pwmDriver.stop(pwmCh_Rect, 0);

        if (pwmCtrl > pwmCtrlMin)
            UART_LOG("PWM disabled (duty cycle was %d)\n", (int) pwmCtrl);

        pwmCtrl = 0;
        pwmRect = 0;
        syncRectEnabled = false;
    }


    [[nodiscard]] inline float rippleCurrent(float hv, float lv) const {
        // this does not consider dc bias, just a constant 0.95 inductivity factor
        return lv / fL * (1.0f - lv / hv);
    }

    /**
     *
     * @param vh higher-voltage side (buck: Vin)
     * @param vl lower-voltage side (buck: Vout)
     * @param il inductor dc current
     */
    [[nodiscard]] bool computeDCM(float vh, float vl, float il) {
        auto ir = rippleCurrent(vh, vl);
        auto dcm = ir > il * (dcmHysteresis ? 1.8f : 2.f) || il < 0.1f; // TODO: il < 0.1f
        if (forcedPwm) dcm = false;
        if (dcm != dcmHysteresis) {
            dcmHysteresis = dcm;
            UART_LOG("converter: %s -> %s (M=%.2f, I=%.2f, ∆I/2=%.2f, pwm=%hu)",
                     dcm ? "CCM" : "DCM", dcm ? "DCM" : "CCM",
                     isBoost ? (vh / vl) : (vl / vh), il, ir * .5f, pwmCtrl);
        }
        return dcm;
    }

    /*!
      * @brief  Compute t_rect/t_ctrl ratio for DCM
      *
      * Used in DCM to limit t_on for the rect switch to prevent reverse coil current (forced PWM).
      * For the buck converter it is t_onLS/t_onHS. (HS=Ctrl)
      * For the boost converter t_onHS/t_onLS. (LS=Ctrl)
      * See doc/Diode Emulation.rst.
      *
      * @param m converter voltage ratio
      * @return pwmRect/pwmCtrl ratio
      */
    [[nodiscard]] float rectCtrlRatio(float m) const {
        return isBoost ? (1.f / (m - 1.f)) : (1.f / m - 1.f);
    }

    [[nodiscard]] bool inDCM() const {
        return dcmHysteresis;
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
     * by Fabian S. (fl4p) https://github.com/fl4p/fugu-mppt-firmware/
     *
     * @param pwmCtrl Duty cycle of the ctrl switch (buck:HS, boost:LS)
     * @param pwmMax The maximum duty cycle value
     * @param voltageRatio Vout/Vin ratio, D (the greater, the safer but less efficient)
     * @return
     */
    void computeSyncRectRatio(float vh, float vl, float il) {

        if (computeDCM(vh, vl, il)) {
            // computation of t_onCtrl and t_onRect ratio is quite error sensitive
            // e.g. at VR=0.64 a -5% error causes a 13% deviation of pwmMaxLs !
            // so we do some proper error computation here
            constexpr float voltageMaxErr = 0.01f; // inc -> safer, less efficient

            // WCEF = worst case error factor
            // compute the worst case error (Vout estimated too low, Vin too high => VR estimated too low)
            // this is true for buck and boost
            constexpr float voltageRatioWCEF = (1.f - voltageMaxErr) / (1.f + voltageMaxErr); // < 1.0


            const float convRatioWCE =
                    (isBoost
                     ? ((vh > vl && vl > 0.1f) ? constrain(vh / vl, 1e-2f, 10.f) : 10.0f) // boost
                     : ((vh > vl && vh > 0.1f) ? constrain(vl / vh, 1e-2f, 1.f - 1e-2f) : 1.0f) //buck
                    ) / voltageRatioWCEF;

            outInVoltageRatio = convRatioWCE;
            if (il < 0.01f || vl < 1.0f) // TODO get rid of magic constants
            {
                if (pwmRectRatioDCM > 0.2f)
                    ESP_LOGI("converter", "Disable sync rect, low coil current (%.2f) or low volt (%.2f) pwm=%hu", il,
                             vl, getCtrlOnPwmCnt());
                pwmRectRatioDCM = 0.0f;
            } else {
                pwmRectRatioDCM = rectCtrlRatio(convRatioWCE);
                //if(dcmHysteresis)pwmRectRatioDCM = min(pwmRectRatioDCM, 1.5f); // TODO
            }
            //pwmMaxRect = (uint16_t) std::round( * (float) pwmCtrl);

            // TODO remove:
            // compute worst-case error at current working point
            /*const float voltageRatio = vl / vh;
            const float pwmMaxLsWCEF = (1 / (voltageRatio * voltageRatioWCEF) - 1) / (1 / voltageRatio - 1); // > 1
            const float pwmMaxRectWCEF = rectCtrlRatio(voltageRatio * voltageRatioWCEF) / rectCtrlRatio(voltageRatio);
            if (!isBoost) {
                assert(abs(pwmMaxLsWCEF - pwmMaxRectWCEF) < 0.01);
            }*/

        }
    }


    const float &updateSyncRectMaxDuty(float vin, float vout, float il) {
        auto &vh(isBoost ? vout : vin);
        auto &vl(isBoost ? vin : vout);

        computeSyncRectRatio(vh, vl, il);
        computePwmRectMax();

        if (pwmRect > pwmRectMax) {
            if (pwmRect - pwmRectMax > (pwmDriver.pwmMax / 40)) {
                UART_LOG("Set pwmLS %hu -> pwmMaxLS=%hu (VR=%.3f)", pwmRect, pwmRectMax, outInVoltageRatio);
            }
            pwmRect = pwmRectMax;
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect); // instantly commit if limit decreases
            } else {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
            }
        }

        return outInVoltageRatio;
    }


    /**
     * Permanently set enable/disable low-side switch
     * @param enable
     */
    void enableSyncRect(bool enable, bool overwriteFPWM = false) {
        if (forcedPwm && !overwriteFPWM) enable = true;
        if (enable != syncRectEnabled) {
            UART_LOG("Sync rect %s", enable ? "enabled" : "disabled");
        }
        syncRectEnabled = enable;
        if (!enable)
            syncRectMinDuty();
    }

    /**
     * Set rect sync duty cycle to minimum (buck: disable power conduction, just keep the bootstrapping powered for HS drive)
     * Notice that this is only needed if no inductor is connected to the half-bridge, as the inductor will push the
     * switch node voltage until LS diode conducts.
     * Boost converter has a min duty cycle of 0 (HS)
     */
    void syncRectMinDuty() {
        if (pwmRect > pwmRectMin) {
            if (pwmRect > pwmRectMin + pwmRectMin / 2) {
                ESP_LOGW("dcdc", "set sync-rect PWM to minimum %hu -> %hu (vRatio=%.3f, pwmMaxLS=%hu)", pwmRect,
                         pwmRectMin, outInVoltageRatio, pwmRectMax);
            }
            pwmRect = pwmRectMin;
            if (pwmEnLogic) {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl + pwmRect);
            } else {
                pwmDriver.update_pwm(pwmCh_Rect, pwmCtrl, pwmRect);
            }
        }
    }

    void shortLs() {
        disable();

        auto pwmChLS = boost() ? pwmCh_Ctrl : pwmCh_Rect;
        auto pwmChHS = !boost() ? pwmCh_Ctrl : pwmCh_Rect;
        pwmDriver.stop(pwmChLS, 1);
        pwmDriver.stop(pwmChHS, 0);
        if (pinSd != 255)digitalWrite(pinSd, 0);
    }
};