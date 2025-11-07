
#include <INA226_WE.h>

//#include "etc/pinconfig.h"
#include "sampling.h"
//#include "ina228.h"

#include "i2c.h"
#include "conf.h"
#include "etc/rt.h"

class ADC_INA226;

void ina226_alert();

ADC_INA226 *ina226_instance = nullptr;

int console_read_usb(char *buf, size_t len); //debug

class ADC_INA226 : public AsyncADC<float> {
    INA226_WE ina226;
    volatile bool new_data = false;
    bool overflow = false;
    TaskNotification taskNotification{};
    uint8_t readChannel = 0;
    uint8_t alertPin;

public:

    static constexpr auto ChVBus = 0, ChI = 1;
    static constexpr auto INA22x_MANUFACTURER_ID_CMD = 0x3E;
    static constexpr auto INA22x_DEVICE_ID_CMD = 0x3F;

    [[nodiscard]] SampleReadScheme scheme() const override {
        //
        return SampleReadScheme::all;
    }


    bool init(const ConfFile &pinConf) override {
        if (ina226_instance) {
            return false;
        }

        alertPin = (uint8_t) pinConf.getByte("ina22x_alert", 255);
        if (alertPin == 255) {
            ESP_LOGW("ina22x", "No ALERT pin specified");
            return false;
        }

        pinMode(alertPin, INPUT_PULLUP);  // esp32s3 has 45k internal pull up

        ina226_instance = this;
        attachInterrupt(digitalPinToInterrupt(alertPin), ina226_alert, FALLING);
        ESP_LOGI("ina22x", "Setup ALERT interrupt pin %hhu", alertPin);

        assert(!new_data);

        auto i2c_port = (i2c_port_t) pinConf.getByte("i2c_port", 0);
        auto addr = pinConf.getByte("ina22x_addr", 0b1000000);

        assert_throw(i2c_port == 0, ""); // not implemented, see _setupPeripherals()
        //if(i2c_port != 0)
        //    ina226._wire = new TwoWire((uint8_t) i2c_port);
        ina226.i2cAddress = addr;

        float resistor = pinConf.getFloat("ina22x_resistor"),
                range = pinConf.getFloat("ina22x_range",
                                         35.0f);// default: 1mOhm, 80A (ina226 shunt voltage range is 81.92mV)
        ina226.setResistorRange(resistor, range);
        //ESP_LOGI("ina226", "ina226.calVal=%d", ina226.calVal);

        if (!resetPeripherals())
            return false;

        //ESP_LOGI("ina226", "ina226.calVal2=%d", ina226.calVal);
        //ina226.setResistorRange(resistor, range);
        //ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);

        return true;
    }

    float getSamplingRate(uint8_t channel) override {
        return 451.f;
    }

    bool resetPeripherals() override {
        if (!i2c_test_address(ina226.i2cAddress)) {
            ESP_LOGI("ina22x", "Chip didnt respond at address 0x%02hhX", (uint8_t) ina226.i2cAddress);
            return false;
        }

        i2c_port_t i2c_port = I2C_NUM_0; // TODO
        auto addr = (uint8_t) ina226.i2cAddress;

        bool ok = false;
        auto mfrId = ina226.readRegister(INA22x_MANUFACTURER_ID_CMD, ok);
        assert_throw(ok, "");
        auto deviceId = ina226.readRegister( INA22x_DEVICE_ID_CMD, ok);
        assert_throw(ok, "");
        ESP_LOGI("ina22x", "MfrID: 0x%04X, DeviceID: 0x%04X", mfrId, deviceId);

        if (deviceId != 0x2260) {
            ESP_LOGW("ina22x", "This is not an INA226 device!");
            return false;
        }


        ina226.reset_INA226();         //in case the device is already/still initialized

        vTaskDelay(2);

        new_data = false;

        try {
            assertPinState(alertPin, true, "ina22x_alert", false);

            assert_throw(!new_data, "unexpected alert signal");

            //if (!ina226.init())
            //    return false;

            ina226.setAverage(AVERAGE_1);
            ina226.setConversionTime(CONV_TIME_1100);
            ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);

            assert_throw(!new_data, "");
        } catch (const std::exception &ex) {
            ESP_LOGE("ina22x", "error %s", ex.what());
            return false;
        }

        if (!testConvReadyAlert(addr, alertPin)) {
            return false;
        }

        if (!testContinuousAlert()) {
            return false;
        }


        //if (!testConvReadyAlert(addr, alertPin)) {
        //    return false;
        //}

        //ina226.setAverage(AVERAGE_16);
        //ina226.setConversionTime(CONV_TIME_204, CONV_TIME_140);


        /**
         * - conversion time should be > 1x - 10x of (1/f_cutoff) with f_cutoff being the analog RC-filter cutoff freq (aliasing)
         * - averaging eliminates aliasing due to i2c sampling of the ADC registers (U & I)
         */

        //ina226.setAverage(AVERAGE_64);
        //ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_140);
        ina226.setAverage(AVERAGE_1);
        ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_1100);
        // TODO adjust hasData() timeout !
        //ina226.setConversionTime(CONV_TIME_588, CONV_TIME_588);


        ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();
        ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);

        return true;
    }

    void deinit() override {
        if (alertPin != 255)
            detachInterrupt(digitalPinToInterrupt(alertPin));
    }

    /*void update() {
        //ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();
    } */

    void debugMode() {
        bool debug = true;
        if (debug) ESP_LOGI("ina22x", "Entering debug console");
        while (debug) {
            char buf[10];
            auto r = console_read_usb(buf, sizeof buf);
            if (r == 1) {
                buf[r] = 0;
                //
                bool ok = true;
                if (buf[0] == 'r') ina226.reset_INA226();
                else if (buf[0] == 'm') ina226.readAndClearFlags();
                else if (buf[0] == 'a') ina226.enableConvReadyAlert();
                else if (buf[0] == 'c') ina226.setMeasureMode(CONTINUOUS);
                else if (buf[0] == 't') ina226.setMeasureMode(TRIGGERED);
                else if (buf[0] == 'l') ina226.enableAlertLatch();
                else if (buf[0] == 'v') ESP_LOGI("ina22x", "V=%.3f", ina226.getBusVoltage_V());
                else if (buf[0] == 'x') break;
                else ok = false;
                if (ok) ESP_LOGI("ina22x", "console cmd %s", buf);
                //else ESP_LOGW("ina225", "unknown");
            }
            vTaskDelay(1);
        }
        ESP_LOGI("ina22x", "exit debug mode");
    }

    bool testConvReadyAlert(uint8_t addr, uint8_t alertPin) {

        ina226.setAverage(AVERAGE_4);
        ina226.setConversionTime(CONV_TIME_140, CONV_TIME_140);
        ina226.setMeasureMode(TRIGGERED);

        delay(1);
        new_data = false;
        delay(5);
        if (new_data) {
            ESP_LOGW("ina22x", "unexpected new data");
            debugMode();
        }
        //assert(!new_data);

        if (digitalRead(alertPin) != HIGH) {
            ESP_LOGE("ina22x", "Alert pin %hhu not pulled up, short to ground?", alertPin);
            return false;
        }


        //auto bus = (i2c_port_t) 0;

        ina226.startSingleMeasurement();
        assert(!new_data); // test disabled ConvReadyAlert

        ina226.enableConvReadyAlert();
        assert(!new_data);
        ina226.startSingleMeasurement();
        //assert(new_data); // TODO enable!adc-reset-peripherals test enabled ConvReadyAlert
        new_data = false;


        //auto confReg = i2c_read_short(bus, addr, INA226_WE::INA226_CONF_REG, true, 100);
        bool ok;
        auto confReg = ina226.readRegister(INA226_WE::INA226_CONF_REG, ok);
        assert_throw(ok, "");

        assert(!new_data);

        auto t0 = micros();

        // trigger a new measurement (see https://www.ti.com/lit/ds/symlink/ina226.pdf#page=11 ):
        //i2c_write_short(bus, addr, INA226_WE::INA226_CONF_REG, confReg);
        ina226.writeRegister(INA226_WE::INA226_CONF_REG, confReg);
        ina226.startSingleMeasurementNoWait();
        auto tWrite = micros();

        while (!new_data) {} // busy wait
        auto tBusyWait = micros();

        ina226.readAndClearFlags();

        /*auto maskEnReg = i2c_read_short(bus, addr, INA226_WE::INA226_MASK_EN_REG, true, 100);

        // TODO read multi reg
         // this is not faster
        bool overflow = (maskEnReg >> 2) & 0x0001;
        bool convAlert = (maskEnReg >> 3) & 0x0001;
        bool limitAlert = (maskEnReg >> 4) & 0x0001;
        assert(convAlert);
        assert(!overflow);
        assert(!limitAlert);*/

        auto tRead = micros();

        ina226.getCurrent_A();
        ina226.getBusVoltage_V();

        //std::array<uint16_t, 2> iuReg;
        //assert(i2c_read_short2(bus, addr, {INA226_WE::INA226_CURRENT_REG, INA226_WE::INA226_BUS_REG, }, iuReg, 100));
        //float current = (static_cast<int16_t>(iuReg[0]) / ina226.currentDivider_mA) * 1e-3f;
        //float voltage = (float) iuReg[1] * 0.00125f;

        auto tReadI = micros();

        //ESP_LOGI("ina", "current %f %f", ina226.getCurrent_A(), current);
        //ESP_LOGI("ina", "voltage %f %f", ina226.getBusVoltage_V(), voltage);
        //assert(ina226.getCurrent_A() == current);
        //assert(ina226.getBusVoltage_V() == voltage);

        // the busyWait time is ~ (convTime_I + convTime_U) * AvgSamples
        assert ((tBusyWait - tWrite) < (100 + (140 + 10) * 2 * 4));

        ESP_LOGI("ina22x", "Single-shot timings: sendTrigger=%lu busyWait=%lu read=%lu tReadIV=%lu (us)",
                 tWrite - t0, tBusyWait - tWrite, tRead - tBusyWait, tReadI - tRead);


        //ina226.getCurrent_mA(); // todo CONFIG_DISABLE_HAL_LOCKS




        return true;
    }

    bool testContinuousAlert() {
        new_data = false;

        //ESP_LOGI("ina22x", "testContinuousAlert");
        ina226.reset_INA226();
        vTaskDelay(1);
        ina226.enableConvReadyAlert();
        auto regME = ina226.readRegister(INA226_WE::INA226_MASK_EN_REG);
        assert(!(regME & 0x0001)); // alert latch disabled

        //debugMode();

        regME = ina226.readRegister(INA226_WE::INA226_MASK_EN_REG);
        assert(!(regME & 0x0001)); // alert latch disabled

        auto t0 = micros();
        new_data = false;
        while (!new_data) { if (micros() - t0 > 100000) return false; } // busy wait
        //ESP_LOGI("ina22x", "alert pass default");

        ina226.readAndClearFlags();

        new_data = false;
        t0 = micros();
        while (!new_data) { if (micros() - t0 > 100000) return false; } // busy wait
        //ESP_LOGI("ina22x", "alert pass default2");

        ina226.setAverage(AVERAGE_1);
        ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_1100);
        ina226.setMeasureMode(CONTINUOUS);
        ina226.readAndClearFlags();
        t0 = micros();
        while (!new_data) { if (micros() - t0 > 100000) return false; } // busy wait
        //ESP_LOGD("ina22x", "Continuous 1st");
        ina226.readAndClearFlags();
        new_data = false;
        t0 = micros();
        while (!new_data) { if (micros() - t0 > 100000) return false; } // busy wait
        auto t1 = micros();
        ESP_LOGD("ina22x", "Continuous 2nd");

        ESP_LOGD("ina22x", "Continuous timings: busyWait=%lu (us)", t1 - t0);

        return true;
    }


    void startReading(uint8_t channel) override {
        assert_throw(channel <= 1, "");
        taskNotification.subscribe();
        readChannel = channel;
    }

    void alertNewDataFromISR() {
        new_data = true;
        taskNotification.notifyFromIsr();
    }

    bool hasData() override {
        static uint32_t numTimeouts = 0;
        if (!new_data && !taskNotification.wait(3)) {
            ++numTimeouts;
            if (numTimeouts % 2000 == 0) {
                ESP_LOGE("ina22x", "%lu timeout!", numTimeouts);
            }
        }

        // we might get a task notification from other ADCs
        if (!new_data)
            return false;

        new_data = false;

        uint16_t value = ina226.readRegister(INA226_WE::INA226_MASK_EN_REG);
        overflow = (value >> 2) & 0x0001;
        bool convAlert = (value >> 3) & 0x0001;
        bool limitAlert = (value >> 4) & 0x0001;

        //ina226.readAndClearFlags();
        if (overflow) {
            ESP_LOGW("ina22x", "Overflow!");
        }
        if (limitAlert) {
            ESP_LOGW("ina22x", "Limit Alert!");
        }

        //if (convAlert) {
        // the MASK_EN_REG register clears when reading
        //value |= 0x0400; // enableConvReadyAlert
        //ina226.writeRegister(INA226_WE::INA226_MASK_EN_REG, value);
        //}

        return convAlert;
    }

    float getSample() override {
        bool success;
        switch (readChannel) {
            case ChVBus: {
                uint16_t raw = ina226.readRegister(INA226_WE::INA226_BUS_REG, success);
                if (!success) {
                    ESP_LOGW("ina22x", "failure reading register");
                    return NAN;
                }
                if (scope) scope->addSample12(this, readChannel, raw / 4);
                return (float) raw * 0.00125f;
            }
            case ChI: {
                auto raw = static_cast<int16_t>(ina226.readRegister(INA226_WE::INA226_CURRENT_REG, success));
                if (!success) {
                    ESP_LOGW("ina22x", "failure reading register");
                    return NAN;
                }
                if (scope) scope->addSample12(this, readChannel, std::abs(raw) / 3 /*abs(i16)->12bit*/);
                auto amp = ((float) raw / ina226.currentDivider_mA / 1000.f);
                if (unlikely(overflow)) ESP_LOGW("ina22x", "Overflow current = %.2fA", amp);
                return amp;
            }
            default:
                assert(false);
        }
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        if (ch == ChVBus) {
            assert_throw(voltage <= 36, "");
        } else if (ch == ChI) {
            ESP_LOGW("ina22x", "Check shunt voltage range! %.2f", voltage);
        } else {
            assert(false);
        }
    }

    float getInputImpedance(uint8_t ch) override {
        return 830e3; // 830k ina226 input impedance
    }
};


void IRAM_ATTR ina226_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
