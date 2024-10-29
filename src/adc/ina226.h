
#include <INA226_WE.h>

#include "pinconfig.h"
#include "sampling.h"
//#include "ina228.h"

#include "i2c.h"
#include "store.h"

class ADC_INA226;


void ina226_alert();

ADC_INA226 *ina226_instance = nullptr;

#define INA22x_MANUFACTURER_ID_CMD    0x3E
#define INA22x_DEVICE_ID_CMD          0x3F

class ADC_INA226 : public AsyncADC<float> {
    INA226_WE ina226;
    volatile bool new_data = false;

    uint8_t readChannel = 0;

    ADC_ESP32 intAdc;

public:

    static constexpr auto ChVBus = 0, ChI = 1, ChAux = 2;


    bool init(const ConfFile &pinConf) override {
        if (ina226_instance) {
            return false;
        }

        auto alertPin = (uint8_t) pinConf.getByte("ina22x_alert", 0);
        if (alertPin == 0) {
            ESP_LOGW("ina22x", "No ALERT pin specified");
            return false;
        }

        // esp32s3 has 45k internal pull up
        pinMode(alertPin, INPUT_PULLUP);


        ina226_instance = this;
        attachInterrupt(digitalPinToInterrupt(alertPin), ina226_alert, FALLING);
        ESP_LOGI("ina226", "Setup ALERT interrupt pin %hhu", alertPin);

        assert(!new_data);

        auto addr = pinConf.getByte("ina22x_addr", 0b1000000);
        if (!i2c_test_address(addr)) {
            ESP_LOGI("ina226", "Chip didnt respond at address 0x%02hhX", addr);
            return false;
        }

        i2c_port_t i2c_port = (i2c_port_t) pinConf.getByte("i2c_port", 0);

        auto mfrId = i2c_read_short(i2c_port, addr, INA22x_MANUFACTURER_ID_CMD);
        auto deviceId = i2c_read_short(i2c_port, addr, INA22x_DEVICE_ID_CMD);

        ESP_LOGI("ina22x", "MfrID: 0x%04X, DeviceID: 0x%04X", mfrId, deviceId);

        if (deviceId != 0x2260) {
            ESP_LOGW("ina226", "This is not an INA226 device!");
            return false;
        }

        assert(!new_data);

        if (!ina226.init())
            return false;

        assert(!new_data);

        if (!testConvReadyAlert(addr, alertPin)) {
            return false;
        }


        if (!intAdc.init(pinConf))
            return false;
        intAdc.startReading((uint8_t) pinConf.getByte("ina226_aux_ch"));

        //ina226.setAverage(AVERAGE_16);
        //ina226.setConversionTime(CONV_TIME_204, CONV_TIME_140);

        float resistor = 1e-3f, range = 35.0f;// default: 1mOhm, 80A (ina226 shunt voltage range is 81.92mV)
        ina226.setResistorRange(resistor, range);

        /**
         * - conversion time should be 1x - 10x of (1/f_cutoff) with f_cutoff being the analog RC-filter cutoff freq (aliasing)
         * - averaging eliminates aliasing due to i2c sampling of the ADC registers (U & I)
         */

        //ina226.setAverage(AVERAGE_64);
        //ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_140);
        ina226.setAverage(AVERAGE_1);
        ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_1100);

        ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();

        return true;
    }

    void update() {
        //ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();
    }

    bool testConvReadyAlert(uint8_t addr, uint8_t alertPin) {

        ina226.setAverage(AVERAGE_4);
        ina226.setConversionTime(CONV_TIME_140, CONV_TIME_140);
        ina226.setMeasureMode(TRIGGERED);

        new_data = false;
        delay(5);
        assert(!new_data);

        if (digitalRead(alertPin) != HIGH) {
            ESP_LOGE("ina22x", "Alert pin %hhu not pulled up, short to ground?", alertPin);
            return false;
        }


        auto bus = (i2c_port_t) 0;

        ina226.startSingleMeasurement();
        assert(!new_data); // test disabled ConvReadyAlert

        ina226.enableConvReadyAlert();
        ina226.startSingleMeasurement();
        assert(new_data); // test enabled ConvReadyAlert
        new_data = false;


        auto confReg = i2c_read_short(bus, addr, INA226_WE::INA226_CONF_REG, true, 100);

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

        ESP_LOGI("ina22x", "Timings: sendTrigger=%lu busyWait=%lu read=%lu tReadIV=%lu (us)",
                 tWrite - t0, tBusyWait - tWrite, tRead - tBusyWait, tReadI - tRead);


        //ina226.getCurrent_mA(); // todo CONFIG_DISABLE_HAL_LOCKS

        return true;
    }


    void startReading(uint8_t channel) override {
        readChannel = channel;
    }

    void alertNewDataFromISR() { new_data = true; }

    bool hasData() override {
        if (!new_data)
            return false;
        new_data = false;

        uint16_t value = ina226.readRegister(INA226_WE::INA226_MASK_EN_REG);
        bool overflow = (value >> 2) & 0x0001;
        bool convAlert = (value >> 3) & 0x0001;
        bool limitAlert = (value >> 4) & 0x0001;

        //ina226.readAndClearFlags();
        if (overflow) {
            ESP_LOGW("ina226", "Overflow!");
        }
        if (limitAlert) {
            ESP_LOGW("ina226", "Limit Alert!");
        }

        //if (convAlert) {
        // the MASK_EN_REG register clears when reading
        //value |= 0x0400; // enableConvReadyAlert
        //ina226.writeRegister(INA226_WE::INA226_MASK_EN_REG, value);
        //}

        return convAlert;

    }

    float getSample() override {
        switch (readChannel) {
            case ChVBus:
                return ina226.getBusVoltage_V();

            case ChI:
                return ina226.getCurrent_A();

            case ChAux:
                return intAdc.getSample();

            default:
                assert(false);
        }
    }

    void setMaxExpectedVoltage(uint8_t ch, float voltage)

    override {
        if (ch == ChAux) {
            ESP_LOGI("ina226", "internal ADC setMaxExpectedVoltage(%i,%.6f)", ch, voltage);
            intAdc.
                    setMaxExpectedVoltage(intAdc
                                                  .readingChannel, voltage);
        }
    }

    float getInputImpedance(uint8_t ch)

    override {
        if (ch == ChAux) {
            return intAdc.
                    getInputImpedance(intAdc
                                              .readingChannel);
        } else {
            return 830e3; // 830k ina226 input impedance
        }
    }

    [[nodiscard]] bool getAltogether() const

    override {
        return true;
    }
};

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR ina226_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
