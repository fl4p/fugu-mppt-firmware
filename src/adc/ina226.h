
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


    bool init(const ConfFile &pinConf) override  {
        if (ina226_instance) {
            return false;
        }

        auto alertPin = (uint8_t) pinConf.getByte("ina22x_alert", 0);
        if  (alertPin == 0) {
            ESP_LOGW("ina22x", "No ALERT pin specified");
            return false;
        }

        auto addr = pinConf.getByte("ina22x_addr", 0b1000000);

        if (!i2c_test_address(addr)) {
            ESP_LOGI("ina226", "Chip didnt respond at address 0x%02hhX", addr);
            return false;
        }

        i2c_port_t i2c_port = (i2c_port_t)pinConf.getByte("i2c_port", 0);


        auto mfrId = i2c_read_short(i2c_port, addr, INA22x_MANUFACTURER_ID_CMD);
        auto deviceId = i2c_read_short(i2c_port, addr, INA22x_DEVICE_ID_CMD);

        ESP_LOGI("ina22x", "MfrID: 0x%04X, DeviceID: 0x%04X", mfrId, deviceId);

        if (deviceId != 0x2260) {
            ESP_LOGW("ina226", "This is not an INA226 device!");
            return false;
        }

        if (!ina226.init())
            return false;

        //ina226.setAverage(AVERAGE_16);
        //ina226.setConversionTime(CONV_TIME_204, CONV_TIME_140);

        /**
         * - conversion time should be 1x - 10x of (1/f_cutoff) with f_cutoff being the analog RC-filter cutoff freq (aliasing)
         * - averaging eliminates aliasing due to i2c sampling of the ADC registers (U & I)
         */

        ina226.setAverage(AVERAGE_64);
        ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_140);

        float resistor = 1e-3f, range = 35.0f;// default: 1mOhm, 80A (ina226 shunt voltage range is 81.92mV)
        ina226.setResistorRange(resistor, range);

        ina226.enableConvReadyAlert();

        ina226_instance = this;



        pinMode(alertPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(alertPin), ina226_alert, FALLING);
        ESP_LOGI("ina226", "Setup ALERT interrupt pin %hhu", alertPin);

        if (!intAdc.init(pinConf))
            return false;
        intAdc.startReading((uint8_t) pinConf.getByte("ina226_aux_ch"));

        ina226.setMeasureMode(CONTINUOUS);

        return true;
    }


    void startReading(uint8_t channel) override {
        readChannel = channel;
    }

    void alertNewDataFromISR() { new_data = true; }

    bool hasData() override {
        return true;

        if (!new_data)
            return false;

        ina226.readAndClearFlags();
        if (ina226.overflow) {
            ESP_LOGW("ina226", "Overflow!");
        }
        if (ina226.limitAlert) {
            ESP_LOGW("ina226", "Limit Alert!");
        }

        return ina226.convAlert;
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

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        if (ch == ChAux) {
            ESP_LOGI("ina226", "internal ADC setMaxExpectedVoltage(%i,%.6f)", ch, voltage);
            intAdc.setMaxExpectedVoltage(intAdc.readingChannel, voltage);
        }
    }

    float getInputImpedance(uint8_t ch) override {
        if(ch == ChAux) {
            return intAdc.getInputImpedance(intAdc.readingChannel);
        } else {
            return 830e3; // 830k ina226 input impedance
        }
    }
};

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR ina226_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
