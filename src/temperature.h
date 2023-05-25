#pragma once

class TempSensorGPIO_NTC {
    float ntcResistance = 10e3f;

    RunningMedian3<float> median3{};
    EWMA<float> ewma{80};

    uint8_t _attack = 60; // discard first samples


    float adc2Celsius(float adc) const {
        // invalid adc 4095
        float tl = log(ntcResistance * (4095.f / adc - 1.f));
        float temp = (1.f / (1.009249522e-3f + 2.378405444e-4f * tl + 2.019202697e-7f * tl * tl * tl)) - 273.15f;
        if (temp < -200 or temp > 300) {
            return NAN;
        }
        return temp;
    }

public:
    float read() {
        auto adc = analogRead((uint8_t) PinConfig::NTC);
        if (_attack) --_attack;
        else if (adc < 4080) {
            ewma.add(median3.next(adc));
        }

        float temp = adc2Celsius(ewma.get());

        if (isnan(temp) && !_attack) {
            ESP_LOGW("temp", "Invalid temp %.1f from ADC %f", temp, adc);
            _attack = 120;
        }

        return temp;
    }

    float last() const { return adc2Celsius(ewma.get()); }
};

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/temp_sensor.h"
class Esp32TempSensor {
    RunningMedian3<float> median3;
    EWMA<float> ewma{40};

    //temperature_sensor_handle_t temp_handle = NULL;
    float tsens_out = NAN;
    temp_sensor_config_t conf{.dac_offset = TSENS_DAC_L2, .clk_div = 6};
public:
    Esp32TempSensor() {

        /*temperature_sensor_config_t temp_sensor = {
                .range_min = 20,
                .range_max = 100,
        };
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));

        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
         */
        ESP_ERROR_CHECK(temp_sensor_set_config(conf));
        ESP_ERROR_CHECK(temp_sensor_start());
    }

    float read() {

        if (temp_sensor_read_celsius(&tsens_out) != ESP_OK) {
            conf.dac_offset = (temp_sensor_dac_offset_t) ((conf.dac_offset + 1) % TSENS_DAC_MAX);
            return NAN;
        }

        temp_sensor_dac_offset_t os = TSENS_DAC_L2;
        if (tsens_out > (conf.dac_offset != TSENS_DAC_L1 ? 75.f : 70.f)) {
            os = TSENS_DAC_L1;
        } else if (tsens_out < -5) {
            os = TSENS_DAC_L3;
        }

        if (os != conf.dac_offset) {
            ESP_LOGI("temp", "Updating temp dac offset %i", (int) conf.dac_offset);
            conf.dac_offset = os;
        }

        ewma.add(median3.next(tsens_out));

        return ewma.get();
    }

    float last() const { return ewma.get(); }
};
#else

#include "esp32-hal.h"

/*
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
 */



class Esp32TempSensor {
public:
    float _last = NAN;

    Esp32TempSensor() {}

    float read() {
        //return (temprature_sens_read() - 32) / 1.8;
        return (_last = temperatureRead());
    }

    float last() const { return _last; }
};

#endif


