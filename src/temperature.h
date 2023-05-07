#pragma once

// #include "driver/temperature_sensor.h"

#include "driver/temp_sensor.h"

class Esp32TempSensor {
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

        return tsens_out;
    }
};