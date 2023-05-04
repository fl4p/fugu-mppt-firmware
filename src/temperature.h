#pragma once

// #include "driver/temperature_sensor.h"

#include "driver/temp_sensor.h"

class Esp32TempSensor {
    //temperature_sensor_handle_t temp_handle = NULL;
public:
    Esp32TempSensor() {
        /*temperature_sensor_config_t temp_sensor = {
                .range_min = 20,
                .range_max = 100,
        };
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));

        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
         */
        ESP_ERROR_CHECK(temp_sensor_set_config(TSENS_CONFIG_DEFAULT()));
        ESP_ERROR_CHECK(temp_sensor_start());
    }

    float read() {
        float tsens_out;
        //ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
        ESP_ERROR_CHECK(temp_sensor_read_celsius(&tsens_out));
        return tsens_out;
    }
};