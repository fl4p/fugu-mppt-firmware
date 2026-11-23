#pragma once

#include <hal/adc_types.h>
#include <esp_timer.h>
#include <cmath>
#include "../util.h"

// Cached slow sensor with an explicit age, following the charger.h/BMS convention: a value that
// nothing refreshed must not be indistinguishable from a measured one. A starved refresh path or a
// failing sensor presents as stale; a sensor that is simply not fitted stays fresh-NaN, so the
// pre-existing isnan policies apply to it unchanged.
class SingleValueSensor {
    // coarseTicks() of esp_timer_get_time(): written on core 0, read from the RT loop on core 1, and a
    // 64-bit load/store is not atomic across cores on Xtensa.
    volatile uint32_t _lastGoodTicks = 0;
    volatile bool _everGood = false;
    volatile uint8_t _badRun = 0; // consecutive implausible reads

protected:
    virtual float readImpl() = 0; // raw sample, NaN on failure
    virtual void resetFilters() {}

    // False once the sensor is known to be absent (no NTC fitted, driver install failed). Such a
    // sensor stays permanently "fresh NaN" so the pre-existing isnan policies apply unchanged,
    // instead of decaying into a staleness report that no refresh could ever clear.
    [[nodiscard]] virtual bool present() const { return true; }

public:
    // Backstop only — the refresh runs every lfPeriod (3 s). Sized above the longest legitimate
    // core-0 stall (`sleep` caps at 60 s, an FTP passive-connect waits up to 2x30 s) so a healthy
    // board does not report staleness; the freezes this guards against lasted minutes.
    static constexpr uint32_t EXPIRE_S = 90;

    // Outside this range a reading is a sensor fault: an open NTC computes to -273 °C, a shorted one
    // to -273 °C or, if the ADC saturates below the rail, to a high but plausible value (`temp` trip).
    // BAD_READS implausible reads in a row (~9 s at lfPeriod) make the sensor stale at once.
    static constexpr float T_MIN = -40.f, T_MAX = 150.f;
    static constexpr uint8_t BAD_READS = 3;
    static bool plausible(float t) { return t >= T_MIN && t <= T_MAX; }

    virtual ~SingleValueSensor() = default;

    [[nodiscard]] bool fitted() const { return present(); }

    // Returns the raw sample just read (NaN if the read failed or was implausible), not last().
    // countBad: implausible reads count toward BAD_READS only on the regular lfPeriod read.
    float read(bool countBad = true) {
        // pre-stale history must not dominate last() once readings come back
        if (present() && !fresh()) resetFilters();
        float v = readImpl();
        // Stamp only on a valid new sample: the filters keep their old finite value after a failed
        // read, so stamping on last() would keep a failing sensor "fresh" forever.
        if (!present() || plausible(v)) {
            _badRun = 0;
            _everGood = true;
            _lastGoodTicks = coarseTicks(esp_timer_get_time());
        } else if (countBad && _badRun < BAD_READS) _badRun = _badRun + 1;
        return v;
    }

    [[nodiscard]] virtual float last() const = 0; // raw cache, may be stale

    [[nodiscard]] bool fresh() const {
        return _everGood && _badRun < BAD_READS && (coarseTicks(esp_timer_get_time()) - _lastGoodTicks) < secToCoarseTicks(EXPIRE_S);
    }

    // Use this wherever a stale reading would be acted on as a measurement.
    [[nodiscard]] float lastFresh() const { return fresh() ? last() : NAN; }
};


class TempSensorGPIO_NTC : public SingleValueSensor {
    const float *valuePtr = nullptr;
    EWMA<float> ewma2{2};

    [[nodiscard]] float volt2Temp(float vr) const {
        // vr = v_adc/vcc_supply
        // returns temperature in celsius
        constexpr auto beta = 3380.f; // [K] (25/50℃) (3380: NCP18XH103F03RB)
        constexpr auto T0 = 25.0f + 273.15f; // [K]
        constexpr auto R0 = 10e3f; // R(T0)
        constexpr auto Rp = 10e3f; // pull-down resistor

        return 1.f / (std::log(Rp / R0 * (1.f / vr - 1.f)) / beta + 1.f / T0) - 273.15f;
    }

protected:
    [[nodiscard]] bool present() const override { return valuePtr != nullptr; }

public:
    TempSensorGPIO_NTC() = default;

    // the NTC channel is a sensor.conf channel; refuse a stale board.conf one
    void begin(const ConfFile &boardConf) {
        assert_throw(valuePtr || boardConf.getByte("ntc_ch", 255) == 255,
                     "board.conf ntc_ch is no longer read, set ntc_ch in sensor.conf");
    }

    float readImpl() override {
        if (valuePtr == nullptr) return NAN;
        constexpr auto vcc = 3.3f;
        float temp = volt2Temp(*valuePtr / vcc);
        if (!plausible(temp)) return NAN;
        ewma2.add(temp);
        return temp;
    }

    [[nodiscard]] float last() const override { return ewma2.get(); }

protected:
    void resetFilters() override { ewma2.reset(); }

public:
    void setValueRef(const float &ref) {
        valuePtr = &ref;
    }
};

#if CONFIG_IDF_TARGET_ESP32S3

#include "driver/temperature_sensor.h"

class Esp32TempSensor : public SingleValueSensor {
    RunningMedian3<float> median3{};
    EWMA<float> ewma{20};

    //temperature_sensor_handle_t temp_handle = NULL;
    float tsens_out = NAN;
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = {
        .range_min = 20,
        .range_max = 100,
        .clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
        .flags = {.allow_pd = false},
    };

public:
    Esp32TempSensor() {
        /*temperature_sensor_config_t temp_sensor = {
                .range_min = 20,
                .range_max = 100,
        };
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));

        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
         */
        // 20~100 is a pre-defined range
    }

    [[nodiscard]] bool present() const override { return temp_sensor != nullptr; }

    void begin() {
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
        if (scope) scope->addChannel(this, 0, 'u', 12, "ucTemp");
    }

    float readImpl() override {
        if (!temp_sensor)return NAN;

        // This has very poor real-time performance if WiFi is enabled
        if (temperature_sensor_get_celsius(temp_sensor, &tsens_out) != ESP_OK) {
            //temp_sensor_config = (temp_sensor_dac_offset_t)((conf.dac_offset + 1) % TSENS_DAC_MAX);
            return NAN;
        }

        if (scope) scope->addSample12(this, 0, tsens_out * 40);
        if (!plausible(tsens_out)) return NAN;

        ewma.add(median3.next(tsens_out));

        return tsens_out;
    }

    [[nodiscard]] float last() const override { return ewma.get(); }

    void resetFilters() override { median3.reset(); ewma.reset(); }

    // unfiltered, bypasses median/EWMA state
    float readRaw() {
        float t = NAN;
        if (temp_sensor) temperature_sensor_get_celsius(temp_sensor, &t);
        return t;
    }
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


class Esp32TempSensor : public SingleValueSensor {
    RunningMedian3<float> median3{};
    EWMA<float> ewma{40};

public:
    Esp32TempSensor() {
    }

    void begin() {
    }

    float readImpl() override {
        //return (temprature_sens_read() - 32) / 1.8;
        const float t = temperatureRead();
        if (!plausible(t)) return NAN;
        ewma.add(median3.next(t));
        return t;
    }

    [[nodiscard]] float last() const override { return ewma.get(); }

    void resetFilters() override { median3.reset(); ewma.reset(); }
};

#endif


