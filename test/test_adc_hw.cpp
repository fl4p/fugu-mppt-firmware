// On-target liveness tests for fry's two ADC backends, run ONE BY ONE with step-by-step ESP_LOGI
// markers. The polled data paths already pass; this exercises the INTERRUPT paths the firmware
// actually relies on (continuous-ADC DMA conv-done ISR, INA226 conversion-ready ALERT pin). If the
// board stalls, the last logged step localises the fault (= ADC hardware not reacting on interrupts).
//
// Params hardcoded to fry (config/dl/fry.local.192.168.4.2) so no littlefs mount is needed.

#include <unity.h>
#include <Arduino.h> // Wire, micros, delay
#include <esp_log.h>
#include <esp_timer.h>

#include <string>

#include "conf.h"
#include "adc/adc_esp32_cont.h"
#include "i2c.h" // i2c_test_address

// --- fry hardware constants (sensor.conf / board.conf) ---
#define FRY_ADC_SR    22000
#define FRY_ADC_AVG   32
#define FRY_VIN_CH    3   // esp32adc1
#define FRY_NTC_CH    7   // esp32adc1
#define FRY_I2C_SDA   42
#define FRY_I2C_SCL   2
#define FRY_I2C_FREQ  800000
#define FRY_INA_ADDR  0x40
#define FRY_INA_ALERT 41 // ina226_alert (active-low, conversion-ready)

// INA226 registers
#define INA_REG_CONFIG   0x00
#define INA_REG_BUS_V    0x02
#define INA_REG_MASK_EN  0x06
#define INA_REG_MFR_ID   0xFE // -> 0x5449 'TI'
#define INA_REG_DIE_ID   0xFF // -> 0x2260
#define INA_CVRF_BIT     0x0008 // Mask/Enable bit3: conversion-ready flag (clears on read)
#define INA_MASK_CNVR_EN 0x0400 // Mask/Enable bit10: enable conversion-ready ALERT pin
#define INA_CONFIG_CONT  0x4127 // avg1, 1.1ms VBUS+VSH, continuous shunt+bus

static const char *TAG = "test_adc";

static volatile uint32_t s_alertCount = 0;
static volatile int s_alertCore = -1;
static void IRAM_ATTR inaAlertIsr() { ++s_alertCount; s_alertCore = xPortGetCoreID(); }

static uint16_t inaRead(uint8_t reg, bool &ok) {
    Wire.beginTransmission((uint8_t) FRY_INA_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { ok = false; return 0; } // repeated-start
    if (Wire.requestFrom((uint8_t) FRY_INA_ADDR, (uint8_t) 2) != 2) { ok = false; return 0; }
    uint16_t v = (uint16_t) Wire.read() << 8;
    v |= (uint8_t) Wire.read();
    ok = true;
    return v;
}

static bool inaWrite(uint8_t reg, uint16_t val) {
    Wire.beginTransmission((uint8_t) FRY_INA_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t) (val >> 8));
    Wire.write((uint8_t) (val & 0xFF));
    return Wire.endTransmission() == 0;
}

// ---- ADC #1: internal continuous ADC (DMA conv-done ISR path) ----
void test_internal_adc_continuous_samples() {
    ESP_LOGI(TAG, "[internal] BEGIN");
    ConfFile sens{{
        {"esp32adc1_avg", std::to_string(FRY_ADC_AVG)},
        {"esp32adc1_sr", std::to_string(FRY_ADC_SR)},
    }};
    ConfFile board{{}};

    ADC_ESP32_Cont adc{sens};
    adc.init(board);
    ESP_LOGI(TAG, "[internal] setMaxExpectedVoltage ch%d/ch%d", FRY_VIN_CH, FRY_NTC_CH);
    adc.setMaxExpectedVoltage(FRY_VIN_CH, 3.0f);
    adc.setMaxExpectedVoltage(FRY_NTC_CH, 3.0f);
    adc.setNtcCh(FRY_NTC_CH);
    ESP_LOGI(TAG, "[internal] start() — arming continuous DMA");
    adc.start();
    ESP_LOGI(TAG, "[internal] draining read() for 2s (waits on conv-done ISR notification)");

    uint32_t nVin = 0, nNtc = 0;
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 2000000) {
        if (adc.hasData())
            adc.read([&](const uint8_t &ch, float v) {
                if (ch == FRY_VIN_CH) ++nVin;
                else if (ch == FRY_NTC_CH) ++nNtc;
            });
    }
    bool good = adc.isGood();
    float srVin = adc.getSamplingRate(FRY_VIN_CH);
    adc.deinit();

    ESP_LOGI(TAG, "[internal] END: vin=%lu ntc=%lu isGood=%d rate(vin)=%.0f sps",
             (unsigned long) nVin, (unsigned long) nNtc, good, srVin);

    TEST_ASSERT_TRUE_MESSAGE(good, "no-sample watchdog tripped: continuous ADC DMA stalled");
    TEST_ASSERT_GREATER_THAN_UINT32(200, nVin);
    TEST_ASSERT_GREATER_THAN_UINT32(100, nNtc);
}

// ---- ADC #2: INA226 — I2C liveness, then conversion-ready ALERT interrupt reactivity ----
void test_ina226_alert_interrupt() {
    ESP_LOGI(TAG, "[ina226] BEGIN: Wire.begin sda=%d scl=%d @%d", FRY_I2C_SDA, FRY_I2C_SCL, FRY_I2C_FREQ);
    Wire.begin(FRY_I2C_SDA, FRY_I2C_SCL, FRY_I2C_FREQ);
    delay(2);

    ESP_LOGI(TAG, "[ina226] i2c_test_address 0x%02X", FRY_INA_ADDR);
    TEST_ASSERT_TRUE_MESSAGE(i2c_test_address(FRY_INA_ADDR), "INA226 did not ACK on I2C bus");

    bool ok = false;
    uint16_t mfr = inaRead(INA_REG_MFR_ID, ok);
    ESP_LOGI(TAG, "[ina226] MFR_ID=0x%04X ok=%d", mfr, ok);
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x5449, mfr, "MFR_ID mismatch — device at 0x40 is not a TI INA226");
    uint16_t die = inaRead(INA_REG_DIE_ID, ok);
    ESP_LOGI(TAG, "[ina226] DIE_ID=0x%04X ok=%d", die, ok);
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x2260, die, "DIE_ID mismatch — not an INA226");

    ESP_LOGI(TAG, "[ina226] write CONFIG=continuous");
    TEST_ASSERT_TRUE_MESSAGE(inaWrite(INA_REG_CONFIG, INA_CONFIG_CONT), "I2C write of CONFIG failed");
    delay(3);

    // (a) polled liveness — do conversions complete at all?
    ESP_LOGI(TAG, "[ina226] poll CVRF for 1s");
    uint32_t conversions = 0, i2cErrors = 0;
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 1000000) {
        uint16_t me = inaRead(INA_REG_MASK_EN, ok);
        if (!ok) { ++i2cErrors; continue; }
        if (me & INA_CVRF_BIT) ++conversions;
        delayMicroseconds(200);
    }
    ESP_LOGI(TAG, "[ina226] polled: conversions=%lu i2cErrors=%lu",
             (unsigned long) conversions, (unsigned long) i2cErrors);

    // (b) ALERT interrupt reactivity — the path that throws "40000 timeout" in production.
    ESP_LOGI(TAG, "[ina226] pinMode alert pin %d INPUT_PULLUP", FRY_INA_ALERT);
    pinMode(FRY_INA_ALERT, INPUT_PULLUP);
    delay(1);
    int idle = digitalRead(FRY_INA_ALERT);
    ESP_LOGI(TAG, "[ina226] alert idle level=%d (expect 1)", idle);

    s_alertCount = 0;
    s_alertCore = -1;
    ESP_LOGI(TAG, "[ina226] attachInterrupt FALLING on alert pin");
    attachInterrupt(digitalPinToInterrupt(FRY_INA_ALERT), inaAlertIsr, FALLING);
    inaRead(INA_REG_MASK_EN, ok); // clear any stale CVRF
    ESP_LOGI(TAG, "[ina226] enable conversion-ready ALERT (MASK_EN=0x%04X)", INA_MASK_CNVR_EN);
    TEST_ASSERT_TRUE_MESSAGE(inaWrite(INA_REG_MASK_EN, INA_MASK_CNVR_EN), "I2C write of MASK_EN failed");

    ESP_LOGI(TAG, "[ina226] waiting up to 2s for ALERT interrupts (clear+rearm each)...");
    uint32_t cleared = 0;
    t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 2000000) {
        if (s_alertCount != cleared) { inaRead(INA_REG_MASK_EN, ok); cleared = s_alertCount; }
        delayMicroseconds(100);
    }
    detachInterrupt(digitalPinToInterrupt(FRY_INA_ALERT));
    inaWrite(INA_REG_MASK_EN, 0x0000); // disable alert

    ESP_LOGI(TAG, "[ina226] END: alertInterrupts=%lu isrCore=%d (conversions were %lu)",
             (unsigned long) s_alertCount, s_alertCore, (unsigned long) conversions);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, i2cErrors, "INA226 I2C errors (bus fault)");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(100, conversions, "INA226 conversions not completing");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, idle, "alert pin not high when idle (short/stuck?)");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(100, s_alertCount,
        "INA226 conversion-ready ALERT interrupt NOT firing (alert pin / GPIO ISR path)");
}
