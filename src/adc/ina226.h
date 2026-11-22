
#include <Arduino.h> // micros(), pinMode(), delay()
#include <INA226_WE.h>

#include "sampling.h"

#include "i2c.h"
#include "i2c_hs.h"
#include "conf.h"
#include "logging.h"
#include "etc/rt.h"
#include "ina226_conv_time.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

class ADC_INA226;

void ina226_alert();

ADC_INA226 *ina226_instance = nullptr;

class ADC_INA226 : public AsyncADC<float> {
    INA226_WE ina226;
    volatile bool new_data = false;
    bool overflow = false;
    TaskNotification taskNotification{};
    uint8_t readChannel = 0;
    uint8_t alertPin;

    INA226_CONV_TIME convSetting_ = CONV_TIME_1100;
    uint16_t convUs_ = 1100;
    uint32_t alertTimeoutMs_ = 4;
    uint32_t measuredPairUs_ = 0; // measured CVRF interval (Vbus+I pair); 0 = not measured
    uint32_t lastAlertUs_ = 0;    // last sample or MASK_EN poll (micros)
    uint16_t ovRaw_ = 0;          // BOL limit in bus LSBs (board.conf ina22x_ov), 0 = conversion-ready alert
    volatile bool ovArmed_ = false;
    uint32_t ovArmFails_ = 0;
    uint8_t ovMiss_ = 0;          // CVRF polls without a conversion since the last sample
    uint16_t ovLead_ = 20;        // us, how early the next poll is scheduled
    uint32_t ovPolls_ = 0, ovMisses_ = 0;
    volatile uint32_t ovIsrCount_ = 0;
    esp_timer_handle_t pollTimer_ = nullptr; // wakes the RT task shortly before a CVRF poll is due
#if CONFIG_IDF_TARGET_ESP32S3
    I2cHs hs_;
    uint32_t hsHz_ = 0;
    uint16_t hsTest_[2][2]{}; // self-test {bad of 200, us/read} without / with HS code
#endif

    uint16_t readReg(uint8_t reg, bool &ok) {
#if CONFIG_IDF_TARGET_ESP32S3
        uint16_t v;
        if (hs_.on() && hs_.read16((uint8_t) ina226.i2cAddress, reg, v, hs_.code())) {
            ok = true;
            return v;
        }
        if (hs_.on() && !(hs_.fails & (hs_.fails - 1)))
            ESP_LOGW("ina22x", "HS read failures: %lu", hs_.fails);
#endif
        return ina226.readRegister(reg, ok, 2);
    }

public:

    static constexpr auto ChVBus = 0, ChI = 1;
    static constexpr auto INA22x_MANUFACTURER_ID_CMD = 0x3E;
    static constexpr auto INA22x_DEVICE_ID_CMD = 0x3F;

    [[nodiscard]] AdcReadMode readMode() const override {
        //
        return AdcReadMode::SnapshotAllChannels;
    }


    bool init(const ConfFile &boardConf) override {
        if (ina226_instance) {
            return false;
        }

        alertPin = (uint8_t) boardConf.getByte("ina22x_alert", 255);
        if (alertPin == 255) {
            ESP_LOGW("ina22x", "No ALERT pin specified");
            return false;
        }

        pinMode(alertPin, INPUT_PULLUP);  // esp32s3 has 45k internal pull up

        ina226_instance = this;
        attachInterrupt(digitalPinToInterrupt(alertPin), ina226_alert, FALLING);
        ESP_LOGI("ina22x", "Setup ALERT interrupt pin %u", alertPin);

        assert(!new_data);

        auto i2c_port = (i2c_port_t) boardConf.getByte("i2c_port", 0);
        auto addr = boardConf.getByte("ina22x_addr", 0b1000000);

        assert_throw(i2c_port == 0, ""); // not implemented, see _setupPeripherals()
        //if(i2c_port != 0)
        //    ina226._wire = new TwoWire((uint8_t) i2c_port);
        ina226.i2cAddress = addr;

        float resistor = boardConf.getFloat("ina22x_resistor"),
                range = boardConf.getFloat("ina22x_range",
                                         35.0f);// default: 1mOhm, 80A (ina226 shunt voltage range is 81.92mV)
        ina226.setResistorRange(resistor, range);
        //ESP_LOGI("ina226", "ina226.calVal=%d", ina226.calVal);

        long convReqUs = boardConf.getLong("ina22x_conv_time_us", 1100);
        auto ct = ina226ConvTimeAtLeast(convReqUs);
        convSetting_ = ct.setting;
        convUs_ = ct.us;
        alertTimeoutMs_ = ina226AlertTimeoutMs(convUs_);
        ESP_LOGI("ina22x", "conv time %u us (req %ld), ~%.0f SPS, alert timeout %lu ms",
                 convUs_, convReqUs, ina226SampleRate(convUs_), alertTimeoutMs_);

        if (!resetPeripherals())
            return false;

        if (auto hsHz = boardConf.getLong("i2c_hs_freq", 0))
            beginHs(boardConf.getLong("i2c_freq", 100000), hsHz);

        //ESP_LOGI("ina226", "ina226.calVal2=%d", ina226.calVal);
        //ina226.setResistorRange(resistor, range);
        //ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);

        return true;
    }

    void beginHs(uint32_t fsHz, uint32_t hsHz) {
#if CONFIG_IDF_TARGET_ESP32S3
        if (!hs_.begin(fsHz, hsHz)) {
            ESP_LOGE("ina22x", "HS mode: clock setup failed");
            return;
        }
        // DEVICE_ID with and without the HS controller code; any mismatch of the mode used keeps Wire
        for (bool code: {false, true}) {
            uint32_t bad = 0, t0 = micros();
            for (int i = 0; i < 200; ++i) {
                uint16_t v = 0;
                if (!hs_.read16((uint8_t) ina226.i2cAddress, INA22x_DEVICE_ID_CMD, v, code) || v != 0x2260) ++bad;
            }
            hsTest_[code][0] = bad;
            hsTest_[code][1] = (micros() - t0) / 200;
            if (code == hs_.code() && (bad || hs_.codeAcks)) hs_.end();
        }
        hs_.fails = 0;
        hsHz_ = hsHz;
        printHs();
#else
        ESP_LOGW("ina22x", "i2c_hs_freq: HS mode is esp32s3 only");
#endif
    }

    void printHs() {
        if (ovRaw_)
            UART_LOG("ina22x: bus OV alert %s at %.3f V (pin), ISR trips %lu, poll period %lu us lead %u us, "
                     "polls %lu misses %lu", ovArmed_ ? "armed" : "NOT armed", ovRaw_ * 1.25e-3f, ovIsrCount_,
                     pairUs(), ovLead_, ovPolls_, ovMisses_);
#if CONFIG_IDF_TARGET_ESP32S3
        if (hsHz_)
            UART_LOG("ina22x: i2c_hs_freq=%lu %s%s, self-test bad/us: plain %u/%u, HS code %u/%u; "
                     "code ACKs %lu, read failures %lu (last 0x%lx)", hsHz_, hs_.on() ? "on" : "OFF (using Wire)",
                     hs_.code() ? " HS" : "", hsTest_[0][0], hsTest_[0][1], hsTest_[1][0], hsTest_[1][1],
                     hs_.codeAcks, hs_.fails, hs_.lastErr);
#endif
    }

    float getSamplingRate(uint8_t channel) override {
#if CONFIG_FUGU_INA226_MEASURED_RATE
        // A measured rate reflects the chip's real throughput (some parts convert faster than
        // their nominal conv-time). measuredPairUs_ is the CVRF interval = one Vbus+I pair.
        if (measuredPairUs_ > 0)
            return 1e6f / (float) measuredPairUs_;
#endif
        return ina226SampleRate(convUs_);
    }

#if CONFIG_FUGU_INA226_MEASURED_RATE
    float nominalSamplingRate(uint8_t) override { return ina226SampleRate(convUs_); }
#endif

    bool resetPeripherals() override {
        if (!i2c_test_address(ina226.i2cAddress)) {
            ESP_LOGI("ina22x", "Chip didnt respond at address 0x%02X", (uint8_t) ina226.i2cAddress);
            return false;
        }

        //i2c_port_t i2c_port = I2C_NUM_0; // TODO
        auto addr = (uint8_t) ina226.i2cAddress;

        bool ok = false;
        // a throw here would allocate on the RT core (stall recovery): fail the reset instead
        auto mfrId = ina226.readRegister(INA22x_MANUFACTURER_ID_CMD, ok);
        if (!ok) return false;
        auto deviceId = ina226.readRegister( INA22x_DEVICE_ID_CMD, ok);
        if (!ok) return false;
        ESP_LOGI("ina22x", "MfrID: 0x%04X, DeviceID: 0x%04X", mfrId, deviceId);

        if (deviceId != 0x2260) {
            ESP_LOGW("ina22x", "This is not an INA226 device!");
            return false;
        }

        //flat: ina22x: MfrID: 0x5449, DeviceID: 0x2260 (444sps)
        // fry:         MfrID: 0x5449, DeviceID: 0x2260 (512sps) -> probably fake, much higher noise on shunt channel
        // expected:  1000/(1.100*2) = 454sps



        ovArmed_ = false;
        ina226.reset_INA226();         //in case the device is already/still initialized

        vTaskDelay(2);

        if (ovRaw_) return armOv(); // runtime re-init in OV mode: no alert self-tests, ALERT edges mean OV

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
        ina226.setConversionTime(convSetting_, convSetting_); // ina22x_conv_time_us (hasData timeout derived)

        ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();
        ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);

#if CONFIG_FUGU_INA226_MEASURED_RATE
        measuredPairUs_ = measurePairUs();
        if (measuredPairUs_)
            ESP_LOGI("ina22x", "measured %.0f SPS (pair %lu us) vs nominal %.0f SPS",
                     1e6f / (float) measuredPairUs_, measuredPairUs_, ina226SampleRate(convUs_));
        else
            ESP_LOGW("ina22x", "rate measurement timed out, using nominal %.0f SPS", ina226SampleRate(convUs_));
#endif
        return true;
    }

    // Switch ALERT from conversion-ready to a latched bus over-voltage alert at `busV` (chip pin volts).
    void setBusOvLimit(float busV) {
        assert_throw(busV > 0 && busV < 36, "ina22x_ov out of the INA226 bus range");
        ina226.writeRegister(INA226_WE::INA226_MASK_EN_REG, 0); // no CNVR edge may read as OV
        ovRaw_ = (uint16_t) lroundf(busV / 1.25e-3f);
        assert_throw(armOv(), "ina22x_ov: arming the INA226 bus OV alert failed");
    }

    // Writes the full operating config (it also restores a chip that reset itself) and verifies it.
    // Never clears g_hwOvTrip; on failure it sets it and stays disarmed, pollConversion() retries.
    bool armOv() {
        if (!pollTimer_) {
            esp_timer_create_args_t a = {};
            a.callback = [](void *p) { ((ADC_INA226 *) p)->taskNotification.notify(); };
            a.arg = this;
            a.dispatch_method = ESP_TIMER_TASK;
            a.name = "ina_poll";
            ESP_ERROR_CHECK(esp_timer_create(&a, &pollTimer_));
        }
        ovArmed_ = false;
        lastAlertUs_ = micros();
        ina226.setAverage(AVERAGE_1);
        ina226.setConversionTime(convSetting_, convSetting_);
        ina226.setMeasureMode(CONTINUOUS);
        ina226.writeRegister(INA226_WE::INA226_CAL_REG, ina226.calVal);
        ina226.writeRegister(INA226_WE::INA226_ALERT_LIMIT_REG, ovRaw_);
        ina226.writeRegister(INA226_WE::INA226_MASK_EN_REG, 0x2001); // BOL | LEN
        bool ok1, ok2, ok3;
        auto cal = ina226.readRegister(INA226_WE::INA226_CAL_REG, ok1);
        auto lim = ina226.readRegister(INA226_WE::INA226_ALERT_LIMIT_REG, ok2);
        auto me = ina226.readRegister(INA226_WE::INA226_MASK_EN_REG, ok3);
        if (ok3 && (me & 0x10)) g_hwOvTrip = true; // AFF cleared by this read: keep the evidence
        if (!ok1 || !ok2 || !ok3 || cal != ina226.calVal || lim != ovRaw_ || (me & 0xFC03) != 0x2001) {
            g_hwOvTrip = true;
            ++ovArmFails_;
            if (!(ovArmFails_ & (ovArmFails_ - 1)))
                ESP_LOGE("ina22x", "bus OV alert arm failed #%lu (cal 0x%04x limit 0x%04x mask 0x%04x)",
                         ovArmFails_, cal, lim, me);
            return false;
        }
        ovMiss_ = 0;
        ovArmed_ = true;
        ESP_LOGI("ina22x", "ALERT = bus OV > %.3f V (pin), polling CVRF every %lu us", ovRaw_ * 1.25e-3f, pairUs());
        return true;
    }

    [[nodiscard]] uint32_t pairUs() const { return measuredPairUs_ ? measuredPairUs_ : 2u * convUs_; }

#if CONFIG_FUGU_INA226_MEASURED_RATE
    // Average n CVRF intervals (one Vbus+I conversion pair each) at the current operational
    // settings, by polling the alert-driven new_data flag. Returns the mean interval in us, or
    // 0 on timeout. Blocks ~n*pair_us (tens of ms); only called from init/resetPeripherals.
    uint32_t measurePairUs(int n = 16) {
        ina226.readAndClearFlags();
        new_data = false;
        uint32_t t = micros();
        while (!new_data) if (micros() - t > 100000) return 0; // align to a full interval
        new_data = false;
        ina226.readAndClearFlags();
        uint32_t tStart = micros();
        int got = 0;
        while (got < n) {
            if (new_data) {
                new_data = false;
                ina226.readAndClearFlags();
                ++got;
            } else if (micros() - tStart > (uint32_t) n * 5000 + 100000) {
                break;
            }
        }
        return got ? (micros() - tStart) / (uint32_t) got : 0;
    }
#endif

    void deinit() override {
        if (alertPin != 255)
            detachInterrupt(digitalPinToInterrupt(alertPin));
    }

    /*void update() {
        //ina226.setMeasureMode(CONTINUOUS);
        ina226.enableConvReadyAlert();
    } */

    bool strayAlert() {
        if (new_data) ESP_LOGE("ina22x", "stray alert signal");
        return new_data;
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
            ina226.readAndClearFlags();
            new_data = false;
            delay(1);
            if (strayAlert()) return false;
        }

        if (digitalRead(alertPin) != HIGH) {
            ESP_LOGE("ina22x", "Alert pin %u not pulled up, short to ground?", alertPin);
            return false;
        }


        //auto bus = (i2c_port_t) 0;

        ina226.startSingleMeasurement();
        if (strayAlert()) return false; // test disabled ConvReadyAlert

        ina226.enableConvReadyAlert();
        if (strayAlert()) return false;
        ina226.startSingleMeasurement();
        //assert(new_data); // TODO enable!adc-reset-peripherals test enabled ConvReadyAlert
        new_data = false;


        //auto confReg = i2c_read_short(bus, addr, INA226_WE::INA226_CONF_REG, true, 100);
        bool ok;
        auto confReg = ina226.readRegister(INA226_WE::INA226_CONF_REG, ok);
        if (!ok) return false;

        if (strayAlert()) return false;

        auto t0 = micros();

        // trigger a new measurement (see https://www.ti.com/lit/ds/symlink/ina226.pdf#page=11 ):
        //i2c_write_short(bus, addr, INA226_WE::INA226_CONF_REG, confReg);
        ina226.writeRegister(INA226_WE::INA226_CONF_REG, confReg);
        ina226.startSingleMeasurementNoWait();
        auto tWrite = micros();

        while (!new_data) {
            if (micros() - tWrite > 100000) {
                ESP_LOGE("ina22x", "no conversion-ready alert");
                return false;
            }
        }
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
        if ((tBusyWait - tWrite) >= (100 + (140 + 10) * 2 * 4)) {
            ESP_LOGE("ina22x", "conversion-ready alert late (%lu us)", tBusyWait - tWrite);
            return false;
        }

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

    void IRAM_ATTR alertNewDataFromISR() {
        if (ovRaw_) {
            if (g_hwOvIsrHook) g_hwOvIsrHook();
            g_hwOvTrip = true;
            ovIsrCount_ = ovIsrCount_ + 1;
        } else
            new_data = true;
        g_isrCore[1] = (uint8_t) xPortGetCoreID();
        taskNotification.notifyFromIsr();
    }

    // OV mode: ALERT carries only BOL, so conversions are found by polling CVRF once per pair period.
    // The poll lead grows while polls hit without a miss (late) and shrinks after a miss (early).
    // Misses retry after 100 us; after 8 in a row, or a failed read, the next poll is a period away.
    // Far from due, the task sleeps on the ADC notification it shares with the other ADCs, which may
    // consume pollTimer_'s wake-up (set 300 us early); within 400 us of due it busy-waits instead.
    bool pollConversion() {
        const uint32_t P = pairUs();
        auto rem = (int32_t) (lastAlertUs_ + P - micros());
        if (!ovArmed_) { // lost arm (reset, I2C error, chip reset): trip until a re-arm succeeds, once per period
            g_hwOvTrip = true;
            if (rem > 0) {
                if (!esp_timer_is_active(pollTimer_)) esp_timer_start_once(pollTimer_, rem);
                taskNotification.wait(alertTimeoutMs_);
                return false;
            }
            if (!armOv()) return false;
            rem = (int32_t) (lastAlertUs_ + P - micros());
        }
        if (rem > 400) {
            if (!esp_timer_is_active(pollTimer_)) esp_timer_start_once(pollTimer_, rem - 300);
            taskNotification.wait(alertTimeoutMs_);
            rem = (int32_t) (lastAlertUs_ + P - micros());
            if (rem > 400) return false;
        }
        if (rem > 0) esp_rom_delay_us(rem);
        uint32_t now = micros();
        bool rok;
        uint16_t me = readReg(INA226_WE::INA226_MASK_EN_REG, rok);
        ++ovPolls_;
        if (!rok) {
            lastAlertUs_ = now;
            return false;
        }
        if ((me & 0xFC03) != 0x2001) { // chip lost its config (reset/brown-out): trip, re-arm next period
            g_hwOvTrip = true;
            ovArmed_ = false;
            lastAlertUs_ = now;
            return false;
        }
        if (me & 0x10) g_hwOvTrip = true; // AFF, latched until this read
        overflow = (me >> 2) & 0x0001;
        if (!(me & 0x08)) {
            ++ovMisses_;
            lastAlertUs_ = now - P + (++ovMiss_ > 8 ? P : 100);
            return false;
        }
        if (ovMiss_) { if (ovLead_ > 20) ovLead_ -= 10; }
        else if (ovLead_ < 300) ovLead_ += 20;
        ovMiss_ = 0;
        lastAlertUs_ = now - ovLead_;
        return true;
    }

    bool hasData() override {
        static uint32_t numTimeouts = 0, numRecovered = 0;

        if (ovRaw_) return pollConversion();

        if (!new_data)
            taskNotification.wait(alertTimeoutMs_);

        if (!new_data) {
            // Missed the ALERT edge. Without FUGU_GPIO_ISR_IRAM the GPIO ISR is masked while the
            // flash cache is off (config/coulomb persist, OTA); a dropped edge would otherwise
            // stall the RT sampler until the loop-latency watchdog shuts the converter down. Poll
            // the chip instead: reading MASK_EN reports CVRF *and* clears+re-arms the alert, so a
            // missed edge degrades to a late sample rather than a starvation event.
            // The task notification is shared with the other ADCs, whose ISRs keep the wait from
            // timing out, so the poll is paced by the time since the last sample instead.
            uint32_t now = micros();
            if (now - lastAlertUs_ < alertTimeoutMs_ * 1000)
                return false;
            lastAlertUs_ = now;
            ++numTimeouts;
            bool rok;
            uint16_t me = readReg(INA226_WE::INA226_MASK_EN_REG, rok);
            if (!rok) return false;
            overflow = (me >> 2) & 0x0001;
            if ((me >> 3) & 0x0001) {
                if (++numRecovered % 20000 == 0)
                    ESP_LOGW("ina22x", "%lu alert edges recovered by poll", numRecovered);
                return true; // getSample() reads the data registers
            }
            if (numTimeouts % 20000 == 0)
                ESP_LOGE("ina22x", "%lu timeout!", numTimeouts);
            return false;
        }

        new_data = false;
        lastAlertUs_ = micros();

        bool rok;
        uint16_t value = readReg(INA226_WE::INA226_MASK_EN_REG, rok);
        if (!rok) return false;
        overflow = (value >> 2) & 0x0001; // MATH overflow, only current/power data invalid (Vbus ok)
        bool convAlert = (value >> 3) & 0x0001;
        bool limitAlert = (value >> 4) & 0x0001;

        if (limitAlert) {
            ESP_LOGW("ina22x", "Limit Alert!");
        }

        return convAlert;
    }

    // overflow is a MATH (current/power) over-range: Vbus stays valid and the saturated current
    // is passed through to the loop's own over-current protection, so it must not fault the ADC.
    bool isGood() override {
        return true;
    }

    float getSample() override {
        bool success;
        switch (readChannel) {
            case ChVBus: {
                uint16_t raw = readReg(INA226_WE::INA226_BUS_REG, success);
                if (!success) {
                    ESP_LOGW("ina22x", "failure reading register");
                    return NAN;
                }
                if (scope) scope->addSample12(this, readChannel, raw / 4);
                return (float) raw * 0.00125f;
            }
            case ChI: {
                auto raw = static_cast<int16_t>(readReg(INA226_WE::INA226_CURRENT_REG, success));
                if (!success) {
                    ESP_LOGW("ina22x", "failure reading register");
                    return NAN;
                }
                if (scope) scope->addSample12(this, readChannel, std::abs(raw) / 3 /*abs(i16)->12bit*/);
                auto amp = ((float) raw / ina226.currentDivider_mA / 1000.f);
                if (unlikely(overflow)) {
                    static uint32_t n = 0;
                    if (n++ % 1000 == 0) ESP_LOGW("ina22x", "Overflow current = %.2fA", amp);
                }
                return amp;
            }
            default:
                assert(false);
                return NAN;
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

    [[nodiscard]] bool bipolar(uint8_t ch) const override { return ch == ChI; }
};


void ina226PrintInfo() {
    if (ina226_instance) ina226_instance->printHs();
}

void IRAM_ATTR ina226_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
