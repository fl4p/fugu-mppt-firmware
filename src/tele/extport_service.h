#pragma once

#include <cmath>
#include <cstring>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_mac.h>

#include "../service.h"
#include "../mppt.h"
#include "../app_state.h"
#include "../etc/version.h"
#include "vedirect_text.h"

extern ADC_Sampler adcSampler;
extern VIinVout<const Sensor *> sensors;
extern MpptController mppt;

// External comms port. mode=vedirect sends a VE.Direct text block once a second on tx_pin (UART1, 19200 8N1).
// rx_pin, if set, guards TX: any byte a VE.Direct peer would not send (HEX frames only), such as an echo
// from a CAN transceiver, latches TX off until the service restarts.
class ExtPortService : public Service {
    static constexpr uart_port_t PORT = UART_NUM_1;
    int8_t _tx = -1, _rx = -1;
    uint16_t _pid = 0;
    uint16_t _refused = 0; // first foreign RX byte + 1 (0 = none)
    bool _installed = false;
    uint32_t _blocks = 0, _lastMs = 0;
    char _ser[12]{};

    // "2.1.0" -> 210
    static constexpr uint16_t fwNum(const char *s = FIRMWARE_VERSION) {
        uint16_t v = 0;
        for (; *s; ++s) if (*s >= '0' && *s <= '9') v = v * 10 + (*s - '0');
        return v;
    }

    static int8_t pin(const ConfFile &c, const char *key) {
        long p = c.getLong(key, -1);
        return (p >= 0 && p < GPIO_NUM_MAX) ? (int8_t) p : -1;
    }

    void release() {
        if (_installed) uart_driver_delete(PORT);
        _installed = false;
        if (_tx >= 0) gpio_reset_pin((gpio_num_t) _tx);
        if (_rx >= 0) gpio_reset_pin((gpio_num_t) _rx);
    }

    static uint8_t cs(bool off) {
        if (g_app.manualPwm()) return VE_CS_EXT;
        if (off) return VE_CS_OFF;
        if (mppt.charger.termCond) return VE_CS_FLOAT;
        if (mppt.charger.termCond.cvHold() || mppt.getState() == MpptControlMode::CV) return VE_CS_ABSORPTION;
        return VE_CS_BULK;
    }

    static uint32_t offReason(const char *why) {
        if (!why) return 0;
        if (!strcmp(why, "Vin-Vout") || !strcmp(why, "minpulse-M")) return VE_OR_NO_INPUT;
        if (!strcmp(why, "calibrating") || !strcmp(why, "filters-empty") || !strcmp(why, mppt.kIdleBlock)) return 0;
        return VE_OR_PROTECTION;
    }

    void send() {
        if (adcSampler.adcStates.empty() || !mppt.sensorPhysicalI || !mppt.sensorPhysicalU) return;
        const float v = sensors.Vout->ewm.avg.get(), i = sensors.Iout->ewm.avg.get(), vpv = sensors.Vin->ewm.avg.get();
        const float ppv = mppt.sensorPhysicalI->ewm.avg.get() * mppt.sensorPhysicalU->ewm.avg.get();
        if (!std::isfinite(v) || !std::isfinite(i) || !std::isfinite(vpv) || !std::isfinite(ppv)) return;

        const bool off = mppt.converter.disabled() && !mppt.isSweeping();
        const auto mode = mppt.getState();
        const auto &dm = mppt.meter.dailyEnergyMeter;
        const auto *yd = dm.days().last();
        auto cWh = [](double wh) { return (uint32_t) std::lround(std::max(wh, 0.0) / 10.0); };

        VeText t{
            .pid = _pid, .fw = fwNum(), .ser = _ser,
            .v_mV = (int32_t) std::lround(v * 1000), .i_mA = (int32_t) std::lround(i * 1000),
            .vpv_mV = (int32_t) std::lround(vpv * 1000), .ppv_W = (uint32_t) std::lround(std::max(ppv, 0.f)),
            .cs = cs(off),
            .mppt = off ? VE_MPPT_OFF : (mode == MpptControlMode::MPPT || mode == MpptControlMode::Sweep)
                                            ? VE_MPPT_ACTIVE : VE_MPPT_LIMITED,
            .err = 0,
            .off = off ? offReason(mppt.startBlockReason()) : 0,
            .h19 = cWh(mppt.meter.totalEnergy.get()), .h20 = cWh(dm.today.energyYield),
            .h21 = (uint32_t) std::lround(std::max(dm.today.pinMax, 0.f)),
            .h22 = yd ? cWh((float) yd->energyYield) : 0, .h23 = yd ? (uint32_t) std::lround((float) yd->pinMax) : 0,
            .hsds = (uint16_t) (dm.days().getNumTotalDays() % 365),
        };
        char buf[256];
        if (size_t n = veTextBlock(buf, sizeof buf, t)) {
            uart_write_bytes(PORT, buf, n);
            ++_blocks;
        }
    }

    // VE.Direct peers only send HEX frames: ':' then hex digits, ending in '\n'
    void checkRx() {
        uint8_t b[32];
        int n;
        while ((n = uart_read_bytes(PORT, b, sizeof b, 0)) > 0) {
            for (int k = 0; k < n; ++k) {
                const uint8_t c = b[k];
                if (c == ':' || c == '\n' || (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) continue;
                _refused = c + 1;
                uart_wait_tx_done(PORT, 0);
                release();
                ESP_LOGW(_name, "foreign byte 0x%02x on rx: wrong adapter? TX off until `svc rs extport`", c);
                return;
            }
        }
    }

protected:
    bool onStart() override {
        ConfFile c{_confPath, true};
        _refused = 0;
        _blocks = 0;
        if (c.getString("mode", "off") != "vedirect") {
            ESP_LOGE(_name, "mode=%s not supported (off|vedirect)", c.getString("mode", "off").c_str());
            return false;
        }
        _tx = pin(c, "tx_pin");
        _rx = pin(c, "rx_pin");
        _pid = (uint16_t) c.getLong("pid", 0xA056);
        if (_tx < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(_tx) || _tx == _rx) {
            ESP_LOGE(_name, "tx_pin missing or invalid");
            return false;
        }
        if (_rx < 0) ESP_LOGW(_name, "no rx_pin: wrong-adapter guard off");
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(_ser, sizeof _ser, "FG%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

        uart_config_t u = {};
        u.baud_rate = 19200;
        u.data_bits = UART_DATA_8_BITS;
        u.parity = UART_PARITY_DISABLE;
        u.stop_bits = UART_STOP_BITS_1;
        u.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        u.source_clk = UART_SCLK_DEFAULT;
        if (uart_driver_install(PORT, 256, 256, 0, nullptr, 0) != ESP_OK) return false;
        _installed = true;
        if (uart_param_config(PORT, &u) != ESP_OK
            || uart_set_pin(PORT, _tx, _rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
            release();
            return false;
        }
        if (_rx >= 0) gpio_pullup_en((gpio_num_t) _rx);
        ESP_LOGI(_name, "vedirect tx=%d rx=%d pid=0x%04X", _tx, _rx, _pid);
        return true;
    }

    void onStop() override { release(); }

    void onTick() override {
        if (_refused) return;
        const uint32_t now = millis();
        if (now - _lastMs < 1000) return;
        _lastMs = now;
        if (_rx >= 0) checkRx();
        if (!_refused) send();
    }

public:
    ExtPortService() : Service("extport", "/littlefs/conf/extport.conf", /*requiresNetwork*/ false,
                               /*enabledDefault*/ false) {}

    [[nodiscard]] std::string statusDetail() const override {
        char b[48];
        if (_refused) snprintf(b, sizeof b, "refused: rx 0x%02x", _refused - 1);
        else snprintf(b, sizeof b, "vedirect %lu blocks", (unsigned long) _blocks);
        return b;
    }
};

inline ExtPortService extPortService;
