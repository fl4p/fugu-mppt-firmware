#pragma once

// 16-bit register read on I2C0 (ESP32-S3 only), driving the controller registers directly between
// Wire transactions, polled. With `code`, I2C high-speed mode: controller code 0x08 at <=400 kHz,
// END (SCL held low), HS timing, Sr + register read, STOP (which ends HS mode). Holds the Wire lock.

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32S3

#include <Wire.h>
#include "hal/i2c_ll.h"
#include "esp_clk_tree.h"

class I2cHs {
    static constexpr uint8_t NREG = 10; // registers of REGS, filter last
#define I2C_HS_REGS(X) X(scl_low_period) X(scl_high_period) X(sda_hold) X(sda_sample) X(scl_rstart_setup) \
                       X(scl_stop_setup) X(scl_start_hold) X(scl_stop_hold) X(to) X(filter_cfg)
    uint32_t fs_[NREG]{}, hs_[NREG]{}; // timing register words, fs_ also restores the filter
    SemaphoreHandle_t lock_ = nullptr;
    bool on_ = false, code_ = false;

    struct LockOf : TwoWire {
        static SemaphoreHandle_t get(TwoWire &w) {
#if CONFIG_DISABLE_HAL_LOCKS
            return nullptr;
#else
            return w.*(&LockOf::lock);
#endif
        }
    };

    static constexpr uint32_t ERR = I2C_LL_INTR_NACK | I2C_LL_INTR_TIMEOUT | I2C_LL_INTR_ARBITRATION;

    static constexpr uint32_t cmd(uint32_t op, uint32_t n = 0, uint32_t ackEn = 0, uint32_t ackVal = 0) {
        return n | ackEn << 8 | ackVal << 10 | op << 11;
    }

    static bool calc(uint32_t src, uint32_t hz, uint32_t *w) {
        i2c_hal_clk_config_t c;
        i2c_ll_master_cal_bus_clk(src, hz, &c);
        i2c_dev_t d{};
        i2c_ll_master_set_bus_timing(&d, &c);
        // the HS input filter (I2C clock cycles) must pass the shorter SCL high
        i2c_ll_master_set_filter(&d, std::min<uint8_t>(I2C0.filter_cfg.scl_filter_thres, c.scl_high / 2));
#define X(r) *w++ = d.r.val;
        I2C_HS_REGS(X)
#undef X
        return c.clkm_div == I2C0.clk_conf.sclk_div_num + 1; // the divider stays as the driver set it
    }

    static void apply(const uint32_t *w);
    static uint32_t wait(uint32_t mask);

public:
    uint32_t fails = 0, codeAcks = 0;

    bool on() const { return on_; }
    bool code() const { return code_; } // HS mode, only needed above 400 kHz

    bool begin(uint32_t fsHz, uint32_t hsHz) {
        uint32_t src = 0;
        esp_clk_tree_src_get_freq_hz((soc_module_clk_t) I2C_CLK_SRC_DEFAULT,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src);
        lock_ = LockOf::get(Wire);
        code_ = hsHz > 400000;
        on_ = src && calc(src, std::min<uint32_t>(fsHz, 400000), fs_) &&
              calc(src, std::clamp<uint32_t>(hsHz, 100000, 3400000), hs_);
        fs_[NREG - 1] = I2C0.filter_cfg.val;
        return on_;
    }

    void end() { on_ = false; }

    bool read16(uint8_t addr, uint8_t reg, uint16_t &val, bool code);
};

// only one TU may include this header (non-inline IRAM definitions)

IRAM_ATTR void I2cHs::apply(const uint32_t *w) {
#define X(r) I2C0.r.val = *w++;
    I2C_HS_REGS(X)
#undef X
    I2C0.ctr.conf_upgate = 1;
}

// raw int bits of `mask` or an error; TIMEOUT after 2 ms
IRAM_ATTR uint32_t I2cHs::wait(uint32_t mask) {
    uint32_t t0 = micros(), r;
    while (!((r = I2C0.int_raw.val) & (mask | ERR)))
        if (micros() - t0 > 2000) return I2C_LL_INTR_TIMEOUT;
    return r & ERR;
}

IRAM_ATTR bool I2cHs::read16(uint8_t addr, uint8_t reg, uint16_t &val, bool code) {
    i2c_dev_t *hw = &I2C0;
    if (lock_ && xSemaphoreTake(lock_, portMAX_DELAY) != pdTRUE) return false;
    uint32_t r = I2C_LL_INTR_ARBITRATION;
    if (!hw->sr.bus_busy) {
        hw->int_clr.val = I2C_LL_INTR_MASK;
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        r = 0;
        if (code) {
            apply(fs_);
            hw->data.val = 0x08;
            hw->comd[0].val = cmd(I2C_LL_CMD_RESTART);
            hw->comd[1].val = cmd(I2C_LL_CMD_WRITE, 1);
            hw->comd[2].val = cmd(I2C_LL_CMD_END);
            hw->ctr.trans_start = 1;
            r = wait(I2C_LL_INTR_END_DETECT);
            if (!hw->sr.resp_rec) ++codeAcks; // nobody may ACK the controller code
            hw->int_clr.val = I2C_LL_INTR_MASK;
        }
        if (!r) {
            apply(hs_);
            hw->data.val = addr << 1;
            hw->data.val = reg;
            hw->data.val = addr << 1 | 1;
            hw->comd[0].val = cmd(I2C_LL_CMD_RESTART);
            hw->comd[1].val = cmd(I2C_LL_CMD_WRITE, 2, 1);
            hw->comd[2].val = cmd(I2C_LL_CMD_RESTART);
            hw->comd[3].val = cmd(I2C_LL_CMD_WRITE, 1, 1);
            hw->comd[4].val = cmd(I2C_LL_CMD_READ, 1, 0, 0);
            hw->comd[5].val = cmd(I2C_LL_CMD_READ, 1, 0, 1);
            hw->comd[6].val = cmd(I2C_LL_CMD_STOP);
            hw->ctr.trans_start = 1;
            r = wait(I2C_LL_INTR_MST_COMPLETE);
        }
        if (r & I2C_LL_INTR_NACK) { // release the bus as the driver does
            hw->int_clr.val = I2C_LL_INTR_MASK;
            hw->comd[0].val = cmd(I2C_LL_CMD_STOP);
            hw->ctr.trans_start = 1;
            wait(I2C_LL_INTR_MST_COMPLETE);
        }
        if (!r) {
            val = (uint16_t) ((hw->data.val & 0xff) << 8);
            val |= hw->data.val & 0xff;
        }
        hw->filter_cfg.val = fs_[NREG - 1];
        hw->ctr.conf_upgate = 1;
        hw->int_clr.val = I2C_LL_INTR_MASK;
    }
    if (lock_) xSemaphoreGive(lock_);
    if (r) ++fails;
    return !r;
}

#endif
