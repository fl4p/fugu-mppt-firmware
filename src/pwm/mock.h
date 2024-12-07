#pragma once

#include <cassert>
#include <stdint.h>


class PWM_Mock {

public:
    uint16_t pwmMax;

    uint16_t v;

    PWM_Mock() : pwmMax(0) {}

    void init_pwm(int channel, int pin, uint32_t freq) {
        // see idf5.3 ledc_find_suitable_duty_resolution()
        uint32_t div = 80000000 / freq;
        auto resolution = std::min((int) log2(div), 14);
        auto pm = ((2 << (resolution - 1)) - 1);
        if (pwmMax == 0) pwmMax = pm;
        assert(pm == pwmMax);
    }

    void update_pwm(int channel, uint32_t duty) {
        v = duty;
    }

    void update_pwm(int channel, uint32_t hpoint, uint32_t duty) {
        v  = duty - hpoint;
    }
};