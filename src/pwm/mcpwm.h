#pragma once

// todo use the push-pull inverter code
//

#include "esp_log.h"
#include "driver/mcpwm.h"
//#include "esp_timer.h"
#include "esp_task.h"


#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us

class PWM_MCPWM {
    mcpwm_timer_handle_t timer = nullptr;
    uint32_t period_ticks, duty_cycle;
    mcpwm_oper_handle_t operator_ = nullptr;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator = nullptr;

public:

    void init_pwm(int channel, int pin, uint32_t freq) {

        if(!timer) {
            uint32_t period_ticks = BLDC_MCPWM_TIMER_RESOLUTION_HZ / freq;

            mcpwm_timer_config_t timer_config = {
                    .group_id = 0,
                    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                    .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
                    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                    .period_ticks = period_ticks,
            };
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
        }
    }

    void update_pwm(int channel, uint32_t duty) {

    }


};




struct PWMTimerSync {
    //const PWMSignal &ref;
    //const PWMSignal &other;
    mcpwm_sync_handle_t timer_sync_source = NULL;
    uint32_t delay_ticks;


    PWMTimerSync(const PWMSignal &ref, const PWMSignal &other, uint32_t delay_ticks)
            : delay_ticks(delay_ticks) {
        _sync(ref, other);
    }

    void _sync(const PWMSignal &ref, const PWMSignal &other) {
        mcpwm_timer_sync_src_config_t timer_sync_config = {.timer_event = MCPWM_TIMER_EVENT_EMPTY,};
        ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(ref.timer, &timer_sync_config, &timer_sync_source));
        mcpwm_timer_sync_phase_config_t sync_phase_config = {
                .sync_src = timer_sync_source,
                .count_value = delay_ticks,
                .direction = MCPWM_TIMER_DIRECTION_UP,

        };
        ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(other.timer, &sync_phase_config));
    }

    void stop() {
        mcpwm_del_sync_src(timer_sync_source);
        timer_sync_source = nullptr;
    }

    //void reconnect_timers() {
    //    if (timer_sync_source) stop();
    //    _sync();
    //}
};