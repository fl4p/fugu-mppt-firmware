#include "adc_esp32_cont.h"

static bool IRAM_ATTR
s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

#if !CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE
#error "please enable CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE for optimal performance"
#endif

return ((ADC_ESP32_Cont *) user_data)->convDoneCallback();
}


void ADC_ESP32_Cont::start()  {

adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
};
ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));


int channel_num = 0;
adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};


uint32_t numCh = 0;
for (auto ch = 0; ch < adc1_channel_t::ADC1_CHANNEL_MAX; ++ch)
if (attenuation[ch] != (adc_atten_t) -1) {
assert(numCh < SOC_ADC_PATT_LEN_MAX);
adc_pattern[numCh].atten = attenuation[ch];
adc_pattern[numCh].channel = ch & 0x7;
adc_pattern[numCh].unit = ADC_UNIT_1;
adc_pattern[numCh].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
ESP_LOGI("adc_esp32", "pattern[%lu] = {.atten=%d, .channel=%d}", numCh, attenuation[ch], ch);
++numCh;
}

adc_continuous_config_t dig_cfg = {
        .pattern_num = numCh,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
};
ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

// start
notification.subscribe();

adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
        .on_pool_ovf = nullptr,
};
ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, this));
ESP_ERROR_CHECK(adc_continuous_start(handle));
}