#include "adc_esp32_cont.h"

#define ADC1_SR 400000 // sampling rate (105k max, see https://www.esp32.com/viewtopic.php?t=1215)
// 50k, 64k, 80k, 100k, 125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k, 400k, 500k, 625k, 640k, 800k
// https://www.wolframalpha.com/input?i=factor+%5B%2F%2Fmath%3A80000000%2F%2F%5D
#define ADC1_AVG 64 // num averaging samples, max 256

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif


static bool IRAM_ATTR
s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

#if !CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE
#error "please enable CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE for optimal performance"
#endif

    return ((ADC_ESP32_Cont *) user_data)->convDoneCallback();
}


void ADC_ESP32_Cont::start() {

    adc_continuous_handle_cfg_t adc_config = {
            .max_store_buf_size = ADC1_READ_LEN * 4,
            .conv_frame_size = ADC1_READ_LEN, // ADC1_READ_LEN
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));


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
    // Note about sample freq:
    // this is the frequency the adc reads samples of any channel
    // if we sample 3 channels in a continous pattern, the effective sampling rate per channel will be 1/3.
    adc_continuous_config_t dig_cfg = {
            .pattern_num = numCh,
            .adc_pattern = adc_pattern,
            .sample_freq_hz = ADC1_SR, // sps= /numCh/averaging
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

uint32_t ADC_ESP32_Cont::read(SampleCallback &&newSampleCallback) {
    uint32_t ret_num = 0;
    esp_err_t ret = adc_continuous_read(handle, result, ADC1_READ_LEN, &ret_num, 0);

    if (ret == ESP_OK) {
        //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            auto *p = (adc_digi_output_data_t *) &result[i];
            uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
/* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
//ESP_LOGI("adc_esp32", "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);

                avgBuf[chan_num].num++;
                avgBuf[chan_num].agg += data;

                //assert(data < 4096);
                //assert(avgBuf[chan_num].agg < 65536);

                if (avgBuf[chan_num].num == ADC1_AVG) {
                    data = avgBuf[chan_num].agg / avgBuf[chan_num].num;
                    float v = (float) esp_adc_cal_raw_to_voltage(data, adc_chars[attenuation[chan_num]]) * 1e-3f;
                    newSampleCallback(chan_num, v);
                    avgBuf[chan_num].num = 0;
                    avgBuf[chan_num].agg = 0;
                }

            } else {
                ESP_LOGW("adc_esp32", "Invalid data [%s_%"PRIu32"_%"PRIx32"]", "ADC1", chan_num, data);
            }
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
//We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
        ESP_LOGW("adc_esp32", "Read timeout.");
//vTaskDelay(100);
    }

    return ret_num;
}