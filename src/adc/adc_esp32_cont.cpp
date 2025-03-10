#include "adc_esp32_cont.h"

#include "tele/scope.h"

//#define ADC1_SR 400000 // sampling rate (105k max, see https://www.esp32.com/viewtopic.php?t=1215)
// 50k, 64k, 80k, 100k, 125k, 128k, 156.25k, 160k, 200k, 250k, 312.5k, 320k, 400k, 500k, 625k, 640k, 800k
// https://www.wolframalpha.com/input?i=factor+%5B%2F%2Fmath%3A80000000%2F%2F%5D
//#define ADC1_AVG 64 // num averaging samples, max 256

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
            .max_store_buf_size = ADC1_READ_LEN * 2,
            .conv_frame_size = ADC1_READ_LEN / 2, // use half read len to drain buffer while data exists
            // the driver will trigger the interrupt once <conv_frame_size> bytes are available. if we miss
            // one interrupt, and we only read <conv_frame_size> bytes per notification, one frame will always
            // stay in the ring buffer (<max_store_buf_size> bytes), adding unnecessary latency
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));


    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};

    uint32_t patLen = 0, chNum = 0;
    bool hasNtc = false;
    for (auto ch = 0; ch <= adc_channel_t::ADC_CHANNEL_9; ++ch)
        if (attenuation[ch] != (adc_atten_t) -1) {
            assert(patLen < SOC_ADC_PATT_LEN_MAX);
            adc_pattern[patLen].atten = attenuation[ch];
            adc_pattern[patLen].channel = ch & 0x7;
            adc_pattern[patLen].unit = ADC_UNIT_1;
            adc_pattern[patLen].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
            ESP_LOGI("adc_esp32", "pattern[%lu] = {.atten=%d, .channel=%d}", patLen, attenuation[ch], ch);
            ++patLen;
            //if(scope)scope->addChannel(ch, 'u', 12, "");
            if (ch == ntcCh) hasNtc = true;
            ++chNum;
        }

    // duplicate pattern for HF channels (without NTC ch) for increased BW
    if (hasNtc && (patLen - 1) * 2 <= SOC_ADC_PATT_LEN_MAX) {
        ESP_LOGI("adc_esp32", "Duplicate pattern without ntc channel");
        for (auto ch = 0; ch <= adc_channel_t::ADC_CHANNEL_9; ++ch)
            if (attenuation[ch] != (adc_atten_t) -1 && ch != ntcCh) {
                assert(patLen < SOC_ADC_PATT_LEN_MAX);
                adc_pattern[patLen].atten = attenuation[ch];
                adc_pattern[patLen].channel = ch & 0x7;
                adc_pattern[patLen].unit = ADC_UNIT_1;
                adc_pattern[patLen].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
                ESP_LOGI("adc_esp32", "pattern[%lu] = {.atten=%d, .channel=%d}", patLen, attenuation[ch], ch);
                ++patLen;
            }
    } else if (hasNtc) {
        ESP_LOGI("adc_esp32", "NTC channel but pattern table to small to duplicate");
    }

    assert_throw(patLen > 0, "");

    ESP_LOGI("adc_esp32", "ADC1 SR=%lu Hz, nCh=%lu, avg=%u, pattern=%lu => %.0f sps/ch", sr, chNum, avgNum, patLen,
             sr / chNum * ((float) (patLen == chNum ? patLen : (patLen + hasNtc)) / patLen) / avgNum);

    // Note about sample freq:
    // this is the frequency the adc reads samples of any channel
    // if we sample 3 channels in a continous pattern, the effective sampling rate per channel will be 1/3.
    adc_continuous_config_t dig_cfg = {
            .pattern_num = patLen,
            .adc_pattern = adc_pattern,
            .sample_freq_hz = sr, // sps= /numCh/averaging
            .conv_mode = ADC_CONV_SINGLE_UNIT_1,
            .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };
    ESP_ERROR_CHECK_THROW(adc_continuous_config(handle, &dig_cfg));

    // start
    notification.subscribe();

    adc_continuous_evt_cbs_t cbs = {
            .on_conv_done = s_conv_done_cb,
            .on_pool_ovf = nullptr,
    };
    ESP_ERROR_CHECK_THROW(adc_continuous_register_event_callbacks(handle, &cbs, this));
    ESP_ERROR_CHECK_THROW(adc_continuous_start(handle));
}

uint32_t ADC_ESP32_Cont::read(SampleCallback &&newSampleCallback) {
    //static uint32_t max_ret_num = 0, min_ret_num = ADC1_READ_LEN + 1;
    uint32_t ret_num = 0;
    // don't wait here, as we already do in haveData(), we dont want to block other ADCs
    esp_err_t ret = adc_continuous_read(handle, result, ADC1_READ_LEN, &ret_num, 0);

    if (ret == ESP_OK) {
        //assert(ret_num == ADC1_READ_LEN);

        /*
        if (ret_num > max_ret_num) {
            if (ret_num == ADC1_READ_LEN and max_ret_num <= 32)
                max_ret_num++; // grace periode
            else {
                max_ret_num = ret_num;
                ESP_LOGI("adc_esp32", "max_ret_num=%lu", max_ret_num);
            }
        }

        if (ret_num < min_ret_num) {
            min_ret_num = ret_num;
            ESP_LOGI("adc_esp32", "min_ret_num=%lu", min_ret_num);
        }
         */

        //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            auto *p = (adc_digi_output_data_t *) &result[i];
            uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
/* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
//ESP_LOGI("adc_esp32", "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);

                if (scope) {
                    scope->addSample12(this, chan_num, data);
                }
                //ESP_LOGI("adccont", "scope %p", scope);

                avgBuf[chan_num].num++;
                avgBuf[chan_num].agg += data;

                //assert(data < 4096);
                //assert(avgBuf[chan_num].agg < 65536);

                if (avgBuf[chan_num].num == avgNum) {
                    data = avgBuf[chan_num].agg / avgBuf[chan_num].num;
                    int mv = 0;
                    adc_cali_raw_to_voltage(adc_chars[attenuation[chan_num]], data, &mv);
                    float v = (float) mv * 1e-3f;
                    newSampleCallback(chan_num, v);
                    //if(scope) scope->addSample12(chan_num, data);
                    avgBuf[chan_num].num = 0;
                    avgBuf[chan_num].agg = 0;
                }

            } else {
                ESP_LOGW("adc_esp32", "Invalid data [%s_%" PRIu32 "_%" PRIx32 "]", "ADC1", chan_num, data);
            }
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
//We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
        //ESP_LOGW("adc_esp32", "Read timeout.");
//vTaskDelay(100);
    } else {
        assert(ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK_THROW(ret);
    }

    return ret_num;
}