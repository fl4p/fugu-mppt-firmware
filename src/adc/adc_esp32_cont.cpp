#include "adc_esp32_cont.h"

#include <algorithm>

#include "tele/scope.h"


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
    good_ = true;
    lastDataUs_ = esp_timer_get_time(); // grace period before the no-sample watchdog can trip
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC1_READ_LEN * 2,
        .conv_frame_size = ADC1_READ_LEN / 2, // use half read len to drain buffer while data exists
        // the driver will trigger the interrupt once <conv_frame_size> bytes are available. if we miss
        // one interrupt, and we only read <conv_frame_size> bytes per notification, one frame will always
        // stay in the ring buffer (<max_store_buf_size> bytes), adding unnecessary latency
        // NOTE: conv_frame_size (= the DMA EOF granularity) is the lever that sets DMA stall headroom —
        // see ADC1_READ_LEN in adc_esp32_cont.h. max_store_buf_size only sizes the software pool.

        .flags = {.flush_pool = false}, // TODO
    };
    ESP_ERROR_CHECK_THROW(adc_continuous_new_handle(&adc_config, &handle));


    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};

    uint32_t patLen = 0, chNum = 0;
    bool hasNtc = false;
    // Normalize all channels to maxAtten (IDF requires uniform attenuation per ADC unit)
    if (maxAtten > ADC_ATTEN_DB_0 && calByAtten[maxAtten] == nullptr) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t conf{
            .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_0, .atten = maxAtten, .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&conf, &calByAtten[maxAtten]));
#else
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1, .atten = maxAtten, .bitwidth = ADC_BITWIDTH_DEFAULT, .default_vref = 0,
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &calByAtten[maxAtten]));
#endif
    }
    for (auto ch = 0; ch <= adc_channel_t::ADC_CHANNEL_9; ++ch)
        if (attenByCh[ch] != (adc_atten_t) -1) {
            attenByCh[ch] = maxAtten;
            assert(patLen < SOC_ADC_PATT_LEN_MAX);
            adc_pattern[patLen].atten = maxAtten;
            adc_pattern[patLen].channel = ch;
            adc_pattern[patLen].unit = ADC_UNIT_1;
            adc_pattern[patLen].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
            ESP_LOGI("adc_esp32", "pattern[%lu] = {.atten=%d, .channel=%d}", patLen, attenByCh[ch], ch);
            ++patLen;
            //if(scope)scope->addChannel(ch, 'u', 12, "");
            if (ch == ntcCh) hasNtc = true;
            ++chNum;
        }

    // duplicate pattern for HF channels (without NTC ch) for increased BW
    if (hasNtc && (patLen - 1) * 2 <= SOC_ADC_PATT_LEN_MAX) {
        ESP_LOGI("adc_esp32", "Duplicate pattern without ntc channel");
        for (auto ch = 0; ch <= adc_channel_t::ADC_CHANNEL_9; ++ch)
            if (attenByCh[ch] != (adc_atten_t) -1 && ch != ntcCh) {
                assert(patLen < SOC_ADC_PATT_LEN_MAX);
                adc_pattern[patLen].atten = maxAtten;
                adc_pattern[patLen].channel = ch;
                adc_pattern[patLen].unit = ADC_UNIT_1;
                adc_pattern[patLen].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
                ESP_LOGI("adc_esp32", "pattern[%lu] = {.atten=%d, .channel=%d}", patLen, maxAtten, ch);
                ++patLen;
            }
    } else if (hasNtc) {
        ESP_LOGI("adc_esp32", "NTC channel but pattern table to small to duplicate");
    }

    assert_throw(patLen > 0, "");

    buildMvTable();
#if !CONFIG_IDF_TARGET_ESP32S3
    inlCorr_ = false; // the INL table is ESP32-S3 data
#endif
    if (maxAtten != ADC_ATTEN_DB_12) inlCorr_ = false; // fitted at 12 dB only
    ESP_LOGI("adc_esp32", "INL correction %s", inlCorr_ ? "on" : "off");

    ESP_LOGI("adc_esp32", "ADC1 SR=%lu Hz, nCh=%lu, avg=%u, pattern=%lu => %.0f sps/ch", sr, chNum, avgNum, patLen,
             sr / chNum * ((float) (patLen == chNum ? patLen : (patLen + hasNtc)) / patLen) / avgNum);

    // Note about sample freq:
    // this is the frequency the adc reads samples of any channel
    // if we sample 3 channels in a continuous pattern, the effective sampling rate per channel will be 1/3.
    adc_continuous_config_t dig_cfg = {
        .pattern_num = patLen,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = sr, // sps= /numCh/averaging
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };
    ESP_ERROR_CHECK_THROW(adc_continuous_config(handle, &dig_cfg));

    notification.subscribe();

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
        .on_pool_ovf = nullptr,
    };
    ESP_ERROR_CHECK_THROW(adc_continuous_register_event_callbacks(handle, &cbs, this));
    ESP_ERROR_CHECK_THROW(adc_continuous_start(handle));
}

void ADC_ESP32_Cont::buildMvTable() {
    if (knotAtten_ == maxAtten) return; // resetPeripherals() re-enters start(); the table only depends on atten
    auto cal = calByAtten[maxAtten];
    assert_throw(cal != nullptr, "adc cali missing");
    // Each knot is a least-squares line through the 64 integer conversions around it, evaluated at
    // the knot: the integer-mV truncation averages out to a constant ~-0.5 mV (which a board's
    // offset absorbs) and the second-step curve is smooth over 64 codes.
    for (int k = 0; k < kKnots; ++k) {
        const int c = k << kKnotShift;
        const int lo = std::max(0, c - 32), hi = std::min(4095, c + 31);
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        const int n = hi - lo + 1;
        for (int r = lo; r <= hi; ++r) {
            int mv = 0;
            adc_cali_raw_to_voltage(cal, r, &mv);
            const double x = r - c;
            sx += x, sy += mv, sxx += x * x, sxy += x * mv;
        }
        const double b = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        mvKnot_[k] = (float) ((sy - b * sx) / n);
    }
    knotAtten_ = maxAtten;
}

float ADC_ESP32_Cont::rawToMv(float raw) const {
    const float f = raw * (1.f / (1 << kKnotShift));
    const int k = std::min((int) f, kKnots - 2);
    return mvKnot_[k] + (f - (float) k) * (mvKnot_[k + 1] - mvKnot_[k]);
}

// Universal ESP32-S3 ADC1 INL at 12 dB, in pin mV, added to the IDF curve-fitting output. Fitted
// jointly on two boards (fboost, fmetal; 200k/7.5k dividers) against an INA228 traced to an
// HP3458A, 412 rungs 16.5..74.5 V, 2026-10-09/10 (pwr-metering ina228-u-4ref-dense). Legendre
// P2..P5 only, so it carries no gain or offset of its own: a board's gain/offset stays in
// sensor.conf <ch>_gain/<ch>_offset. Held out per board: ~100 -> ~37 mV rms input-referred.
// Outside the fitted 571..2659 mV it holds its edge value rather than extrapolate.
float ADC_ESP32_Cont::inlDelta(float mv) {
    constexpr float kMid = 1615.f, kHalf = 1044.f;
    constexpr float c[] = {-1.868178f, -15.276578f, 13.885663f, 34.574096f, -13.801880f, -12.758385f};
    float x = (mv - kMid) * (1.f / kHalf);
    x = x < -1.f ? -1.f : x > 1.f ? 1.f : x;
    float y = c[5];
    for (int i = 4; i >= 0; --i) y = y * x + c[i];
    return y;
}

uint32_t ADC_ESP32_Cont::read(SampleCallback &&newSampleCallback) {
    uint32_t ret_num = 0;
    // don't wait here, as we already do in haveData(), we dont want to block other ADCs
    esp_err_t ret = adc_continuous_read(handle, result, ADC1_READ_LEN, &ret_num, 0);

    if (ret == ESP_OK) {
        if (ret_num) lastDataUs_ = esp_timer_get_time();
        //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            auto *p = (adc_digi_output_data_t *) &result[i];
            uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
            // Check the ch number validation, the data is invalid if the channel num exceed the maximum channel
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                //if (scope)scope->addSample12(this, chan_num, data);

                avgBuf[chan_num].num++;
                avgBuf[chan_num].agg += data;

                if (avgBuf[chan_num].num == avgNum) {
                    const float raw = (float) avgBuf[chan_num].agg / (float) avgBuf[chan_num].num;
                    if (scope)scope->addSample12(this, chan_num, avgBuf[chan_num].agg / avgBuf[chan_num].num);
                    float mv = rawToMv(raw);
                    if (inlCorr_) mv += inlDelta(mv);
                    newSampleCallback(chan_num, mv * 1e-3f);
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
        // unexpected driver error: never throw from the RT loop. Flag unhealthy so the sampler
        // treats it as AdcError via isGood(); start() clears the flag on recovery.
        if (good_) ESP_LOGE("adc_esp32", "adc_continuous_read error 0x%x", ret);
        good_ = false;
    }

    return ret_num;
}
