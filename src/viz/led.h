#if USE_FASTLED
#include <FastLED.h>
#else

#include "led_strip.h"

#endif

#include "etc/pinconfig.h"
#include "store.h"

/**
 * topping mode: blue 0,9,48
 */


class LedIndicator {
#if USE_FASTLED
    CRGB leds[1]{0};
#else
    led_strip_handle_t led_strip;
#endif

    bool enabled = false;

public:
    bool begin(const ConfFile &pinConfig) {
        std::string k = "led_WS2812";
        auto pin = pinConfig.getByte(k, 0);
        if (!pin) pin = pinConfig.getLong(k + "B", 0);
        if (!pin)return false;

        //ESP_LOGI("led", "pin = %d", pin);
        //for (auto &s: pinConfig.keys())
        //    ESP_LOGI("led", "k=%s", s.c_str());

        /// LED strip common configuration
        led_strip_config_t strip_config = {
                .strip_gpio_num = pin,  // The GPIO that connected to the LED strip's data line
                .max_leds = 1,                 // The number of LEDs in the strip,
                .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
                .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // The color component format is G-R-B
                .flags = {
                        .invert_out = false, // don't invert the output signal
                }
        };

/// RMT backend specific configuration
        led_strip_rmt_config_t rmt_config = {
                .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
                .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency: 10MHz
                .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
                .flags = {
                        .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
                }
        };

        /// Create the LED strip object
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

#if USE_FASTLED
        assert(pin == 1);
        constexpr uint8_t LED_PIN = 1;
        FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, 1);
        FastLED.setBrightness(15);
#endif
        enabled = true;

        return true;
    }

    void setRGB(uint8_t ir, uint8_t ig, uint8_t ib) {
        if (!enabled)return;

#if USE_FASTLED
        auto rgb = CRGB(ir, ig, ib);
        if (rgb != leds[0])
            ESP_LOGD("LED", "setRgb(%hhu, %hhu, %hhu)", ir, ig, ib);
        //else return;
        leds[0] = rgb;
        FastLED.show();
#else
        if (!ir && !ig && !ib) {
            ESP_LOGD("led", "Clear!");
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
        } else {
            uint8_t damp = 8;
            if(ir) ir = max(1, ir / damp);
            if(ig) ig = max(1, ig / damp);
            if(ib) ib = max(1, ib / damp);
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, ir, ig, ib));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGD("led", "R=%d,G=%d,B=%d", ir, ig, ib);
        }
#endif
    }

    void setRGB(const char *hex) {
        if (!enabled)return;
        auto l = strtol(hex, nullptr, 16);
        if (strlen(hex) == 3) {
            setHexShort(l);
        } else {
            setRGB((l >> 16) & 0xFF, (l >> 8) & 0xFF, (l >> 0) & 0xFF);
        }
    }

    inline void setHex(uint32_t l) {
        setRGB(l >> 16, (l >> 8) & 0xff, l & 0xff);
    }

    void setHexShort(uint16_t l) {
        // css short color f93 => ff9933
        // 0xY / 0xYY is always 17
        setRGB(((l >> 8) & 0xf) * 17, ((l >> 4) & 0xf) * 17, ((l >> 0) & 0xf) * 17);
    }
};
