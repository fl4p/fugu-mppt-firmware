#include <FastLED.h>

#include "pinconfig.h"


/**
 * topping mode: blue 0,9,48
 */


class LedIndicator {
    CRGB leds[1];

public:
    void begin() {
        if (!(uint8_t) PinConfig::LED)
            return;
        FastLED.addLeds<WS2812B, (uint8_t) PinConfig::LED, GRB>(leds, 1);
        FastLED.setBrightness(15);
        setHexShort(0x210);
    }

    void setRGB(uint8_t ir, uint8_t ig, uint8_t ib) {
        if (!(uint8_t) PinConfig::LED)
            return;

        auto rgb = CRGB(ir, ig, ib);
        if (rgb != leds[0])
            ESP_LOGI("LED", "setRgb(%hhu, %hhu, %hhu)", ir, ig, ib);
        //else return;
        leds[0] = rgb;
        FastLED.show();

    }

    void setRGB(const char *hex) {
        auto l = strtol(hex, nullptr, 16);
        if (strlen(hex) == 3) {
            setHexShort(l);
        } else {
            setRGB((l >> 16) & 0xFF, (l >> 8) & 0xFF, (l >> 0) & 0xFF);
        }
    }

    void setHexShort(uint16_t l) {
        // css short color f93 => ff9933
        // 0xY / 0xYY is always 17
        setRGB(((l >> 8) & 0xf) * 17, ((l >> 4) & 0xf) * 17, ((l >> 0) & 0xf) * 17);
    }
};
