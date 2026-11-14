#include "logging.h"

#include <array>
#include <string>
//#include <sstream>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "lcd.h"


const char *TAG = "lcd";


bool testAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}


bool LCD::init(uint8_t configuredAddr) {
    if (lcd)
        return false;

    uint8_t addr = 0;

    if (configuredAddr && testAddress(configuredAddr)) {
        addr = configuredAddr;
    }

    if (!addr) {
        std::array<uint8_t, 2> addresses{0x27, 0x3F};
        for (auto a: addresses) {
            if (testAddress(a)) {
                addr = a;
                break;
            }
        }
    }

    if (!addr) {
        ESP_LOGW(TAG, "The LCD was not found on the I2C bus");
        return false;
    }

    ESP_LOGI(TAG, "Using addr 0x%02X", addr);
    lcd = new LiquidCrystal_I2C(addr, 16, 2);

    periodicInit();

    lcd->backlight();
    lightUntil = 60000 * 2 + millis();

    return true;
}

void LCD::updateValues(const LcdValues &values) {
    static int8_t bgIsOn = -1;

    if (!lcd) return;

    //periodicInit();

    auto now = millis();
    if (now - lastDrawTime < 500 or now < msgUntil) return;
    lastDrawTime = now;

    auto pin = values.Vin * values.Iin;

    // keep light on when power is >=300W
    if (pin >= 300)
        lightUntil = std::max(lightUntil, now + 30000);

    auto bgOn = (now <= lightUntil);
    if (bgIsOn != bgOn) {
        lcd->setBacklight(bgOn);
        bgIsOn = (int8_t) bgOn;
    }


    char line[20];
    snprintf(line, 20, "%4.1fV %4.1fA %3.0fW ", values.Vin, values.Iin, pin);
    lcd->setCursor(0, 0);
    lcd->print(line);

    snprintf(line, 17, "%4.1fV %4.1fA %2.0f\xDF" "C", values.Vout, values.Iout, values.Temp);
    lcd->setCursor(0, 1);
    lcd->print(line);
}

void LCD::displayMessage(const std::string &msg, uint16_t timeoutMs) {
    if (!lcd) return;
    //periodicInit();
    lcd->clear();
    /*std::istringstream iss(msg);
    int i = 0;
    for (std::string line; std::getline(iss, line);) {
        lcd->print(line.c_str());
        lcd->setCursor(0, ++i);
    }*/

    msgUntil = timeoutMs + millis();
    lightUntil = std::max(lightUntil, std::min(timeoutMs * 2, 30000) + millis());
}

void LCD::displayMessageF(const std::string &msg, uint16_t timeoutMs, ...) {
    static char buf[60];

    va_list args;
    va_start(args, timeoutMs);
    vsnprintf(buf, 60, msg.c_str(), args);
    va_end(args);

    displayMessage(std::string(buf), timeoutMs);
}

void LCD::periodicInit() {
    auto now = millis();

    if (!lcd)
        return;

    if (lastInit && lastInit + (60000ul * 5ul) > now)
        return;

    // disable periodic init due to latency issues
    //if(lastInit)
    //    return;

    lcd->begin(); //16, 2);
    lcd->clear();

    ESP_LOGI(TAG, "lcd->begin()");

    lastInit = millis();
}
