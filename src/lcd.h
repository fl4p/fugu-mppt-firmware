#pragma once

struct LcdValues {
    float Vin;
    float Vout;
    float Iin;
    float Iout;
    float Temp;
};

class LiquidCrystal_I2C;

class LCD {
    LiquidCrystal_I2C *lcd = nullptr;
    unsigned long lastDrawTime = 0;
    unsigned long msgUntil = 0;
public:
    bool init();

    void displayMessage(const std::string &msg, uint16_t timeoutMs);

    void updateValues(const LcdValues &values);
};