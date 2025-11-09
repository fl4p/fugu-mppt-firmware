#include <vector>
#include "adc.h"
#include "math/statmath.h"

/**
 * Fake ADC for unit testing
 */
class ADC_Dummy : public AsyncADC<float> {
    uint8_t readingChannel = 255;

    const std::vector<std::vector<float>> &samples;
    std::vector<std::vector<float>::const_iterator> samplePtr;

public:

    std::vector<float> maxExpectedVoltages;

    explicit ADC_Dummy(const std::vector<std::vector<float>> &samples) : samples(samples) {
        samplePtr.resize(samples.size());
        maxExpectedVoltages.resize(samples.size());

        resetPointer();
    }

    bool init(const ConfFile &boardConf) override {
        return true;
    }

    void startReading(uint8_t channel) override {
        readingChannel = channel;
        ESP_LOGI("dummy adc", "Reading channel %i", (int) channel);
    }

    bool hasData() override { return samplePtr[readingChannel] != samples[readingChannel].end(); }

    void setMaxExpectedVoltage(uint8_t ch, float voltage) override {
        maxExpectedVoltages[ch] = voltage;
    }

    float getSample() override {
        auto v = *samplePtr[readingChannel];
        ++samplePtr[readingChannel];
        ESP_LOGI("dummy adc", "Get ch %i sample = %.4f", (int) readingChannel, v);
        return v;
    }

    void resetPointer() {
        for (auto i = 0; i < samples.size(); ++i)
            samplePtr[i] = samples[i].begin();
    }
};
