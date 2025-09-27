#pragma once

//#include <cstdlib>

//#include <driver/adc.h>
//#include <esp_adc_cal.h>
#include "store.h" // ConfFile

enum class SampleReadScheme : uint8_t {
    cycle = 0, // cycle around channels
    any, // using callback
    all,
};

template<class T>
class AsyncADC {
public:

    /**
     * Tells the user how to read samples from this ADC
     * @return
     */
    [[nodiscard]] virtual SampleReadScheme scheme() const = 0;

    /**
     * Initializes ADC hardware
     * @param pinConf
     * @return
     */
    virtual bool init(const ConfFile &pinConf) = 0;

    virtual void deinit() = 0;

    virtual bool resetPeripherals() { return true; }

    /**
     * start reading
     * implementation setup
     * - periodic timers
     * - wake-up the ADC hardware from sleep (energy saving)
     */
    virtual void start() {

    }

    virtual float getSamplingRate(uint8_t channel) {
        throw std::runtime_error("not implemented");
    };

    /**
     * select the channel to read (scheme cycle and all)
     * @param channel
     */
    virtual void startReading(uint8_t channel) = 0;

    /**
     * check if ADC data is available for reading.
     * the implementation should block some time.
     * @return
     */
    virtual bool hasData() = 0;


    /**
     * read a single sample value (scheme cycle and all)
     * @return
     */
    virtual T getSample() = 0;

    /**
     * read any amount of samples from any channel (scheme any)
     * @param newSampleCallback
     * @return
     */
    //typedef void SampleCallback(const uint8_t &ch, float val);
    typedef std::function<void(const uint8_t &ch, T val)> SampleCallback;

    virtual uint32_t read(SampleCallback &&newSampleCallback) {
        throw std::runtime_error("not implemented");
    };

    //virtual uint8_t getReadingChannel() = 0;

    virtual void setMaxExpectedVoltage(uint8_t ch, float voltage) = 0;

    virtual float getInputImpedance(uint8_t ch) = 0;

    virtual void reset(const uint8_t ch) {

    }

    virtual ~AsyncADC() = default;
};