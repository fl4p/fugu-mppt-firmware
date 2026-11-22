#pragma once

#include "store.h" // ConfFile

// How the sampler must pull samples from an ADC backend.
enum class AdcReadMode : uint8_t {
    // Muxed ADC: one channel converted per poll, sampler advances the mux round-robin.
    // Backend implements startReading()/hasData()/getSample().
    MuxedRoundRobin = 0,
    // Continuous DMA: backend pushes samples for any channel via read(cb).
    StreamedCallback,
    // Latched regs (e.g. INA226): one conversion-ready event yields every channel.
    // Sampler reads all channels each poll via startReading()/getSample().
    SnapshotAllChannels,
};

template<class T>
class AsyncADC {
public:
    /**
     * Tells the user how to read samples from this ADC
     * @return
     */
    [[nodiscard]] virtual AdcReadMode readMode() const = 0;

    /**
     * Initializes ADC hardware
     * @param boardConf
     * @return
     */
    virtual bool init(const ConfFile &boardConf) = 0;

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
        using namespace std::string_literals;
        throw std::runtime_error("getSR"s + " " + "not implemented");
    };

    // configured rate, independent of what the hardware delivers
    virtual float nominalSamplingRate(uint8_t channel) { return getSamplingRate(channel); }

    /**
     * select the channel to read (MuxedRoundRobin and SnapshotAllChannels)
     * @param channel
     */
    virtual void startReading(uint8_t channel) = 0;

    /**
     * check if ADC data is available for reading.
     * the implementation should block some time.
     * @return
     */
    virtual bool hasData() = 0;


    virtual bool isGood() {
        return true;
    };


    /**
     * read a single sample value (MuxedRoundRobin and SnapshotAllChannels)
     * @return
     */
    virtual T getSample() = 0;

    /**
     * read any amount of samples from any channel (StreamedCallback)
     * @param newSampleCallback
     * @return
     */
    //typedef void SampleCallback(const uint8_t &ch, float val);
    typedef std::function<void(const uint8_t &ch, T val)> SampleCallback;

    virtual uint32_t read(SampleCallback &&newSampleCallback) {
        using namespace std::string_literals;
        throw std::runtime_error("read"s + "not implemented");
    };

    //virtual uint8_t getReadingChannel() = 0;

    virtual void setMaxExpectedVoltage(uint8_t ch, float voltage) = 0;

    virtual float getInputImpedance(uint8_t ch) = 0;

    // the raw sample can go below 0
    [[nodiscard]] virtual bool bipolar(uint8_t ch) const { return false; }

    virtual void reset(const uint8_t ch) {
    }

    virtual ~AsyncADC() = default;
};
