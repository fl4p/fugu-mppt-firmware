#pragma once

class ConfFile;
struct Limits;

// Builds the vin / iin / iout / vout / ntc Sensor instances on the global adcSampler from
// sensor.conf: picks the ADC backend per channel, derives the raw->physical LinearTransform
// (resistor-divider ratio for voltages, factor/midpoint for currents), and substitutes a
// VirtualSensor for a missing current channel (computed from the other side and
// VIRTUAL_SENSOR_EFF). vout is added last so over-voltage protection sees it with the least latency.
// Throws on a missing or invalid sensor.conf.
void setupSensors(const ConfFile &boardConf, const Limits &lim);

// Assumed conversion efficiency of a VirtualSensor current channel.
constexpr float VIRTUAL_SENSOR_EFF = 0.97f;

// Sets g_app.loopRateMin to half the nominal Vout sample rate (left as is for a virtual Vout).
// Valid only after adcSampler.begin().
void setLoopRateMin();
