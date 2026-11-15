#pragma once
// `pwr` console command (CONFIG_FUGU_WITH_PWR_KNOBS): runtime power levers, not persisted.

class ADC_Fake;
extern ADC_Fake *g_pwrFakeAdc; // set by setupSensors() when sensor.conf uses adc=fake

#include <cstdint>

extern uint16_t g_pwrNetLoopMs; // network loop delay per pass, ms (`pwr netloop`)

bool pwrCommand(int argc, const char *const *argv);
void pwrTick(); // network loop, every pass: `pwr pm` window sampling and end
