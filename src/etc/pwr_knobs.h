#pragma once
// `pwr` console command (CONFIG_FUGU_WITH_PWR_KNOBS): runtime power levers, not persisted.

class ADC_Fake;
extern ADC_Fake *g_pwrFakeAdc; // set by setupSensors() when sensor.conf uses adc=fake

bool pwrCommand(int argc, const char *const *argv);
