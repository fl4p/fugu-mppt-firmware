
ESP32's internal ADC has non-linearity issues and is quite noisy.
The linearity can be fixed with calibration and we keep a margin to the ADC voltage range. (e.g. at 6dB PGA attenuation 150 mV ~ 1750 mV)
We can reduce noise with a sliding mean filter, which reduces update rate.
The ESP32 ADC being much faster than the ADS1015, the reduced update rate is still suffiecient for an MPPT application.

A1 (LV) 	--> GPIO39 (ADC1_3)
A2 (Hall)	--> GPIO36 (ADC1_0)
A3 (HV)		--> GPIO34 (ADC1_6)