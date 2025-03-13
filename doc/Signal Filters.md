# Noise

The most obvious noise comes from the ADC itself. This noise can easily be reduced by increasing ADC conversion time and
sample averaging.

Another significant noise source are the loads connected to the battery.
The higher the impedance of the battery and wiring, the greater the noise.
Connect the MPPT charger and the loads separetly as close as possible to the battery terminals.
This will reduce noise coupling from the loads to the charger. Keep in mind that high frequency noise
can propagate more easily due to wire inductance, even when using very thick cables.
Noise can degrade tracking performance significantly.

Laptops can have a quite complex noise spectra that is difficult to describe.
A sliding median filter and a averaging filter are a good choice here.

A 50 Hz inverter has an 100 Hz ac component at the input (this is because we "see" `abs(sin(a*t))` at the input).
A notch filter can remove this quite well.

A median filter can filter load bursts.



