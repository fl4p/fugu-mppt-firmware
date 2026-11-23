> Original note (December 2024). The maintained version is [Sensors](https://fl4p.github.io/fugu-mppt-firmware/docs/internals/sensors/).


The firmware implements a sensor layer that abstracts ADC asynchronous reading.
It supports multiply ADC.

A sensor represents an ADC channel, which is usually one physical pin or two pins for differential channels.
