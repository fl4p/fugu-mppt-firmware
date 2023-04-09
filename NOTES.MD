disable bccu on overvoltage

compute U,I noise sigma, P2P
estimate PV and battery impendance

* constant voltage method (k=.76)
  https://www.researchgate.net/publication/285589712_Combined_Constant_Voltage_and_Perturb_and_Observe_Method_Based_Algorithm_for_Fast_and_Efficient_Maximum_Power_Point_Tracking_Using_Buck-Boost_Converter



# Hardware

- breakout (stackable for externsions) with 5v, 12V? supply
- bccu FET use SMD? HYG007N03LS1C2 (but this is 30V only!)

## Schematic
- Reset+Program Buttons
- programmer as extension?
- use sync bucks?
- put hall sensor before caps
- think about power supply. libreSolar switches to LV supply if available. does this really reduce power consumption? TODO remove HV diode? crowbar circuit?

## PCB
- esp32 bigger pads
- small buck coils footprint
- SMD electrolytic
- use bigger mosfet holes
- use bigger terminals (pv + battery terminal blocks p=13)
- terminal labels not visible
- label r12 not readable (at ntc)
- border around usb controller (to see which parts not to solder)

- replace hall sensor with shunt? the ACS712ELCTR-30A-T costs $2,73 (lcsc 10x 2023-02). see  ads1115 datasheet for current sense application with midpoint voltage 
- esp32 pin mount for easy replacement ( https://de.aliexpress.com/item/1005002213376743.html )
- replace ADS1015 (which is hard to find in stock 2023-02) with  TLA2024 ?
- instead of isolated 12v power supply, use high-driver with bootstrapping
- step-down (synchronized) sop8 alternative?  https://www.lcsc.com/product-detail/DC-DC-Converters_Anpec-Elec-APW7301KAI-TRG_C115136.html
- OLED display
- USB type-c connector
- -SPI outbreak for additional hardware /CAN
- 5V outbreak
- place hall sensor before capacitor!
- use 100nF-200nf Film capacitors for EMF reduction
- two temp sensors? (coil + fets)
- Snubber?
- 3.3V + 12V OV protection Zener-Diode + Fuse behind DCDC to protect components

## Simplifications
* Use ESP32 ADCs


## Common Ground
- do we need common ground?
- if not we can remove the BCCU DCDC
- makes shunt current sensing easier
- add R_sense and current amplifier (without common GND, put this on the low side and use ordinary OPV)

# Firmware
- OUTPUT voltage can get higher than expected! (sudden battery cut, such as Lifepo4 BMS)
. minimum ontime lower FET, otherwise
- handle negative current (set REC=1)
- choose a framework:
-- https://github.com/RT-Thread/rt-thread
-- https://github.com/cesanta/mongoose-os
-- https://github.com/SmingHub/Sming
null- Zephyr
null- Mbed
- switching frequency? https://github.com/LibreSolar/mppt-2420-lc/issues/23#issuecomment-373252866

- store board indentifier in EEPROM so the FW can work on different boards and ESP32 pinout mappings

- go through the issues + pull requests on github
- Boost mode? from the battery terminals, the sync buck is a boost
- Bluetooth connection (setup?)


# Misc
- port to kicad or flux.ai ? ( https://github.com/jharvey/Charge_Controller )
- 2 channel version


# Extensions
Can be stacked on the I2C/SPI break out
- SD-Card
- CAN/RS434 interface


Fixes TODO
* It is actually possible to draw quite a lot of current from the PV input.
  it doesn't act as a real diode, current can flow as long as the voltage at the input capacitor is high enough
  should use differential current measurement. PV input is NOT short circuit proof!

* Use Alert Pin + Interrupt for faster reading
* Fix energy integration (millisRoutineInterval)

TODO
* Wifi AutoAp
* Settings Flash Storage
* WebInterface
* OTA Updates

* Github Issues
* OLED Display

* PWM soft start
* over temp cap Current
null* Silen Mode (reduce current instead of FAN switch on)



Forks:
* https://github.com/JosePineiro/FUGU-ARDUINO-MPPT-FIRMWARE/releases
  https://github.com/Kureigu07/FUGU-ARDUINO-MPPT-FIRMWARE
  https://github.com/krzysztof9nowak/FUGU-ARDUINO-MPPT-FIRMWARE (with web interface?)

```
while ! arduino-cli upload -b "esp32:esp32:esp32:UploadSpeed=115200" -p /dev/tty.usbserial-110 --discovery-timeout 1ms; echo retry; end
```

* Boost Mode? swap PV/Batterie




# Temp measurement
U_in=52.6V, U_out=26.7V, I_out=20A: T_coil=80°C, T_mosfet=50°C



# Lowside FET
- https://www.ti.com/lit/an/slua962/slua962.pdf?ts=1679569354557&ref_url=https%253A%252F%252Fwww.google.com%252F


# Battery Side Current measurement
Solar Input is short-circuit safe. But the battery output is not. Can



* avoid trickle charge!  https://www.youtube.com/watch?v=wy516po6uVU


Ota update pull request:
https://github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE/pull/23