
# Hardware
- use bigger terminals (pv + battery terminal blocks p=13)
- replace hall sensor with shunt? the ACS712ELCTR-30A-T costs $2,73 (lcsc 10x 2023-02). see  ads1115 datasheet for current sense application with midpoint voltage 
- esp32 pin mount for easy replacement ( https://de.aliexpress.com/item/1005002213376743.html )
- replace ADS1015 (which is hard to find in stock 2023-02) with  TLA2024 ?
- instead of isolated 12v power supply, use high-driver with bootstrapping
- step-down (synchronized) sop8 alternative?  https://www.lcsc.com/product-detail/DC-DC-Converters_Anpec-Elec-APW7301KAI-TRG_C115136.html
- OLED display
- USB type-c connector
- -SPI outbreak for additional hardware /CAN
- 5V outbreak

## Simplifications
* Use ESP32 ADCs


## Common Ground
- do we need common ground?
- if not we can remove the BCCU DCDC
- makes shunt current sensing easier
- add R_sense and current amplifier (without common GND, put this on the low side and use ordinary OPV)

# Firmware
. minimum ontime lower FET, otherwise
- handle negative current (set REC=1)
- choose a framework:
-- https://github.com/RT-Thread/rt-thread
-- https://github.com/cesanta/mongoose-os
-- https://github.com/SmingHub/Sming
null- Zephyr
null- Mbed

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