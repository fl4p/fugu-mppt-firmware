
* daily rolling stats
* calibration data
  * Vin, Vout, I factors
  * NTC temp sensor
* hardware config
  * id or name
  * ADC config (ADS, INA226)
  * max input voltage, max out, max input current, max output current, max power
  * max temp
  * pin-out override
    * NTC adc channel
    * SD/EN,IN,LIN,BFlow
  * backflow switch position (pv or bat)
  * current sensor positions (pv or bat)
  * current sensor bidirectional (y or n)
  * max loop lag
* Charger Config
  * battery voltage
  * battery chemistry
  * max current (not the hw limit)
  * topping mode
    * enable/disable
    * voltage, max current
  * linear time voltage ramp
    * increase max voltage linearly after reaching a threshold
* wifi networks




```

ssid=mentha
pw=secret

#ssid=df

iin_calibration=1.0345336
vout_calibration=2.3434
vin_calibration=5.1234
```