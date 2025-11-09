This is configuration for a 12V power supply.

It uses `forced_pwm=1` in `converter.conf` to achieve a much better output voltage regulation.

You can set output voltage with `vout_max` in `charger.conf`.

In `limits.conf` you can set additional current and voltage hard-limits.


# TODO
- need to increase controller speed for Vout
- able to set Iout or Vout control priority