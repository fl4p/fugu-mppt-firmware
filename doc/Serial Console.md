# Serial Console

You can use serial commands to interact with the charger while it is running.
It is also suitable for implementing automated test.

Each successful command is confirmed with:

```
OK: <cmd>
```

# General Commands
* `wifi on`, `wifi off` disable wifi and telemtry. disabled wifi usually increases the control loop rate.
* `restart`: reset the MCU
* `speed <float>` set  tracking speed (default 1.0)
* `fan <float>` set fan speed 0-100
* `led <RRGGBB>`, `led <RGB>` set the LED rgb color in hex or short hex (e.g. `led 33ff33` or `led 3f3`)
* `sweep`: starts a global MPP scan / search
* `+<int>`, `-<int>` manual perturb the buck duty cycle (for testing)
* `reset-lag`: resets max lag statistic
* `scan-i2c`: run a i2c bus scan

# Manual PWM Commands

* `dc <int>` set the buck duty cycle and put the charger in manual PWM mode. No tracking, protection only.
* `+<int>`, `-<int>` relative buck duty cycle perturbation step. be careful with large positive jumps, this can cause extreme
  current transients destroying the switches. also availble in tracking mode to test tracker recovery.
* `ls-disable`, `ls-enable`: disable/enable low-side switch (i.e. diode emulation, sync rectification).
* `bf-enable`, `bf-disable`: enable/disable the backflow switch. An enabled back-flow switch will allow current flow
  from the output to the input (bat to solar).
* `mppt` switches back to MPP tracking mode
