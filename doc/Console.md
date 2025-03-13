# Serial Console

You can send text commands on the UART (or telnet) to interact with the charger while it is running.
It is also suitable to implement automated tests. Input and output are multiplexed across UART, USB serial JTAG and
telnet.

Default baud rate for serial UART is 115200. Terminate command with `\n` or `\r` (new line).
The charger confirms a successful command with:

```
OK: <cmd>
```

# General Commands

* `wifi on`, `wifi off` disable wifi and telemetry. disabled wifi usually increases the control loop rate.
* `wifi-add <ssid>:<password>`
* `ip` show IP address
* `restart`: reset the MCU
* `speed <float>` set tracking speed (default 1.0)
* `fan <float>` set fan speed 0-100
* `led <RRGGBB>`, `led <RGB>` set the LED rgb color in hex or short hex (e.g. `led 33ff33` or `led 3f3`)
* `sweep`: starts a global MPP scan / search
* `+<int>`, `-<int>` manual perturb the converter duty cycle (for testing)
* `reset-lag`: resets max lag statistic and displays [rtcount](Real-time%20Counter.md) statistics
* `scan-i2c`: run a i2c bus scan
* `sensor`: display sensor data
* `mem`: display heap and PSRAM sizes
* `ota <url>`: download and flash a new app image from a HTTP(S) URL
* `hostname <hostname>` set device's hostname
* `set-config <file> <key> <value>` set a config file and write it to flash

# Manual PWM Commands

* `dc <int>` set the converter duty cycle and put the charger in manual PWM mode. No tracking, protection only.
* `+<int>`, `-<int>` relative converter duty cycle perturbation step. be careful with large positive jumps, this can cause
  extreme
  current transients destroying the switches. also availble in tracking mode to test tracker recovery.
* `ls-disable`, `ls-enable`: disable/enable low-side switch (i.e. diode emulation, sync rectification).
* `bf-enable`, `bf-disable`: enable/disable the backflow switch. An enabled back-flow switch will allow current flow
  from the output to the input (bat to solar).
* `mppt` switches back to MPP tracking mode
* `short-ls`

# Telnet

Use any telnet client to connect on port 23. No password is required. Only one connection at a time.


Connect from Home Assistant:

* install "Terminal & SSH" add-on
* in add-on Configuration add `busybox-extras` to Packages