# todo

* remove cmdAnf, just leave anf disabled, strip it, 
  * comment out this line:   UART_LOG("  ANF(span=%4.0f):  Nstd= %7.3f   Sstd=%7.3f   NSR=%7.3f%s", s->anf.span,
* create a wifi service

* review conf files
* add logic
* profile the active control loop (mppt.update) — on the mock it stays in UV and early-returns
  (1.4us); drive it active (`dc <n>` / non-zero-V mock) or measure on fry/flat to get real
  per-section rtcount for the control path (5 PD ctrls, computeSyncRectRatio, fan/digitalWrite).
  this is what drives loopRt to ~64% on a live converter; the sampler path is not the bottleneck.
  * vconv (CONFIG_FUGU_WITH_VCONV + config/lab/vconv_mock) DOES exercise the full control path and
    is great for *profiling*. Its old timing-FRAGILITY (sub-us perturbations flipping it between
    MPP ~815W and a ~60W duty-pinned bad point) was a co-sim DESYNC: when the RT loop overran, piled-up
    ADC notifications collapsed into one wakeup but the plant only advanced ONE dt while the controller
    thought several samples elapsed. FIXED in 285cb181 — ADC_VConv::hasData() now steps the plant by
    the coalesced tick count (stepSeconds(dt*ticks)), so plant-time tracks samples-consumed regardless
    of jitter. Re-verified on 139C: locks 814-815W at Vin~64.6V and advances to slow-P&O, stable. So
    vconv is now OK for validating control-behavior changes too (still confirm anything risky on
    fry/flat). profiling lesson: rtcount means are inflated by ADC-ISR preemption; the `min` column is
    the truer pure-compute figure.
* ina226 588us conv time timeouts
* critical commits
  * the esp32 adc fix
  * the RT core pining

The transitive include chain (adc.h → store.h → littlefs/nvs) is too heavy to shim
ad-hoc — the host path isn't worth it. The canonical proof is on-target. Let me
first confirm the test is actually invoked in the runner (not just declared), then
decide on running



❯ is there any advertising ble protocol to publish bms data?

⏺ Yes — a few options, with one obvious winner for your HA context: 
BTHome v2 (https://bthome.io) — the de-facto open standard for BLE-advertised sensor data, natively supported by Home Assistant without any custom integration. Encodes
typed measurements in the BLE Service Data field (UUID 0xFCD2). Already has all the fields you'd want for BMS publishing: voltage, current, power, energy, temperature,
battery %, and even packet/sequence counters. ESPHome has a bthome component, and Shelly/Xiaomi/etc. devices use it. This is what I'd reach for.

Other options, less appealing:
- Eddystone-TLM — Google beacon format with battery voltage + temperature telemetry. Deprecated and only carries those two scalars.
- Victron Instant Readout — proprietary (but reverse-engineered) manufacturer-data format used by their SmartShunt / BMV / SmartSolar. HA has victron_ble. Real-world
  precedent that a BMS-class device can broadcast all live data this way, but it's their namespace.
- Ruuvi RAWv2 — sensor-tag format, includes battery voltage, but the schema is fixed and not really meant for BMS.
- Bluetooth SIG Battery Service (0x180F) — GATT only, single byte (%), not advertising.

For batmon-ha specifically, the natural shape would be: optional "publisher" mode that re-emits decoded BMS readings as BTHome advertisements (would need a BLE
peripheral-capable adapter and bluez peripheral role, which is the catch on HA OS hosts). Want me to sketch what that'd look like?

✻ Baked for 22s

❯ is bthome also suitable for mppt chargers?

⏺ Mostly yes, with caveats. The numeric telemetry maps cleanly — PV voltage/current/power, battery voltage/current, charge power, daily/total energy (Wh, kWh), temperature,
all native BTHome v2 types. You can repeat the same measurement type within one packet (BTHome distinguishes by order), so two voltages / two currents / two energy counters
works.





and can your review the sdkconfig situation? it is not drifting anymore. take a look at the other sdkconfig.* files. i want everything
to work on esp32 and esp32s3 target and with all the WITH_* build flags. any file updates needed? should we remove the sdkconfig from git?


charger.h:  4. Post-termination, only Ah-release or the 3.27 V floor unlatches — fine for solar, could stick "full" in high-load AC-coupled setups.

- N_cells log: formula floor(Vbat_max/cv_eoc) and units check out; the four-field log (N_cells, Vbat_max, cv_eoc, Vbat_fallback) makes the 13.6 V ×
  cv_eoc=3.50 → 3S misconfig immediately obvious.

- review voltage detection

- test conf-editor.html

- write a python script that interacts with the device over the serial console
    - test all possible console commands in a meaningful order
    - accept a cli parameter that informs if the device is using a mock setup (fake ADC readings, not driving any PWM)

- dod counter not working properly
- update console documentation
- questdb binary wire protocol
- hardware tests https://github.com/fl4p/fugu-mppt-firmware/issues/44
    - backflow switch
    - HS switch (and short LS)
    - output impedance measurement https://github.com/fl4p/fugu-mppt-firmware/issues/47
      -
            * detect high impedance battery connection

        * https://h.fabi.me/grafana/d/f4a22deb-8528-427d-9473-4e7b06c6d874/fugu-mppt?orgId=1&from=1741693099256&to=1741704737323

- move away from floating point arithmetic (https://github.com/espressif/idf-extra-components/tree/master/iqmath)
- make notch filter pluggable
  -provisioning overrides
    * eg ina22x_resistor, voltage dividers etc that persists when flashing a new provisioning image
    * also wifi, coil. "extend" a config set?

$$V_{term} = CVmin + I \cdot \frac{CVmax - CVmin}{k\cdot C} $$

```
V=45.3/28.01 I=12.1/19.00A 549.1W 35℃37℃ 444sps  0㎅/s CCM(H|L|Lm)=1285| 754| 762 st=⇡MPPT,1 lag=3107㎲ N=384851 rssi=-52
V=45.4/28.06 I=12.1/18.95A 546.4W 35℃37℃ 444sps  0㎅/s CCM(H|L|Lm)=1287| 756| 760 st=⇡MPPT,1 lag=3107㎲ N=386187 rssi=-53
sweep
I (4481957) main: received serial command: 'sweep'
PWM disabled (duty cycle was 1288)

I (4481968) mppt: Start sweep
I (4481972) mppt: Start calibration
I (4481976) sensor: vin reset calibration
I (4481980) sensor: iout reset calibration
I (4481982) sensor: ntc reset calibration
I (4481986) sensor: vout reset calibration
I (4481991) main: OK: sweep
V=69.5/26.35 I= nan/ 0.07A   nanW 35℃37℃ 2743205264sps  0㎅/s CCM(H|L|Lm)=   0|   0| 759 st=   CV,1 lag=3107㎲ N=4 rssi=-53
(186504320): Current above threshold 18.80 (pwm=0)
(186504918): Sync rect enabled
I (4481974) mppt: Stop sweep after 871.45s at controlMode=CV (limIdx=1, tgt=29.00, act=27.88) PWM=0, MPP=(0.0W,PWM=0,0.0V)
I (4482022) plot: Not enough data to plot V
I (4482027) plot: Not enough data to plot D
I (4482050) store: Wrote /littlefs/stats (size 32)
I (4482053) flash: Wrote flash value /littlefs/stats
I (4482426) sampler: Sensor iout calibration: avg=0.1055 std=0.000332
I (4482427) sampler: Sensor iout offset-calibrated: 0.105500
I (4482433) sampler: Sensor vout calibration: avg=26.3244 std=0.000000
I (4482436) sampler: Sensor vin calibration: avg=69.4100 std=0.000000
I (4482875) sampler: Sensor iout calibration: avg=-0.0315 std=0.000457
I (4482875) sampler: Sensor iout offset-calibrated: -0.031536
I (4482875) sampler: Calibration done!
(187412263): Converter enabled
(187412518): Current below threshold nan (pwm=16)
(187412671): Back-flow switch disabled
(187412737): Sync rect disabled
(187421706): converter: CCM -> DCM (M=0.38, I=0.11, ∆I/2=5.51, pwm=16)
I (4482888) mppt: Near MPP, slow-down, set dutyCycle 1294 (powerSample=2.77)
V=69.4/25.91 I=0.05/ 0.12A   3.2W 35℃37℃ 144sps  0㎅/s DCM(H|L|Lm)=  18| 123| 123 st=⇣MPPT,1 lag=3611㎲ N=442 rssi=-53
I (4483880) mppt: Reset maxPower to 0 (<3.007 * 90%)
V=69.4/25.99 I=0.03/ 0.08A   2.3W 35℃37℃ 444sps  0㎅/s DCM(H|L|Lm)=  31| 123| 123 st=↑MPPT,1 lag=3611㎲ N=1778 rssi=-51
V=69.4/25.98 I=0.04/ 0.09A   2.5W 35℃37℃ 443sps  0㎅/s DCM(H|L|Lm)=  37| 123| 123 st=↓MPPT,1 lag=3611㎲ N=3114 rssi=-52
V=69.4/25.89 I=0.03/ 0.09A   2.4W 35℃37℃ 443sps  0㎅/s DCM(H|L|Lm)=  43| 123| 123 st=↑MPPT,1 lag=3611㎲ N=4450 rssi=-53
V=69.3/25.91 I=0.04/ 0.11A   3.0W 35℃37℃ 443sps  0㎅/s DCM(H|L|Lm)=  50| 123| 123 st=↑MPPT,1 lag=3611㎲ N=5787 rssi=-53
V=69.2/25.91 I=0.04/ 0.11A   2.8W 35℃37℃ 443sps  0㎅/s DCM(H|L|Lm)=  54| 123| 123 st=↑MPPT,1 lag=3611㎲ N=7123 rssi=-54
E (4500180) ina22x: 170000 ti
```

```
V=59.8/27.61 I=8.56/18.01A 511.8W 38℃40℃ 443sps  0㎅/s CCM(H|L|Lm)= 993|1054|1054 st=⇡MPPT,1 lag=3611㎲ N=596963 rssi=-51
sweep
I (7726171) main: received serial command: 'sweep'
PWM disabled (duty cycle was 984)

I (7726182) mppt: Start sweep
I (7726187) mppt: Start calibration
I (7726190) sensor: vin reset calibration
I (7726193) sensor: iout reset calibration
I (7726196) sensor: ntc reset calibration
I (7726199) sensor: vout reset calibration
I (7726202) main: OK: sweep
V=70.3/26.52 I= nan/-0.26A   nanW 38℃40℃ 3126459500sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=   CV,1 lag=3611㎲ N=3 rssi=-52
(3430717120): Current above threshold 17.89 (pwm=0)
(3430717724): Sync rect enabled
I (7726188) mppt: Stop sweep after 1346.25s at controlMode=CV (limIdx=1, tgt=29.00, act=27.39) PWM=0, MPP=(0.0W,PWM=0,0.0V)
I (7726241) plot: Not enough data to plot V
I (7726245) plot: Not enough data to plot D
I (7726268) store: Wrote /littlefs/stats (size 32)
I (7726271) flash: Wrote flash value /littlefs/stats
I (7726641) sampler: Sensor iout calibration: avg=0.0958 std=0.000157
I (7726641) sampler: Sensor iout offset-calibrated: 0.095813
I (7726645) sampler: Sensor vout calibration: avg=26.5019 std=nan
I (7726653) sampler: Sensor vin calibration: avg=70.2534 std=0.000000
I (7727089) sampler: Sensor iout calibration: avg=-0.0281 std=0.000636
I (7727090) sampler: Sensor iout offset-calibrated: -0.028057
I (7727090) sampler: Calibration done!
(3431626624): Back-flow switch disabled
V=70.3/26.49 I=0.03/ 0.08A   2.2W 38℃40℃ 172sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=529 rssi=-52
V=70.3/26.48 I=0.02/ 0.05A   1.5W 38℃40℃ 446sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=1871 rssi=-53
V=70.2/26.48 I=0.02/ 0.04A   1.2W 38℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=3212 rssi=-54
V=70.2/26.47 I=0.01/ 0.04A   1.1W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=4554 rssi=-54
V=70.2/26.46 I=0.01/ 0.04A   1.0W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=5895 rssi=-52
V=70.2/26.46 I=0.01/ 0.03A   0.9W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=7236 rssi=-51
V=70.1/26.45 I=0.01/ 0.03A   0.8W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=8577 rssi=-52
V=70.2/26.46 I=0.01/ 0.03A   0.8W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=9918 rssi=-54
V=70.2/26.45 I=0.01/ 0.03A   0.8W 37℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=11259 rssi=-52
V=70.2/26.45 I=0.01/ 0.03A   0.7W 36℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=12600 rssi=-50
V=70.2/26.45 I=0.01/ 0.03A   0.8W 36℃40℃ 445sps  0㎅/s CCM(H|L|Lm)=   0|   0|1063 st=  N/A,1 lag=3611㎲ N=13941 rssi=-51

```

TODO nan is injected into filtering

* after calibration wait until ewm.avg is finite, filters have settled, then sweep

## components

add named components (mqtt, adc, etc)

* similar to a service
* wraps / abstracts a building block (hw driver or software component)
* interface:
    * begin(), end()
    * status
    * setLogLevel()
    * config namespace?

Add a small service architecture (similar to systemd services, but way simpler).
A services wraps / abstracts a building block / module (such as BLE server, the pwm driver or another software
component)
Services have start(), stop(), reload(), loop() functions, and report a status (running, stopped, failed).
Each service has its own log level, which can be adjusted at run-time.

* user has access over console (e.g. restart, query stats, enable debug log, change config)


* boost mode, forced_pwm, reverse current
    * the largerDecrease update block causes excessive reverse current into the power supply (bat/LV terminal)
    * ignoring largerDecrease fixes the problem (TODO: verify, is this still true?)
* iout midpoint calibration fix! with ina226 we get an offset of
  0.9A ! https://github.com/fl4p/fugu-mppt-firmware/issues/28
* store last warnings, errors
* simulate ina226 external reset
* inspect ADC noise with scope-client
* manual sync control to find inductivity

# INA226 alert timeouts trip the loop-latency shutdown (fry/flat)

2026-05-22: "Loop latency high (<200 Hz), shutdown!" fired 21x on fry/flat under load. Root cause is the
INA226, not WiFi/scope/the console command next to it (scope wasn't streaming, `lag` stayed ~2.7ms = no
compute stall — the RT loop is *blocked waiting* for ADC data).

- On fmetal **both vout (ina226 ch0) and iout (ch1) are on the INA226** (vin/ntc on internal adc). So a
  missed conv-ready alert freezes `Vout->numSamples` — which is exactly the watchdog metric (main.cpp:506).
- Alert path is edge-fragile: continuous + non-latched conv-ready, FALLING-edge ISR (GPIO 41), flag cleared
  by reading MASK_EN in `hasData()` (ina226.h:364/379). Late read vs the ~1.1ms conversion → missed edge.
  Chronic ~54 timeouts/s (`ina22x: <n> timeout!` logs every 20000th); each miss blocks 3ms (`wait(3)`) in
  the RT loop. `i2c_freq=800000` is aggressive (>std 400k); fry's INA226 suspected counterfeit (ina226.h:107).
- The watchdog is correct — don't touch it.

Candidate fixes (ranked):

1. `hasData()` non-blocking (`wait(0)`) so a missed/late alert costs ~0, not 3ms. Verify internal-adc pacing
   doesn't busy-spin.
2. Don't rely on the edge: poll conv-ready flag / use latched alert; drop fry i2c_freq to 400k.
3. Decouple vout OV-protection metric from INA cadence (virtual/last-good fallback so a hiccup degrades data
   instead of shutting the converter down).

# filtering

- med3 doesnt really help
- consider kalman
- consider smaller f_cut for Iout RC-filter (larger R)
- tracker accumulation buffer


1. A malformed value still throws even when the key has a default. In getX (conf.h:~205-230), a parse failure (trailing
   chars / out-of-range) throws unconditionally — the def is
   only used when the key is absent, not when it's present-but-garbage. So temp_max=9O (letter O) throws rather than
   falling back to 90, and inside Limits/sensor setup that
   becomes setupErr → converter disabled. If you want "bad value → warn and use the default," that's a deliberate change
   to getX: on parse error, if a default was supplied, log a
   warning and return def instead of throwing. This is the biggest remaining gap against your stated requirement.

┌─────────────────────────────────────────┬──────┬───────────────────────────────────────────────────────────────────────────────────┐
│ File (HEAD)               │ Line │ Use │
├─────────────────────────────────────────┼──────┼───────────────────────────────────────────────────────────────────────────────────┤
│ etc/mqtt_client.py │ 52 │ username_pw_set("pv", "<redacted>") — MQTT broker │
├─────────────────────────────────────────┼──────┼───────────────────────────────────────────────────────────────────────────────────┤
│ etc/mqtt-console-logger/mqtt_console.py │ 57 │ same MQTT broker password │
├─────────────────────────────────────────┼──────┼───────────────────────────────────────────────────────────────────────────────────┤
│ etc/micropython-relay/relay.py │ 51 │ InfluxDB token for influx.fabi.me (openpe:<redacted>) — different service, same
│
│ │ │ password │
└─────────────────────────────────────────┴──────┴───────────────────────────────────────────────────────────────────────────────────┘

So the secret is reused across your MQTT broker and an InfluxDB instance, and both are exposed in the current public
tree (not just
history). Rotate <redacted> on both services — that's the only fix that matters once something's been public.

make service log obey log level setting

So the realistic choices:

- (A) portMUX snapshot (my original): ~10 lines, no behavior change. Lock held only to copy ≤4 pointers; callbacks run
  outside the lock
  (so no deadlock when a callback logs).
- (B) "array touched only by the network loop": fully lock-free, matches the async-log style — but requires moving
  fan-out out of
  vprintf_mux and an MP-safe registration handoff, and it defers core-0 sync logs.

# TODO consider B ^^^

todo desc:
ble_passkey
399533
× long Passkey for BLE pairing
ble_security
passkey
× string BLE security mode (justworks, passkey, etc.)

the ble console dosnt exit properly when the device restarts, ctrl+c doesnt work

I want to implement a tool to find the coil inductivity with just the buck converter (no external current probe).
It has 3 sensors, in/out voltage, output current. its input is connected to a solar panel, output to a battery.

This is what claude already told me:

```
A few ways around this if you don't have a current probe:

* Run it into DCM. In discontinuous conduction mode the voltage ratio does depend on L. With a known light load R, V_out/V_in = 2 / (1 + √(1 + 8·L·f_sw / (D²·R))). Measure V_out/V_in at that operating point, solve for L.
* Find the CCM/DCM boundary. Sweep load current down until you see the converter just enter DCM (kink in V_out vs. I_load, or you observe it on a scope). At the boundary, I_load,crit = V_out(1−D)/(2·L·f_sw), which gives L.
* Output voltage ripple. ΔV_out ≈ ΔI_L / (8·f_sw·C_out). If you trust your output cap value, AC-couple the scope on V_out, measure ripple, back out ΔI_L, then L.
* Transient step response. Apply a load step and watch how fast the inductor current can slew. The slope is bounded by V_L/L.
```

* sampling rate is 511sps

What do you recommend? You have all the data available at InfluxDB (although downsampled).
If you need HF-data, use scope-client.

write md doc with a bit of theory:

* methods on how to obtain L0, with and without coil current probe
* how a broken current sensor affects the measurement
    * gain and offset error
* how a wrong L0 (too high or too low) value affects diode emulation
* then how this script works (and why you chose that method)

add charts of the CCM and DCM coil current (can be ascii)

scope proto: zigzag-delta + varint

`version` command?

* can we run the LS sweep?
* add this to the coil measurement document. including both charts, separated, one for flat and one for fry

The L-wiggle is a delivered-charge ripple, and its source is the open-loop, integer-quantized low-side turn-off time. In
computePwmRectMax() (buck.h:233):

pwmRectMax = round(pwmCtrl · pwmRectRatioDCM)        // LS count, whole PWM ticks
pwmRectRatioDCM = rectCtrlRatio(M_wce) = 1/M_wce − 1 // buck, voltages only

Three things make the charge per cycle (hence apparent L) swing as duty sweeps:

1. Rounding to whole ticks. Each LS tick is 1/(fsw·pwmMax) ≈ 12.5 ns. The ideal LS-off lands between ticks; round()
   stair-steps it, so the turn-off
   alternately sits a hair before the inductor zero-crossing (body-diode finishes → Vf loss → less charge) or after it (
   reverse current pulls charge back). Both
   reduce net Iout, so L reads high — and which side you're on beats against duty.
2. A fixed offset — gate delay + dead-time + the deliberate conservative bias voltageMaxErr = 0.01 (buck.h:431, turns LS
   off ~2 % early on purpose for
   safety). This shifts the whole pattern toward the body-diode side.
3. The asymmetry around the zero-crossing (Vf one side, reverse current the other) rectifies that timing jitter into the
   periodic charge error you see.

It's open-loop because the converter is sensorless on inductor current — it computes the turn-off from voltages and L0
instead of detecting the zero
crossing.

How to reduce it — ranked by effort/payoff

1. Error-feedback (dither) the LS-count rounding — cheapest, most targeted.
   Instead of round(), carry the rounding remainder forward in an accumulator so the average LS-off time tracks the
   ideal continuously:
   acc += pwmCtrl · ratio; ls = (int)acc; acc -= ls; // sigma-delta the fractional tick
   This converts the duty-pinned beat into high-frequency jitter that the output cap/battery filter out. ~3 lines,
   RT-safe (just an int add in pwmPerturb), no
   hardware change. Directly kills the quantization half of the oscillation.

2. Calibrate and apply the fixed timing offset.
   Use measure_coil.py --ls-sweep --hs N (§6) on each board to find the actual offset between the commanded LS count and
   the true Iout peak (zero-crossing),
   then add it as a constant pwmRectOffset after the round — and, if the voltage sensors are trustworthy on that board,
   trim voltageMaxErr from 0.01. This
   centers operation on the zero-crossing so you ride the flat top of the peak instead of a steep side, removing the DC
   bias and shrinking the swing.

3. Close the loop on reverse current — the real fix, biggest change.
   The oscillation only exists because turn-off is computed, not measured. Servo pwmRect down whenever reverse current
   is detected (the firmware already has a
   "reverse-current pullback"; promote it to an integral controller fed by the Iout dip or a switch-node comparator).
   That removes the dependence on L0/M
   accuracy entirely — and incidentally makes the L0 value almost irrelevant to timing.

maybe we should turn off wifi, i noticed pwm glitches with wifi enabled. we can steer over ble

⏺ While the build runs, here's the auto-calibrate answer — and yes, the offset knob I just added (rectOnOffset /
coil.conf::rect_offset) is exactly what an
auto-calibrator would tune.

The physics makes it tractable: the thing to learn is a single scalar — the fixed dead-time/gate-delay in counts.
rectCtrlRatio·HS already handles the
operating-point (M, HS) dependence, so a dead-time (fixed time → fixed counts) should be ~constant across duties. So
auto-cal trims one constant slowly, not
a per-cycle search. (Worth confirming constancy by running --ls-sweep at 2–3 HS values; if peak−ideal drifts with HS,
there's also a ratio error to fix
separately.)

The signal is asymmetric, which dictates the strategy. The Iout peak is broad and flat on the body-diode side (poor
gradient — P&O would just wander there)
but the reverse-current cliff is sharp. And overshoot = reverse current = the destroy-the-LS-switch direction (§4). So
don't hill-climb to the peak — climb
the offset up until reverse-current onset, then back off a margin.

I'd offer two tiers:

Tier 1 — host-driven, safe, I can add now. measure_coil.py --ls-sweep already computes peak − ideal. Add --apply: after
the sweep it writes
coil.conf::rect_offset = (peak − ideal − margin) via set-config. Supervised, one-shot, bounded by the sweep's own
--i-max and "passed the peak" stop. No
autonomous RT behavior on a live converter. This is the low-risk "auto-calibrate."

Tier 2 — on-device autonomous servo. A slow P&O in loopLF (core 0, never the RT loop):

- Gate hard: steady DCM, Iout above an SNR floor, voltages stable, not in MPPT-sweep/transient/protection.
- Raise rectOnOffset by 1 ct per ~second; watch averaged Iout. Stop the moment Iout rolls over or the existing
  reverse-current pullback fires (use that as
  the hard "too far" signal — it's a cleaner landmark than the broad peak), then back off a safety margin and hold.
- Clamp rectOnOffset to a conservative max; debounce-persist the learned value to coil.conf::rect_offset; restart from
  the stored value on boot.
- Always err to the early/body-diode side.

Tier 2 (on-device auto-cal)
remains a sim-first follow-up whenever you want it.

--ls-sweep --apply: plausibility checks, apply and re-test after applied
write into doc
re-run HS sweep

/measure_coil.py should show a final plot

* how does measure_coil get the values?

let wifi use the svc api (instead of wifi off)

* add an optional argument to wifi off <timeout> to disable wifi for some minutes

#3 — why the oscillation survives the dither (hypothesis)

I initially suspected the dither was frozen during a held-duty sweep, but the code refutes that: in manual mode the RT
loop calls converter.pwmPerturb(0)
every iteration (main.cpp:661-663), which re-runs computePwmRectMax(), so rectDitherErr does iterate and toggle the LS
count N↔N+1 each RT cycle,
time-averaging to the fractional ideal. The dither is working as designed.

The problem is what it's averaging over. The dither assumes the Iout-vs-LS response is locally linear, so that toggling
N↔N+1 yields the fractional-point
result. But we measured that response in the ls-sweep and it's sharply asymmetric around the operating point: a flat
body-diode plateau on the short side (no
penalty) and a steep reverse-current cliff on the long side. Averaging a toggled input across that nonlinearity gives a
biased Iout (Jensen's inequality),
and the bias depends on where the fractional ideal sits relative to the integer grid — which slides with H. That's the
residual ~115-count oscillation the
dither can't remove.

And the kicker: rect_offset=+100 moves the operating point toward the cliff (we confirmed auto_LS now sits only ~17 ct
below the Iout peak), i.e. into the
most asymmetric region — so the calibration, which buys efficiency, plausibly makes this oscillation worse, not better.
That's consistent with our earlier
dither A/B showing no clear improvement.

To confirm (not over-theorize), the clean test is on the dry_mock sim: sweep with dither on/off and rect_offset 0 vs
100, and see whether oscillation tracks
proximity-to-cliff rather than dither state. That needs a sim build — I'd like your go-ahead before setting it up rather
than starting it blind.

* MCPWM instead of ledc?
* re-arange

MCPWM vibes
Sketch the implementation of the MCPWM driver for the synchronous buck convert preserving the diode-emulation.

* frequency and counts configurable
* there is a function that computes the highest counts for frequency to achieve best pwm resolution
* dead time is configurable
* it is possible to attach it to an GPIO for instant shutdown (no CPU involved)
* it supports multiple, interleaved channels
* Continuous glitch free updates of the duty cycle

braintstorm->sketching->spec->impl

wifi lively check with mqtt host and roam ??

- write a dead-time optimization script

does the charger compensate battery current to zero after termination?
does the recharge_dod work?

2. Gate driver + diode emulation transitions
   Above, plus verify the CCM↔DCM rectifier-cutoff timing across the `pwm` (HS) sweep — i.e. that `pwmRectMax`/
   `rect_offset` produces the right LS turn-off
   relative to the inductor zero-crossing. Still mock-ADC, but exercises the full `computePwmRectMax` path.
3. Full converter under load
   All of the above plus real Vin/Iin/Vout/Iout via PSU + e-load + current probe — closes the loop on
   duty/MPPT/protection too. Requires more wiring and a
   bench rig from `website/docs/lab/automated-bench-tests.md`.


- update the tests for the charger.h
- check each input parameter description for contracts

# command tests

```python
# command acceptance:
t = [
    ("dc 100", True),
    ("dc", False),
    ("dc -1", False),
    ("dc 1.0", False),
    ("dc -0.9", True),
    ("dc 2040", False),
]

"""
# closed scope test
* use picoscope to test PWM signals
* use both channels to monitor Ctrl and Sync (buck: HS and LS) channels
* accept configuration for scope attenuation (e.g. x10, x50)
before the actual pwm test, perform a systems check:
use digitalWrite() to write a 0,1,0 sequence to the two PWM pins for channel identification and to verify that
the signal is actually picked up by the scope channels.
track the high-level voltage (the scope probes might have a wrong attenuation) and bail if it is below 1.5V.

with the scope channel <-> gpio pin map, we can now start the PWM driver test sequences:

"""

pwmCtrlMax = 2048
cmax = pwmCtrlMax
smin = pwmSyncMin
tq = 1 / (fsw * pwmMax)
rtol = lambda a, b, tol: abs(a - b) / abs(b)
t = [
    ("dc 1 0"),  # check ton(HS) < 1.5 *tq, rtol(ton(LS), smin * tq) < 0.01
    ("dc 2 0"),  # check ton(HS) < 2.5 *tq, rtol(ton(LS), smin * tq) < 0.01
    ("dc 10 0"),  # check ton(HS) < 12 *tq, rtol(ton(LS), smin * tq) < 0.01
]
```




alias idf-noble='WITH_BLE=0 idf.py -B build-noble'








# panic:

```
ERR: vconv bat r_bat 1
V=76.0/36.43 I=0.00/ 0.00A   0.0W 1℃45℃ 6809sps  0㎅/s CCM(H|L|Lm)=   0|   0|2045 st= MANU,0 lag=994㎲ N=235232 rssi=0
> vconv set r_bat 1
I (261992) main: received serial command: 'vconv set r_bat 1'
vconv: r_bat=1
V=76.0/12.59 I=0.00/ 0.00A   0.0W 1℃45℃  0sps  0㎅/s CCM(H|L|Lm)=   0|   0|2045 st= MANU,0 lag=994㎲ N=316786 rssi=0
OK: vconv set r_bat 1
> dc 1
I (266654) main: received serial command: 'dc 1'
I (266654) main: Switched to manual PWM
V=76.0/ 0.00 I=0.00/ 0.00A   0.0W 1℃45℃  0sps  0㎅/s CCM(H|L|Lm)=   0|   0|2045 st= MANU,0 lag=994㎲ N=330770 rssi=0
OK: dc 1
(266168188): Converter enabled
(266168949): converter: CCM -> DCM (M=-0.00, I=0.00, ∆I/2=-0.00, pwm=1)
I (266656) mppt: Reached target duty cycle 1
V=76.0/ 0.04 I=0.00/ 0.04A   0.0W 1℃45℃ 2992sps  0㎅/s DCM(H|L|Lm)=   1|  40|  40 st= MANU,1 lag=994㎲ N=339767 rssi=0
> dc 100
I (271806) main: received serial command: 'dc 100'
V=76.0/ 0.04 I=0.00/ 0.04A   0.0W 1℃45℃  0sps  0㎅/s DCM(H|L|Lm)=   1|  40|  40 st= MANU,1 lag=994㎲ N=346224 rssi=0
OK: dc 100
(271321325): converter: DCM -> CCM (M=0.00, I=0.15, ∆I/2=0.02, pwm=13)
I (271815) mppt: Reached target duty cycle 100
V=75.9/ 3.71 I=0.19/ 3.71A  14.2W 1℃45℃ 2995sps  0㎅/s CCM(H|L|Lm)= 100|  40|1946 st= MANU,1 lag=994㎲ N=355223 rssi=0
V=75.9/ 3.71 I=0.19/ 3.71A  14.2W 1℃45℃ 3000sps  0㎅/s CCM(H|L|Lm)= 100|  40|1946 st= MANU,1 lag=994㎲ N=364229 rssi=0
> sweep
I (279877) main: received serial command: 'sweep'
PWM disabled (duty cycle was 100)

I (279877) mppt: Start sweep
I (279878) mppt: Start calibration
V= nan/ 0.48 I= nan/  nanA   nanW 1℃45℃  0sps  0㎅/s CCM(H|L|Lm)=   0|   0|1946 st=START,1 lag=994㎲ N=1 rssi=0
OK: sweep
I (279877) mppt: Stop sweep 123.48s mode=MPPT (lim=4 tgt=1000.00 act=13.76) PWM=100 MPP=(0.0W,0,0.0V)
(279391137): Current above threshold 3.71 (pwm=0)
(279391222): Back-flow switch enabled
(279391262): Sync rect enabled
I (279878) mppt: Start sweep
I (279879) mppt: Start calibration
I (279879) sensor: vin reset calibration
I (279879) sensor: ntc reset calibration
I (279879) sensor: vout reset calibration
I (279926) plot: Not enough data to plot V
I (279930) plot: Not enough data to plot D
I (279948) store: Wrote /littlefs/stats (size 32)
I (279948) flash: Wrote flash value /littlefs/stats
I (279946) sampler: Sensor vin calibration: avg=76.0000 std=0.000000
I (279947) sampler: Sensor vout calibration: avg=0.0000 std=1.673200
I (279947) sampler: Sensor iout calibration: avg=0.0000 std=0.878408
I (279948) sampler: Sensor iout offset-calibrated: 0.000000
I (279948) sampler: Sensor ntc calibration: avg=0.9000 std=0.000000
I (279948) sampler: Calibration done!
(279461545): Back-flow switch disabled
(279462078): Converter enabled
(279462489): converter: CCM -> DCM (M=-0.00, I=-0.05, ∆I/2=-0.01, pwm=15)
(279464103): converter: DCM -> CCM (M=0.00, I=0.17, ∆I/2=0.09, pwm=15)
(279464861): Current above threshold 0.26 (pwm=15)
(279465045): Back-flow switch enabled
(279465075): Sync rect enabled
I (282022) mppt: Stop sweep 2.08s mode=CV (lim=1 tgt=14.60 act=14.53) PWM=397 MPP=(210.7W,397,75.0V)
I (282023) mppt: PWM fade to 397 stopped at controlMode CV
I (282027) mppt: Grouping 3 V points (75.21,174.07)~(76.00,0.37) into 100 bins, binW=0.008

174 ┼─────────────────────────────────────────────────╮                                                  
169 ┤                                                 │                                                  
164 ┤                                                 │                                                  
159 ┤                                                 │                                                  
153 ┤                                                 │                                                  
148 ┤                                                 │                                                  
143 ┤                                                 │                                                  
138 ┤                                                 │                                                  
133 ┤                                                 │                                                  
128 ┤                                                 │                                                  
123 ┤                                                 │                                                  
117 ┤                                                 │                                                  
112 ┤                                                 │                                                  
107 ┤                                                 │                                                  
102 ┤                                                 │                                                  
 97 ┤                                                 │                                                  
 92 ┤                                                 ╰───────────────────────────────────────────────── 
  P|V     75.2 .. 76




assert failed: block_locate_free tlsf_control_functions.h:618 (block_size(block) >= *size)


Backtrace: 0x40382cb5:0x3fcc1e30 0x40382c81:0x3fcc1e50 0x4038b445:0x3fcc1e70 0x40389666:0x3fcc1f90 0x403890c3:0x3fcc1fb0 0x40376237:0x3fcc1fd0 0x40376259:0x3fcc2000 0x40375eca:0x3fcc2020 0x4038b4c5:0x3fcc2040 0x4210e801:0x3fcc2060 0x4203682e:0x3fcc2080 0x4203699d:0x3fcc20c0 0x42030d61:0x3fcc20f0 0x4201772a:0x3fcc2210
--- 0x40382cb5: panic_abort at /Users/fab/dev/esp/idf5.5/components/esp_system/panic.c:469
--- 0x40382c81: esp_system_abort at /Users/fab/dev/esp/idf5.5/components/esp_system/port/esp_system_chip.c:87
--- 0x4038b445: __assert_func at /Users/fab/dev/esp/idf5.5/components/newlib/src/assert.c:80
--- 0x40389666: block_locate_free at /Users/fab/dev/esp/idf5.5/components/heap/tlsf/tlsf_control_functions.h:618
--- (inlined by) tlsf_malloc at /Users/fab/dev/esp/idf5.5/components/heap/tlsf/tlsf.c:444
--- 0x403890c3: multi_heap_malloc_impl at /Users/fab/dev/esp/idf5.5/components/heap/multi_heap.c:216
--- 0x40376237: aligned_or_unaligned_alloc at /Users/fab/dev/esp/idf5.5/components/heap/heap_caps_base.c:93
--- (inlined by) heap_caps_aligned_alloc_base at /Users/fab/dev/esp/idf5.5/components/heap/heap_caps_base.c:175
--- 0x40376259: heap_caps_malloc_base at /Users/fab/dev/esp/idf5.5/components/heap/heap_caps_base.c:202
--- 0x40375eca: heap_caps_malloc at /Users/fab/dev/esp/idf5.5/components/heap/heap_caps.c:84
--- (inlined by) heap_caps_malloc_default at /Users/fab/dev/esp/idf5.5/components/heap/heap_caps.c:110
--- 0x4038b4c5: malloc at /Users/fab/dev/esp/idf5.5/components/newlib/src/heap.c:24
--- 0x4210e801: operator new(unsigned int) at /builds/idf/crosstool-NG/.build/xtensa-esp-elf/src/gcc/libstdc++-v3/libsupc++/new_op.cc:50
--- 0x4203682e: std::__new_allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> >::allocate(unsigned int, void const*) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/new_allocator.h:151
--- (inlined by) std::allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> >::allocate(unsigned int) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/allocator.h:196
--- (inlined by) std::allocator_traits<std::allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> > >::allocate(std::allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> >&, unsigned int) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/alloc_traits.h:478
--- (inlined by) std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false>* std::__detail::_Hashtable_alloc<std::allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> > >::_M_allocate_node<std::piecewise_construct_t const&, std::tuple<char const* const&>, std::tuple<> >(std::piecewise_construct_t const&, std::tuple<char const* const&>&&, std::tuple<>&&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/hashtable_policy.h:2019
--- (inlined by) std::_Hashtable<char const*, std::pair<char const* const, rtcount_stat>, std::allocator<std::pair<char const* const, rtcount_stat> >, std::__detail::_Select1st, std::equal_to<char const*>, std::hash<char const*>, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<false, false, true> >::_Scoped_node::_Scoped_node<std::piecewise_construct_t const&, std::tuple<char const* const&>, std::tuple<> >(std::__detail::_Hashtable_alloc<std::allocator<std::__detail::_Hash_node<std::pair<char const* const, rtcount_stat>, false> > >*, std::piecewise_construct_t const&, std::tuple<char const* const&>&&, std::tuple<>&&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/hashtable.h:312
--- (inlined by) std::__detail::_Map_base<char const*, std::pair<char const* const, rtcount_stat>, std::allocator<std::pair<char const* const, rtcount_stat> >, std::__detail::_Select1st, std::equal_to<char const*>, std::hash<char const*>, std::__detail::_Mod_range_hashing, std::__detail::_Default_ranged_hash, std::__detail::_Prime_rehash_policy, std::__detail::_Hashtable_traits<false, false, true>, true>::operator[](char const* const&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/hashtable_policy.h:843
--- 0x4203699d: std::unordered_map<char const*, rtcount_stat, std::hash<char const*>, std::equal_to<char const*>, std::allocator<std::pair<char const* const, rtcount_stat> > >::operator[](char const* const&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/unordered_map.h:988
--- (inlined by) rtcount(char const*) at /Users/fab/dev/pv/fugu-mppt-firmware/src/etc/rt.cpp:118
--- 0x42030d61: MpptController::update() at /Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.cpp:263
--- 0x4201772a: loopRT(void*) at /Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:659
--- (inlined by) loopRT at /Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:477




ELF file SHA256: 949ab0c18

Rebooting...
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0xc (RTC_SW_CPU_RST),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x4202ee52
--- 0x4202ee52: std::enable_if<std::__and_<std::__not_<std::__is_tuple_like<float> >, std::is_move_constructible<float>, std::is_move_assignable<float> >::value, void>::type std::swap<float>(float&, float&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/move.h:222
--- (inlined by) std::pair<float, float>::swap(std::pair<float, float>&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_pair.h:309
--- (inlined by) std::enable_if<std::__and_<std::__is_swappable<float>, std::__is_swappable<float> >::value, void>::type std::swap<float, float>(std::pair<float, float>&, std::pair<float, float>&) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_pair.h:1091
--- (inlined by) void std::iter_swap<__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > > >(__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_algobase.h:185
--- (inlined by) __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > > std::__unguarded_partition<__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__ops::_Iter_less_iter>(__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__ops::_Iter_less_iter) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_algo.h:1842
--- (inlined by) __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > > std::__unguarded_partition_pivot<__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__ops::_Iter_less_iter>(__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__ops::_Iter_less_iter) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_algo.h:1857
--- (inlined by) void std::__introsort_loop<__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, int, __gnu_cxx::__ops::_Iter_less_iter>(__gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, __gnu_cxx::__normal_iterator<std::pair<float, float>*, std::vector<std::pair<float, float>, std::allocator<std::pair<float, float> > > >, int, __gnu_cxx::__ops::_Iter_less_iter) at /Users/fab/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/xtensa-esp-elf/include/c++/14.2.0/bits/stl_algo.h:1889
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x1700
load:0x403c8700,len:0xec0
--- 0x403c8700: _stext at ??:?
load:0x403cb700,len:0x31c0
entry 0x403c894c
--- 0x403c894c: call_start_cpu0 at /Users/fab/dev/esp/idf5.5/components/bootloader/subproject/main/bootloader_start.c:25
I (24) boot: ESP-IDF v5.5.1-dirty 2nd stage bootloader
I (24) boot: compile time May 24 2026 02:26:43
I (24) boot: Multicore bootloader
I (25) boot: chip revision: v0.2
I (28) boot: efuse block revision: v1.4
I (31) qio_mode: Enabling default flash chip QIO
```



TODO rename re-bulk charger conf?









Write frequency per (device, field) — over the last 7 days

flat (last write 2026-05-24 12:17, ~6h ago)


from(bucket: "ha_van")
|> range(start: -7d)
|> filter(fn: (r) => r._measurement == "mppt")
|> group(columns: ["device", "_field"])
|> count()
|> map(fn: (r) => ({ r with pts_per_sec: float(v: r._value) / 604800.0 }))
|> group()
|> sort(columns: ["pts_per_sec"], desc: true)

Wrapped in the influx CLI call I ran:

ssh fab@tm.fabi.me 'timeout 180 influx query --org openpe --raw '"'"'
from(bucket:"ha_van")
|> range(start:-7d)
|> filter(fn:(r)=>r._measurement=="mppt")
|> group(columns:["device","_field"])
|> count()
|> map(fn:(r)=>({r with pts_per_sec: float(v:r._value) / 604800.0}))
|> group()
|> sort(columns:["pts_per_sec"], desc: true)
'"'"' '

ssh fab@tm.fabi.me 'timeout 180 influx query --org openpe --raw '"'"'
from(bucket:"ha_van")
|> range(start:-1min)
|> filter(fn:(r)=>r._measurement=="mppt")
|> group(columns:["device","_field"])
|> count()
|> map(fn:(r)=>({r with pts_per_sec: float(v:r._value) / 60}))
|> group()
|> sort(columns:["pts_per_sec"], desc: true)
'"'"' '


┌───────────────────────────────────────────────────────┬──────┐
│                         Field                         │  Hz  │
├───────────────────────────────────────────────────────┼──────┤
│ E, E_today, mcu_temp, mppt_state, pwm_dir_f, pwm_duty │ 36.5 │
├───────────────────────────────────────────────────────┼──────┤
│ I, P, P_smooth, Ui, Uo                                │ 36.5 │
├───────────────────────────────────────────────────────┼──────┤
│ ntc_temp                                              │ 36.5 │
├───────────────────────────────────────────────────────┼──────┤
│ pwm_dcm, pwm_ls_duty, pwm_ls_max                      │ 23.2 │
├───────────────────────────────────────────────────────┼──────┤
│ P_filt, P_prev, dP, dP_thres                          │ 21.7 │
├───────────────────────────────────────────────────────┼──────┤
│ cv_lim_idx                                            │ 5.7  │
└───────────────────────────────────────────────────────┴──────┘

fry (last write 2026-05-24 04:03, ~14h ago)

┌───────────────────────────────────────────────────────┬──────┐
│                         Field                         │  Hz  │
├───────────────────────────────────────────────────────┼──────┤
│ E, E_today, mcu_temp, mppt_state, pwm_dir_f, pwm_duty │ 35.4 │
├───────────────────────────────────────────────────────┼──────┤
│ I, P, P_smooth, Ui, Uo, ntc_temp                      │ 35.4 │
├───────────────────────────────────────────────────────┼──────┤
│ pwm_dcm, pwm_ls_duty, pwm_ls_max                      │ 21.4 │
├───────────────────────────────────────────────────────┼──────┤
│ P_filt, P_prev, dP, dP_thres                          │ 20.4 │
├───────────────────────────────────────────────────────┼──────┤
│ cv_lim_idx                                            │ 6.4  │
└───────────────────────────────────────────────────────┴──────┘
