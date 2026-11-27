---
title: Telemetry fields
sidebar_position: 6
---

# Telemetry fields

The firmware writes one InfluxDB measurement ([Telemetry](../guide/telemetry/index.md)), `mppt`, tagged with the device hostname. This page lists its fields.

## Transports

The firmware sends the same points to every enabled transport. The table lists what each transport needs and what it sends.

| Transport          | Enable                                               | Wire                                                           |
|--------------------|------------------------------------------------------|----------------------------------------------------------------|
| UDP to InfluxDB    | `tele` service, [`tele.conf`](config/tele.md) `influxdb_host`, Wi-Fi up and clock synced (SNTP) | Line protocol to UDP port 8086, batched per datagram; `binary=1` sends the compressed binary wire instead (needs `CONFIG_FUGU_WITH_TELE_BINARY`) |
| BLE stream         | `CONFIG_FUGU_WITH_BLE_TELE`, `ble` service running, a connected client, `set-time`, `tele.conf` `ble=1`, and either `tele` stopped or `binary=1` (+ `svc rs tele`); then `tele-ble 1` | Binary wire over the NUS TELE characteristic                   |
| MQTT stream        | `CONFIG_FUGU_WITH_TELE_MQTT`, MQTT connected, clock set, and either `tele` stopped or `binary=1`; then `tele-mqtt <s>` (lease of up to 3600 s) | Uncompressed binary frames on `pv/tele/<hostname>`, published every 500 ms or when about 1 KB is buffered |
| BLE advertising    | `CONFIG_FUGU_WITH_BLE_ADV`, `ble` service running, `tele.conf` `adv_ms` (default 500 ms) | 17-byte record with a subset of fields (see below)             |

`etc/influx_binary_proxy.py` decodes the binary wire and BLE records back into line protocol. See
[Host tools](host-tools.md#influx_binary_proxypy).

:::note
Timestamps are epoch milliseconds. Configure the InfluxDB UDP listener (or the proxy's `--precision`) for `ms`.
:::

A line looks like this:

```text
mppt,device=<hostname> I=4.213,Ui=38.12,Uo=27.05,P=113.96,E=15230.4,E_today=412.7,pwm_duty=1432i 1727600000000
```

## Tag

Each point carries one tag.

| Tag      | Value                                                                  |
|----------|------------------------------------------------------------------------|
| `device` | Device hostname (`hostname` console command; defaults to `fugu-<target>-<MAC>`) |

## Fields

The firmware produces a point at most every 20 ms. While the idle power profile is on, the minimum interval is
`tele.conf` `idle_interval_ms`. Some fields appear only on every n-th point, as given in the *Rate* column. At an
interval of 1 s or more, every point carries all fields.

| Field         | Unit   | Type  | Rate          | Meaning                                                                         |
|---------------|--------|-------|---------------|---------------------------------------------------------------------------------|
| `I`           | A      | float | every point   | Current of the physical current sensor: `Iout`, or `Iin` when `Iout` is virtual (median of 5) |
| `Ui`          | V      | float | every point   | Input voltage `Vin` (median of 5)                                               |
| `Uo`          | V      | float | every point   | Output voltage `Vout` (median of 5)                                             |
| `P`           | W      | float | every point   | Power `I` × voltage on the same side (median of 5)                              |
| `E`           | Wh     | float | every point   | Total energy counter                                                            |
| `E_today`     | Wh     | float | every point   | Energy yield of the current day                                                 |
| `pwm_duty`    | counts | int   | every point   | Control-switch on-time in PWM timer counts (high side in buck mode)             |
| `pwm_dir_f`   |        | float | every 20th    | Control value of the last loop iteration (sign = duty direction)                |
| `mppt_state`  |        | int   | every 20th    | Control mode, see below                                                         |
| `P_max_today` | W      | float | every 20th    | Peak power of the current day                                                   |
| `Ro`          | mΩ     | float | every 20th, when valid | Output resistance from the last qualifying global sweep: a linear fit of `Vout` against `Iout` over the rising part of the sweep. It includes the cable, the BMS path and the battery's internal resistance. A sweep qualifies when its `Iout` span is at least 3 A, the fit's residual RMS at most 40 mV and the standard error of the slope at most 3 mΩ; a sweep that ends in a protection trip does not count. Absent before the first qualifying sweep, 6 h after the last one, and in PSU or boost mode or when `Iout` is virtual |
| `Rc`, `Rb`    | mΩ     | float | every 20th, when valid | Converter-to-bus and bus-to-pack resistance, fitted per pack-voltage frame from the output-to-pack voltage drop against output and battery current (`mqtt.conf` `bat_voltage_topic`, `ibat_topic`); a constant voltage offset is fitted out. Absent until about 200 frames fit with a residual below 40 mV, and after an hour without frames |
| `wf`          |        | int   | every 20th, `wire_protect` ≠ 0 | Output wire-fault level: 0 ok, 1 warn, 2 derate, 3 trip, -1 unverified (no estimate has data, or none for more than 3 charging days, and no level is held). With `wire_protect=1` or during the learning phase the value stays ≤ 1. See [limits.conf](config/limits.md#output-wire-fault-protection) |
| `Pw`          | W      | float | every 20th, when valid | Excess power in the output wiring from the worst estimate that produced a value within the last 2 minutes |
| `hs_short`    |        | int   | every 20th, when latched | High-side short latch: 1 low side clamped on, 2 all gates off, 3 gates off until `Vin` stays low (clamp pending), 4 low-side width ramp |
| `bf_fail`     |        | int   | every 20th, when latched | 1 while a backflow-switch failure is latched                       |
| `mcu_temp`    | °C     | float | every 40th    | MCU die temperature                                                             |
| `ntc_temp`    | °C     | float | every 40th    | Board NTC temperature                                                           |
| `lag`         | µs     | int   | every 40th    | Peak RT-loop lag ([Real-time latency](../development/debugging/real-time-latency.md)); reset by `reset-lag` and at each periodic sweep |
| `rect_off`    | counts | int   | every 40th    | Active rectifier timing offset                                                  |
| `heap`        | bytes  | int   | every 40th    | Free heap                                                                        |
| `heap_min`    | bytes  | int   | every 40th    | Lowest free heap since boot                                                      |
| `vcell`       | V      | float | every 40th, BMS fresh | Highest cell voltage from the BMS, as the charger uses it (`status` `vcell_high`). Absent while the value is stale |
| `bms_age`     | s      | int   | every 40th, after the first BMS value | Age of the last BMS cell voltage                             |
| `chg`         |        | int   | every 40th    | Charger state (`status` `Charger:` line), see below                             |
| `bal`         | s      | int   | every 40th    | In-band time of the due top-balancing run (`balance` `x/…s`); 0 when no run is due |
| `ah_full`     | Ah     | float | every 40th    | Charge deficit since the last full charge (`status` `ahSinceFull`), from the BMS charge while fresh, else the integrated battery current |
| `pwm_ls_duty` | counts | int   | every 10th, converter enabled | Rectifier-switch on-time                                         |
| `pwm_ls_max`  | counts | int   | every 10th, converter enabled | Maximum allowed rectifier on-time                                |
| `pwm_dcm`     |        | bool  | every 10th, converter enabled | Converter operates in discontinuous conduction mode              |
| `Vset`        | V      | float | every 10th, PSU mode | Output voltage setpoint (moves in PV emulator mode)                            |
| `cv_lim_idx`  |        | int   | when a limiter was active | Lowest index of the limiters that were active since the last point, see below |

Source: `MpptController::telemetry()` in [`src/mppt.cpp`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/src/mppt.cpp).

### `mppt_state`

The `mppt_state` field reports the control mode.

| Value | Mode    | Meaning                                         |
|-------|---------|-------------------------------------------------|
| 0     | `N/A`   | None                                            |
| 1     | `CV`    | Voltage limited (`Vin` min or `Vout` max)        |
| 2     | `CC`    | Current limited (`Iin` or `Iout` max)            |
| 3     | `CP`    | Power limited (thermal derating / `P_max`)      |
| 4     | `MPPT`  | Tracking the maximum power point                |
| 5     | `Sweep` | Global sweep or fade to a target duty           |

### `chg`

The `chg` field reports the charger state, in the order the `status` command checks it.

| Value | State          | Meaning                                                  |
|-------|----------------|----------------------------------------------------------|
| 0     | `charging`     | Charging toward the termination line                     |
| 1     | `CV HOLD`      | Highest cell held at `cv_eoc`                            |
| 2     | `CV HOLD balance` | CV hold of a top-balancing run                        |
| 3     | `PARTIAL HOLD` | Load-following at the partial-charge ceiling             |
| 4     | `BLOCKED`      | Charging blocked, pack too cold                          |
| 5     | `TERMINATED`   | Terminated, floating at `cv_float`                       |

A hold paused for stale [BMS data](../guide/charging/bms-integration.md) reports 0, also during a top-balancing run; `vcell` is then absent.

`bal` counts while a run is due, including the time between hold sessions. It never reports the full
`balance_hold_s`: when the run completes, is skipped or ends early, `bal` drops to 0 together with the
due flag, so the console log tells which.

For the charge stages, see [Charge termination](../guide/charging/termination.md).

### `cv_lim_idx`

The `cv_lim_idx` field reports the limiter by index.

| Value | Limiter      | Target                                              |
|-------|--------------|-----------------------------------------------------|
| 0     | `VinCTRL`    | `Vin` ≥ `Vin_min`                                   |
| 1     | `VoutCTRL`   | `Vout` ≤ charger voltage limit (or PSU setpoint)    |
| 2     | `IinCTRL`    | `Iin` ≤ `Iin_max`                                   |
| 3     | `IoutCTRL`   | `Iout` ≤ charge current limit                       |
| 4     | `PowerCTRL`  | Power ≤ thermal/`P_max` limit                       |

For the limiters, see [Control loop](../internals/control-loop.md).

## BLE advertising record

The connectionless advertisement (`src/tele/tele_adv.cpp`) carries `Ui`, `Uo`, `I`, `P` (float16), `mcu_temp`,
`ntc_temp` (integer °C, omitted when not fitted), `pwm_duty`, `lag` (capped at 65535 µs), `mppt_state` and
`cv_lim_idx`. `influx_binary_proxy.py --adv` writes them to the same `mppt` measurement, tagged with the
hostname taken from the BLE name.
