---
title: Telemetry fields
sidebar_position: 6
---

*this document is an LLM generated placeholder*

# Telemetry fields

The firmware writes one InfluxDB measurement, `mppt`, tagged with the device hostname. This page lists its fields.

## Transports

The same points go to every enabled transport:

| Transport          | Enable                                               | Wire                                                           |
|--------------------|------------------------------------------------------|----------------------------------------------------------------|
| UDP to InfluxDB    | `tele` service, [`tele.conf`](config/tele.md) `influxdb_host`, Wi-Fi up and clock synced (SNTP) | Line protocol to UDP port 8086, batched per datagram; `binary=1` sends the compressed binary wire instead |
| BLE stream         | `CONFIG_FUGU_WITH_BLE_TELE`, `ble` service running, a connected client, `set-time`, `tele.conf` `ble=1`, and either `tele` stopped or `binary=1` (+ `svc rs tele`); then `tele-ble 1` | Binary wire over the NUS TELE characteristic                   |
| BLE advertising    | `CONFIG_FUGU_WITH_BLE_ADV`, `ble` service running, `tele.conf` `adv_ms` | 17-byte record with a subset of fields (see below)             |

The binary wire and BLE records are decoded back into line protocol by `etc/influx_binary_proxy.py`, see
[Host tools](host-tools.md#influx_binary_proxypy).

:::note
Timestamps are epoch **milliseconds**. Configure the InfluxDB UDP listener (or the proxy's `--precision`) for `ms`.
:::

Example line:

```text
mppt,device=<hostname> I=4.213,Ui=38.12,Uo=27.05,P=113.96,E=15230.4,E_today=412.7,pwm_duty=1432i 1727600000000
```

## Tag

| Tag      | Value                                                                  |
|----------|------------------------------------------------------------------------|
| `device` | Device hostname (`hostname` console command; defaults to `fugu-<target>-<MAC>`) |

## Fields

A point is produced at most every 20 ms. Some fields are only included on every n-th point, as listed in the
*Rate* column.

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
| `mcu_temp`    | °C     | float | every 40th    | MCU die temperature                                                             |
| `ntc_temp`    | °C     | float | every 40th    | Board NTC temperature                                                           |
| `lag`         | µs     | int   | every 40th    | Peak RT-loop lag; reset by `reset-lag` and at each periodic sweep                |
| `pwm_ls_duty` | counts | int   | every 10th, converter enabled | Rectifier-switch on-time                                         |
| `pwm_ls_max`  | counts | int   | every 10th, converter enabled | Maximum allowed rectifier on-time                                |
| `pwm_dcm`     |        | bool  | every 10th, converter enabled | Converter operates in discontinuous conduction mode              |
| `Vset`        | V      | float | every 10th, PSU mode | Output voltage setpoint (moves in PV-sim mode)                            |
| `cv_lim_idx`  |        | int   | when a limiter was active | Lowest index of the limiters that were active since the last point, see below |

Source: `MpptController::telemetry()` in [`src/mppt.cpp`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/src/mppt.cpp).

### `mppt_state`

| Value | Mode    | Meaning                                         |
|-------|---------|-------------------------------------------------|
| 0     | `N/A`   | None                                            |
| 1     | `CV`    | Voltage limited (`Vin` min or `Vout` max)        |
| 2     | `CC`    | Current limited (`Iin` or `Iout` max)            |
| 3     | `CP`    | Power limited (thermal derating / `P_max`)      |
| 4     | `MPPT`  | Tracking the maximum power point                |
| 5     | `Sweep` | Global sweep or fade to a target duty           |

### `cv_lim_idx`

| Value | Limiter      | Target                                              |
|-------|--------------|-----------------------------------------------------|
| 0     | `VinCTRL`    | `Vin` ≥ `Vin_min`                                   |
| 1     | `VoutCTRL`   | `Vout` ≤ charger voltage limit (or PSU setpoint)    |
| 2     | `IinCTRL`    | `Iin` ≤ `Iin_max`                                   |
| 3     | `IoutCTRL`   | `Iout` ≤ charge current limit                       |
| 4     | `PowerCTRL`  | Power ≤ thermal/`P_max` limit                       |

See [Control Loop](../internals/control-loop.md).

## BLE advertising record

The connectionless advertisement (`src/tele/tele_adv.cpp`) carries `Ui`, `Uo`, `I`, `P` (float16), `mcu_temp`,
`ntc_temp` (integer °C, omitted when not fitted), `pwm_duty`, `lag` (capped at 65535 µs), `mppt_state` and
`cv_lim_idx`. `influx_binary_proxy.py --adv` writes them to the same `mppt` measurement, tagged with the
hostname taken from the BLE name.
