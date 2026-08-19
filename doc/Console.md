# Serial Console

Send text commands over UART (or telnet, USB serial-JTAG, or MQTT) to interact with the charger
while it is running. The same string protocol is used on every transport, which also makes it
suitable for automated tests. Input and output are multiplexed across UART, USB serial-JTAG and
telnet.

Default UART baud rate is 115200. Terminate each command with `\n` or `\r` (new line).
A successfully handled command is confirmed with:

```
OK: <cmd>
```

An unknown, malformed, or out-of-context command is confirmed with `ERR: <cmd>`
and a logged reason: a parser error for an unknown command, or the specific
rejection message for invalid arguments / wrong context.

# System Commands

| Command | Description |
| --- | --- |
| `wifi on`, `wifi off [minutes]` | Enable / disable Wi-Fi (and with it all network services). Disabling Wi-Fi usually increases the control-loop rate. Bare `wifi off` disables for good and clears the stored SSID in NVS — it persists across reboots (NVS `wifi_off`) until `wifi on`; `wifi off <minutes>` disables temporarily (RAM only, a reboot re-enables Wi-Fi) and re-enables after the timeout, keeping the stored SSID. |
| `wifi add <ssid>:<password>` | Store a new Wi-Fi network. |
| `ip` | Show the local IP address. |
| `hostname <hostname>` | Set the device hostname (persisted in NVS, applied on next boot). |
| `ota <url>` | Download and flash a new app image from an HTTP(S) URL. Halts the converter and ADC during the update. |
| `set-time <epoch_ms>` | Set the wall clock without NTP (epoch milliseconds; sets TZ to CET). For BLE-only telemetry; SNTP still runs when WiFi comes up and may step the clock. |
| `tele-ble [0\|1]` *(needs `CONFIG_FUGU_WITH_BLE_TELE`)* | Start/stop the BLE telemetry stream on the NUS TELE characteristic (binary wire, tamp-compressed). Requires a set clock (`set-time`) and a connected BLE client; no argument prints status + dropped bytes. Host side: `etc/influx_binary_proxy.py --ble` or `etc/fugu_console.py --ble --tele`. |
| `curl [-X M] [-H k:v] [-d data] <url>` *(needs `CONFIG_FUGU_WITH_NETTOOLS`)* | Blocking HTTP(S) request — prints the status line and response body to the issuing console. TLS is verified against the mbedTLS certificate bundle. `-X` sets the method (GET/POST/PUT/DELETE/HEAD/PATCH); `-d` sends a request body (implies POST, defaults Content-Type to form-encoded unless a `-H Content-Type:…` is given); `-H` adds a header (up to 4, `key:value`). Flag values are single tokens — no spaces, so use compact JSON. The body is streamed and capped at 16 KB. |
| `ping <host> [count]` *(needs `CONFIG_FUGU_WITH_NETTOOLS`)* | ICMP echo to an IPv4 host/IP (`count` 1–60, default 4). Prints per-reply lines (seq/ttl/time) and a sent/received/loss summary. |
| `nslookup <host>` (alias `resolve`) *(needs `CONFIG_FUGU_WITH_NETTOOLS`)* | Print every IPv4 address the resolver returns for `<host>`. |
| `tcpconnect <host> <port>` (alias `probe`) *(needs `CONFIG_FUGU_WITH_NETTOOLS`)* | Non-blocking TCP connect with a 5 s timeout; reports `open` / `refused` / `timeout` / `error` and the elapsed time. Port-level reachability check for a broker or OTA endpoint. |
| `netstat` (alias `ifconfig`) *(needs `CONFIG_FUGU_WITH_NETTOOLS`)* | STA link + IP config snapshot: ssid/bssid/channel/rssi, ip/gateway/netmask/dns, and MAC. |
| `restart` (aliases `reset`, `reboot`) | Reset the MCU. |

# Control & Diagnostics

| Command | Description |
| --- | --- |
| `fan <float>` | Set fan speed, 0–100. |
| `led <RRGGBB>`, `led <RGB>` | Set the LED color in hex or short hex (e.g. `led 33ff33` or `led 3f3`). |
| `sensor` | Dump per-sensor state (last/raw value, EWM average and std, adaptive-noise-filter stats). `sensor avg` prints one compact line of EWM averages (`sens: vin=… iout=… …`) for fast polling. The ANF line reads `(stale)` unless `anf on` is active (the filter is kept off the RT path by default). |
| `anf [on\|off]` | Enable/disable the per-sample AdaptiveNoiseFilter, which feeds the noise/NSR stats in `sensor`. It's diagnostics-only and off by default to keep it out of the RT sample path; turn it on while inspecting sensor noise, off when done. |
| `mem` | Display heap and PSRAM size (total and free). |
| `heap [check]` | Per-capability heap report (INTERNAL / DMA / SPIRAM): free, minimum-ever-free, and largest free block (bytes) — fragmentation at a glance. `heap check` additionally runs `heap_caps_check_integrity_all` and prints `OK`/`CORRUPT`. |
| `tasks` | FreeRTOS task table: name, state (run/rdy/blk/sus), priority, pinned core (0/1/any), and **minimum-ever free stack in bytes** (`uxTaskGetStackHighWaterMark`) — the value to watch for stack overflows. Complements `rt-stats` (which shows CPU %, not headroom). |
| `bootinfo` | Last reset reason (`POWERON`/`PANIC`/`TASK_WDT`/`BROWNOUT`/…), the running OTA slot with its rollback verify state (`PENDING_VERIFY`/`VALID`/…), heap free + min-ever + largest block, and uptime/app version. First stop for "why did it reboot / did the OTA confirm?". |
| `log <tag> <level>` | Set the runtime `ESP_LOG` level for any tag at `error`/`warn`/`info`/`debug`/`verbose`/`none` (`*` = all tags). Generalises `svc log` (which only covers services); e.g. `log wifi debug`. Levels above the compile-time max are silently capped. |
| `ls [path]` | List a littlefs directory (default `/littlefs`) with sizes, or stat a single file. A relative path is taken under `/littlefs/` (e.g. `ls conf`). |
| `cat <file>` | Print a littlefs text file (relative paths under `/littlefs/`, e.g. `cat conf/board.conf`); capped at 16 KB. |
| `peek <addr> [len]` | Read memory at `<addr>` (hex `0x…`, decimal, or octal). With `len ∈ {1,2,4,8}` (default 4) prints one typed hex value (`peek 0x… = 0x…`); other `len` ≤ 256 prints a hex+ASCII dump. Refuses addresses outside internal RAM / DROM / external RAM / RTC slow+fast RAM / IRAM/IROM / peripheral MMIO (the last two need 4-byte aligned `addr` and `len` for word-bus reads). MMIO is unguarded: a register whose peripheral clock is gated faults the bus, and several registers are read-destructive (UART/I2C FIFO pop, MCPWM capture, `*_INT_ST` latches). The host CLI (`etc/fugu_console.py`) accepts `peek <symbol>[.field…][+offset]` and ships `sym <pattern>` + `peek-struct <symbol>[.field…]` — all resolved client-side against the build ELF (DWARF for member offsets / field decoding), so the device only ever sees a numeric address. |
| `peek-struct <obj>[.field…] [depth]` *(host-only)* | DWARF-typed dump of an object or sub-object: enumerates each member (offset, type, name) and decodes its value (int / float / bool / pointer / enum / char[]). Reads the byte image via chunked `peek` calls. Embedded aggregates expand inline up to `[depth]` levels (default 2, range 0..16); past the budget they print as `<TypeName, N B>` and can be drilled into with a longer dotted path. Static `constexpr` class members are skipped (no storage). |
| `uptime` | Print seconds since boot (monotonic; resets only on reboot) and the running app description (name, version, build date/time, IDF version). |
| `rt-stats` | Print FreeRTOS per-task runtime statistics (sampled over ~2 s), sorted busiest-first, with each task's pinned core and **per-core CPU %** (a core-pinned task reads 0–100 % of its own core; read saturation off the `IDLEx` row, not the busy task). |
| `reset-lag` | Reset the max-lag statistic and print [rtcount](Real-time%20Counter.md) timings. |
| `scan-i2c` | Run an I²C bus scan. |
| `adc-restart` | Re-initialize the ADC backends. |
| `adc-reset` | Reset the ADC peripherals. |
| `coredump [info\|get\|erase]` *(needs `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`)* | Inspect or extract the panic core dump saved to the `coredump` flash partition. `info` (default) reports presence, size, integrity (`check=ok`) and `crashed=<epoch>` — a wall-clock estimate of the crash time, stamped into NVS on the first time-synced boot after a new dump (keyed by the dump's checksum; `0` = unknown/not yet stamped). `get` streams the raw partition image as base64 between `==COREDUMP-BEGIN==`/`==COREDUMP-END==` markers (mirrors to telnet/MQTT/BLE, so a backtrace can be pulled with no serial); `erase` clears the dump and its NVS stamp. Pull + decode host-side with `etc/fugu_console.py --coredump get` (writes `coredump.bin`), then symbolicate against the **exact** build's ELF — `etc/idf-devtools/elf_archive.py decode --device <name> coredump.bin` auto-matches by the dump's embedded app-SHA from the flash-time ELF archive (the firmware sets `CONFIG_APP_RETRIEVE_LEN_ELF_SHA=64` so the full hash is stored and the match is unambiguous; a short/zero value would match any ELF and yield a bogus backtrace). SHA-independent fallback: `xtensa-esp32s3-elf-addr2line -e <that>.elf <backtrace PCs>`. |
| `crash <null\|abort\|stack>` *(needs `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`)* | **Deliberately panic** the device to exercise the coredump path: `null` (write to 0x0 → StoreProhibited), `abort` (`abort()`), `stack` (unbounded recursion → stack overflow). The explicit subtype is required (the console is reachable over MQTT). Bench/test only. |

# Config Commands

Hardware and runtime parameters live in `.conf` files on the device's littlefs partition under
`/littlefs/conf/`. These commands edit them in place without re-flashing.

| Command | Description |
| --- | --- |
| `set-config <file> <key> <value>` | Set a key in a config file and persist it to flash. |
| `del-config <file> <key>` | Remove a key; the whole line, including any inline comment, is deleted. |
| `get-config <file> [<key>]` | Print a single key, or dump every key if `<key>` is omitted. |
| `conf-check` | Re-read `charger.conf`/`limits.conf` and warn about keys no loader reads (typos / obsolete, e.g. `cv_min` where the firmware reads `cv_float`). Same check runs at boot for the parameter confs. |

Examples:

```
set-config coil.conf L0 50
set-config limits.conf iout_max 35
set-config converter.conf vout_max 28.5
set-config mqtt.conf broker_uri mqtt://192.168.1.134:1882
set-config charger.conf cell_voltage_eoc 3.53
set-config sensor.conf vout_filt_len 10

del-config sensor.conf vout_filt_len

get-config mqtt.conf broker_uri
get-config converter.conf

conf-check
```

# Charger Commands

| Command | Description |
| --- | --- |
| `status` | Print a charger/battery snapshot: termination state, effective limits (`Vbat_max`/`Vout_max`, `Ibat_lim`/`Iout_max`), the termination line (`v_term`/`cv_min`/`cv_eoc`/`Cbat`/`recharge_dod`) and the BMS feed (`vcell_high` with staleness, `ibat`, `ahSinceFull`, `vout_avg`). In PSU mode also prints the setpoint, trip count and escalation state. |
| `vset <float>` | Set the battery max voltage (`Vbat_max`), range 0–999. Marks the setpoint as explicit so the persistent-OV auto-detect will not silently discard it. |
| `iset <float>` | Set the battery current limit (`Ibat_lim`), range 0–999. |
| `ovset <float>` | Set an independent hard output over-voltage trip limit, range 0–999. 0 clears it (reverts to the derived threshold: `Vbat_max` × 1.5, or × 1.03 with `reverse_current_paranoia`). When set, the OV threshold is `min(ovset, vout_max)` regardless of the CV setpoint. |
| `psu <float>` | Enter PSU (constant-voltage) mode and set the output voltage setpoint. The limiter chain regulates Vout to the setpoint with CV/CC foldback — no MPPT tracker, no periodic sweep, no charger-layer battery semantics. Trips use a fast 100 ms auto-retry with escalation to a hard latch after repeated faults. Range-checks against `limits.conf::vout_max`. |
| `psu off` | Exit PSU mode, return to MPPT tracking. |
| `psu` | Print PSU mode state: setpoint, trip count, escalation/latch state. |
| `pv <isc> <voc> [k]` | Enter PV-sim (solar-array-simulator) mode: the output follows the panel curve V=f(Iout) with `Voc` at no load and MPP at `k·Voc` (k default 0.8, range [0.5,0.95]). Runs on the PSU machinery (same trips/latch); the setpoint moves along the curve, slew-limited, clamped to [Vin+0.5, min(Voc, `vout_max`)] — a boost can only emulate the curve above Vin. The Iout limiter is capped at 1.1·Isc; **below the Vin floor the body diode passes current firmware cannot limit** — keep the input supply's current limit low. Re-issuing while active updates the curve in place (no setpoint jump) and clears a trip latch/backoff — the escape hatch, like `psu <V>`. |
| `pv scale <s>` | Irradiance knob: re-apply the curve with `Isc = s ×` the last full `pv` command's Isc, s in (0,1.2]. Scales don't compound, and a scale does **not** clear a trip latch or fault backoff. |
| `pv off` | Ramp to 0 duty and enter manual mode (deliberately not the MPPT fallback of `psu off` — this is a bench source). |
| `pv` | Print PV-sim state: curve params, live setpoint/Vout/Iout, trip state. |

These override the running charger parameters only; use `set-config charger.conf …` to persist.

# Manual PWM Commands

| Command | Description |
| --- | --- |
| `dc <int>` | Set the converter duty cycle directly and switch the charger to manual PWM mode (no tracking, protection still active). A non-zero duty enables sync rectification and the backflow switch unless `reverse_current_paranoia` is set. |
| `+<int>`, `-<int>` | Relative duty-cycle perturbation step. Available both in manual and tracking mode (to test tracker recovery). **Be careful with large positive jumps** — they can cause extreme current transients that destroy the switches. |
| `mppt` | Switch back to MPP tracking mode (only valid while in manual PWM or PSU mode). |
| `sweep` | Start a global MPP scan / search. Exits PSU mode if active. |
| `speed <float>` | Set tracking speed scale, range 0–10 (default 1.0). |
| `dt [ns]`, `deadtime [ns]` | MCPWM hardware dead-time. Without an argument it prints the configured value, the realized gap, and the limits derived from it (`pwmMax`, `minLS`, `maxHS`). With an argument it retunes the dead-time submodule from the RT core, quantized to the timer tick (6.25 ns at 39 kHz). HiLi + `pwm_driver=mcpwm` only — an InEn gate driver does its own dead-time. **RAM only**: persist with `set-config board.conf pwm_deadtime_ns <ns>` and reboot. The new delay and the duty clamp both latch on a period boundary (`update_dead_time_on_tez`) — usually the same one, and when a boundary falls between the two writes the intervening period runs the new dead-time against the old comparators, which only widens the gaps. Raising it is allowed in any mode, but note it is an *unramped* duty step — at the ¹⁄₃₂ ceiling `maxHS` drops by ~190 counts in one control tick; **lowering it requires manual PWM** (`dc N` first). Keep a scope on the switch node either way. |
|  | Refusals, all of them shoot-through guards: **retuning only** — if the board booted with `pwm_deadtime_ns=0` the submodule is bypassed, and arming it live would put the LS pin on the HS waveform between the two register writes, so `dt` refuses and you must set the conf key and reboot. **≥ 50 ns realized gap** — the HS generator spends one tick claiming its dead-time path, so the realized gap is one tick less than the configured value (1 tick would close it entirely). The floor is a typo guard, not a tuned value: it is far below any dead-time we ship, and a gap that clears it is still not necessarily safe for a given FET and gate driver — that is yours to verify on the bench. **≤ ¹⁄₃₂ period** (800 ns at 39 kHz) — the band is reserved out of `pwmMax` *and* pushes `minLS` up by the same amount, since it delays LS turn-on and the bootstrap-refresh pulse has to survive it. |
| `measure-coil l0\|ls [steps\|hs] [dwell_ms] [apply]` | Measure the coil on-device by driving a DCM sweep (takes over manual PWM, restores MPPT when done). `l0` sweeps duty and reports the inductance (median over the DCM band); `ls` holds HS and sweeps the low-side count to find the `rect_offset_ns` timing. `apply` writes the result to `coil.conf`. Needs `Vin > Vout` (sun/headroom). Port of `etc/measure_coil.py`; see [Coil Inductance Measurement](Coil%20Inductance%20Measurement.md). |

The following commands require manual PWM mode:

| Command | Description |
| --- | --- |
| `sync [on\|1\|off\|0\|forced]` | Disable/enable the low-side switch (diode emulation / synchronous rectification). `forced` puts the converter into forced-PWM mode and disables the various reverse-current checks. |
| `bf <0\|1>`, `panel <0\|1>` | Disable/enable the backflow (panel) switch. When enabled it allows current to flow from output to input (battery to solar). Requires a configured backflow switch. |
| `short-ls` | Short the low-side switch. Only valid in boost topology with `Vin` near zero (e.g. for a controlled output discharge). |

# Service Commands

The optional non-RT subsystems (`mqtt`, telemetry, `ftp`, `telnet`, `lcd`, `scope`) are managed as
services. Each has its own state, log level, and `enabled` flag persisted in its conf file.

| Command | Description |
| --- | --- |
| `svc` / `svc list` | List all services with state, log level and enabled flag. |
| `svc on <name>` | Enable (persist) and start a service. |
| `svc off <name>` | Disable (persist) and stop a service. |
| `svc restart <name>` / `svc rs <name>` | Restart a service (stop then start); re-reads its conf. |
| `svc log <name> <error\|warn\|info>` | Set and persist a service's log level. |

# Scripts

Console scripts are plain text files stored on the device's littlefs partition under
`/littlefs/scripts/`. Each line is a console command (same syntax as typing it). Lines starting
with `#` are comments; blank lines are skipped. Upload via FTP (to `/littlefs/scripts/`) or
create them with `script-set` below. View with `cat scripts/<name>`.

| Command | Description |
| --- | --- |
| `run <name>` | Execute a stored script. Resolves `/littlefs/scripts/<name>` then `<name>.txt`. Each command's output is prefixed with `=== <cmd> ===`. Stops on the first error and aborts. Nesting up to 3 levels deep. |
| `sleep <seconds>` | Delay 0–60 seconds (fractional OK, e.g. `sleep 0.5`). Blocks the console task but not the RT control loop. Mainly for use inside scripts to wait between commands. |
| `scripts` | List all scripts in `/littlefs/scripts/`. |
| `script-set <name> <cmd; cmd; …>` (alias `script`) | Create or overwrite a script from a single line. Commands are separated by `;`. The file is written to `/littlefs/scripts/<name>.txt`. Overwrites if the file exists. |

Examples:

```
script-set setup_rig set-config board.conf mcu esp32s3; set-config coil.conf L0 50; sleep 2; conf-check
run setup_rig
scripts
cat scripts/setup_rig.txt
```

Limitations: `;` is the command separator and cannot appear in command arguments — use FTP for
scripts that need `;` in values (e.g. URLs). Line length is limited to ~199 characters. `;;` in
a script acts as SimpleCLI's line delimiter and will execute as separate commands.

# Telnet

Use any telnet client to connect on port 23. No password is required. Only one connection at a time.

Connect from Home Assistant:

* install the "Terminal & SSH" add-on
* in the add-on Configuration, add `busybox-extras` to Packages
