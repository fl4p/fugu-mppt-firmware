*this document is an LLM generated placeholder*

# Flash writes and the RT loop (ESP32-S3)

State on branch `flash-autosus`, 2026-11-13. Not yet validated over a full day.

## Result

With flash auto-suspend active and the RT loop resident in internal RAM, a flash erase or program no
longer delays the control loop.

| Device | Config | Worst loop gap during flash ops | Outside flash ops |
|---|---|---|---|
| live converter, INA226 at 1100 us conversion | 445 sps | 2278 us (the loop period itself) | 2275 us |
| live converter, INA226 at 332 us conversion | 1470 sps | 717-734 us over 650 writes | 725-999 us |
| bench S3 (internal ADC only) | 744 sps | 430 us over 200 writes | 429 us |

Before: 40-67 ms with caches off, 151-202 ms with auto-suspend alone.

The loop gap can never be shorter than one sample period. On a board with an INA226 that period is
two conversions, so `sensor.conf::ina22x_conv_time_us` sets the floor (1100 -> 2.2 ms, 332 -> 0.68 ms).
When it changes, scale the `*_filt_len` of the INA226 channels by the same factor.

## Why a flash op stalled the loop

Each cache miss during an erase costs about 260 us: the flash chip is suspended, read, and resumed.
During a page program it is longer. Anything the RT core touches in flash pays that, and so does
anything it waits for.

1. Loop code in flash. Fixed by `main/rt_iram.lf`, generated from an on-device profile.
2. Interrupts on the wrong core. The ADC DMA and I2C interrupts were installed from `setup()` on
   core 0. Core 0 runs Wi-Fi and littlefs from flash, so its interrupt latency during an erase is
   milliseconds. The continuous ADC tolerates about 1.9 ms before its DMA wedges, and only the 250 ms
   no-sample watchdog recovers it. `runOnCore()` (`src/etc/rt.h`) now installs both on the RT core;
   `intr` prints the table.
3. Code the profiler cannot see: inside critical sections (`_i2c_hal_set_bus_timing`, once per I2C
   transaction), interrupt handlers (`__onPinInterrupt`), assembly without symbol sizes
   (`dsps_biquad_f32_ae32`), and vtables. These are in `main/rt_iram_extra.lf`.
4. Kernel lock held across a miss on core 0: `CONFIG_FREERTOS_IN_IRAM`.
5. Log lines. Float formatting runs from flash and took 19-69 ms during an erase, and about 0.4 ms
   at any time. RT-core log calls are now captured raw (format pointer and argument words) and
   formatted by the flush on core 0. A `%s` argument is copied at the call; while a flash op runs
   that copy is skipped and a string outside flash prints as `?`.
6. A notch retune (0.8 ms of soft-float) is skipped while a flash op runs.

## What did not work

`CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY` saves about 14 KB of RAM and Espressif recommends it together with
auto-suspend. With it the BLE controller's level-3 interrupt runs from flash, and the converter reset
with `INT_WDT` during a littlefs erase. Do not enable it.

## RAM

Internal RAM use rose from 165.6 KB to about 192 KB. Paid for by: Wi-Fi IRAM options off (18 KB),
RT task stack 16 -> 8 KB, 8 static Wi-Fi RX buffers. Free heap about 33 KB, minimum 22 KB.

## Workflow

```bash
# 1. profile build: CONFIG_FUGU_WITH_RTPROF=y (FUGU_RT_FLASH=1 keeps the RT path in flash for a first profile)
# 2. on the device, while converting:
rtprof clear        # flash addresses only: what is still in flash on the RT core
rtprof all          # every address, e.g. to see what the core does during a write
rtprof              # dump
# 3. map; the dump carries the ELF sha and the mapper refuses another ELF
etc/rtprof_map.py dump.log build/fugu-firmware.elf --rate 1 --lf main/rt_iram.lf --merge main/rt_iram.lf
# 4. after linking, check that every entry really left flash
etc/rtprof_map.py --check main/rt_iram.lf - build/fugu-firmware.elf
```

Measuring: `reset-lag q`, do the writes, `bootinfo` prints `lag flash/other` (worst loop gap during
and outside flash ops) and the ISR cores. `reset-lag` without `q` also prints the per-section table,
which shows which part of the loop was slow.

Traps met on the way:

- The sampler only sees tasks with interrupts enabled. For the rest, list the IRAM functions' `l32r`
  literals that point into flash (objdump of the IRAM range).
- Inline (COMDAT) functions: the map lists discarded copies first. The mapper skips them, but the
  kept copy can still move between builds, so run `--check`.
- A fragment entry for a toolchain archive (libc, libgcc, libstdc++) needs the whole object.
- A symbol with a `$isra`/`$constprop` suffix cannot be named in a fragment: use `IRAM_ATTR`,
  `always_inline` or `flatten` in the source.
- `idf.py size` rebuilds a stale build dir; use `python -m esp_idf_size <map>` when the ELF must stay.
- Two uncommitted builds have the same version string and `ota_mqtt.py` then skips the push (`-f`).
