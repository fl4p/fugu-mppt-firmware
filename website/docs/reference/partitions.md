---
title: Partition layout
sidebar_position: 7
---

# Partition layout

The firmware uses a custom partition table ([`partitions.csv`](https://github.com/fl4p/fugu-mppt-firmware/blob/main/partitions.csv))
sized for **4 MB flash**: two OTA app slots, a config filesystem, a test filesystem and a coredump partition.

## Flash map

Offsets as resolved by `gen_esp32part.py` (partition table at `0x8000`, `CONFIG_PARTITION_TABLE_OFFSET`):

| Name            | Type | Subtype    | Offset     | Size                  | Content                                              |
|-----------------|------|------------|------------|-----------------------|------------------------------------------------------|
| *bootloader*    |      |            | `0x0` (S3), `0x1000` (ESP32) |                       | Second-stage bootloader                              |
| *partition table* |    |            | `0x8000`   | 4 KB                  |                                                      |
| `nvs`           | data | nvs        | `0x9000`   | 16 KB                 | Wi-Fi networks, hostname, other runtime state        |
| `otadata`       | data | ota        | `0xd000`   | 8 KB                  | Active OTA slot selection and rollback state         |
| `phy_init`      | data | phy        | `0xf000`   | 4 KB                  | RF calibration data                                  |
| `ota_0`         | app  | ota_0      | `0x10000`  | 1,871,872 B (1828 KB) | App image, slot 0                                    |
| `ota_1`         | app  | ota_1      | `0x1e0000` | 1,871,872 B (1828 KB) | App image, slot 1                                    |
| `littlefs_test` | data | littlefs   | `0x3a9000` | 64 KB                 | Scratch filesystem for on-target unit tests          |
| `littlefs`      | data | littlefs   | `0x3b9000` | 128 KB                | Board config (`/littlefs/conf/*.conf`) and persisted state |
| `coredump`      | data | coredump   | `0x3d9000` | 64 KB                 | ELF coredump written on panic                        |

The table ends at `0x3e9000`, leaving 92 KB of the 4 MB flash unused. App partitions are 64 KB aligned, so there
is a 28 KB gap between the end of `ota_0` (`0x1d9000`) and `ota_1`.

## OTA slots

There is no `factory` partition. `ota_0` and `ota_1` alternate: an update is written to the inactive slot and
`otadata` switches the boot slot. With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` a new image must confirm itself
once the real-time loop is healthy, otherwise the bootloader returns to the previous slot. See
[OTA over Wi-Fi](../guide/updating/ota-wifi.md).

Each slot holds at most 1,871,872 bytes. A build with BLE is close to this limit; check the app size in the
`idf.py build` output and see [Binary size](../development/binary-size.md).

:::warning
Do not grow the OTA slots without re-checking the 4 MB headroom. Changing any offset moves the partitions after
it, which wipes the on-device config on the next full flash.
:::

## littlefs

`littlefs` holds the board configuration. The top-level `CMakeLists.txt` builds an image from a `config/` folder
and flashes it together with the firmware; `provision.py` writes it separately
(see [Provisioning](../guide/getting-started/provisioning.md)). It is placed after the app slots
([PR #13](https://github.com/fl4p/fugu-mppt-firmware/pull/13)).

`littlefs_test` is formatted and mounted at `/littlefs_test` by the unit tests (`test/test_conf.cpp`,
`test/test_meter.cpp`), isolated from the real config. 64 KB fits a daily energy-ring store (~14 KB) plus littlefs
block overhead.

## coredump

`coredump` receives an ELF coredump on panic (`CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y`), which survives the
reboot and can be read back with the `coredump` console command.

It is appended **after** `littlefs` on purpose: every existing offset stays unchanged, so the OTA slots and the
on-device config of an existing device survive the table change. Inserting it before `littlefs` would shift
`littlefs` and wipe the config.

:::note
The partition table is only written by a serial flash. A device that has only been updated over OTA keeps its old
table (and bootloader) until it is flashed over serial once. Omit the `littlefs` image in that flash to keep the
config.
:::

Decode a dump with the ELF archive, see [Host tools](host-tools.md#elf_archivepy).
