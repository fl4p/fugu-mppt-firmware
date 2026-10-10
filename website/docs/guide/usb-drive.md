---
title: USB config drive
sidebar_position: 5
---

# USB config drive

With `CONFIG_FUGU_WITH_USB_MSC` (ESP32-S3 only, off by default), the device appears as a USB drive named
`FUGU_CONF` when it is plugged into a computer. The drive shows the configuration files from `/littlefs/conf/`,
so you can edit them with any text editor. The same USB port also provides the serial
[console](../reference/console.md) as a CDC device.

## Editing the configuration

1. Plug in the device. The drive `FUGU_CONF` and a serial port appear.
2. Edit, create, or delete `*.conf` files in the root of the drive.
3. Eject the drive (Finder *Eject*, Windows *Eject*, `eject` on Linux). On eject, the device checks the changes and
   writes them back. If you unplug without ejecting, the changes are discarded.
4. If anything changed and the converter is idle, the device restarts so the new settings take effect. If it is
   converting, it logs `conf changed, restart to apply` instead. Send `restart` when convenient.

The result of each eject is printed on the console, for example `usb drive: updated /littlefs/conf/charger.conf`.
If the device rejects the changes, it writes nothing, fails the eject command and the console names the problem.
The host may remove the drive anyway: macOS does, and discards the edits. Unplug and replug the device, or send
`restart`, to get the drive back with the device's current files, then redo the edit.

## What is checked

The device applies the changes only if all of them pass the following checks. Otherwise it writes nothing.

- File names must match `[A-Za-z0-9_-]+.conf`. Other files (editor backups, `.DS_Store`, `._*`) are ignored.
- Every line must be blank, a `#` comment, or `key=value` with a key of letters, digits, `_`, `.`, or `-`, in
  printable ASCII, and at most 254 characters long. A file may be at most 16 KiB. A UTF-8 byte-order mark at the
  start is removed.
- `charger.conf` and `limits.conf` must pass the same checks the firmware applies at boot (`conf-check`). Other
  files are only checked for syntax, so a wrong value there shows up at the next boot. The drive and the console come
  up before the configuration is loaded, so a file that stops the firmware from starting can usually be fixed
  over USB.
- The files the firmware needs at boot (`board`, `sensor`, `limits`, `coil`, `converter`, `charger`,
  `tracker`.conf) can be edited but not deleted.
- The drive must be readable as a whole. A damaged directory entry, a broken long file name, or a cluster chain
  shared by two files blocks the commit.

Each file is replaced atomically. A change to several files is applied file by file and stops at the first error.
A power loss or a flash error during an eject can leave some of them updated. The console then reports
`PARTIALLY applied`. Reconnect the drive and check the files.

## Limits

The drive has the following limits:

- The drive is 2 MiB, but the device keeps written data in RAM: up to 96 distinct sectors (48 KiB) may differ from
  the files the drive started with, per session. A sector stays counted once written, even if it is later
  restored. Writing data a sector already holds (for example zeros to an unused sector) costs nothing, so the
  metadata a host creates (`.fseventsd`, `._*` files) usually costs little. Beyond the limit, writes fail, and the
  host reports this as a removed or unavailable medium.
- The root directory holds 64 entries, and a long file name takes extra entries (one per 13 characters).
- Only the root directory of `conf/` is shown, up to 32 KiB of files. Subdirectories created on the drive are
  ignored.
- Edits made over another console (`set-config`, FTP) while the drive is mounted are not merged. On eject, every
  file changed on the drive overwrites the device's copy.
- After an eject the drive stays absent until the device is plugged in again or restarts.

:::warning
The drive shows every configuration file, including `wifi.conf` and `mqtt.conf` with their passwords. Any computer
the device is plugged into can read them.
:::

## USB port

The firmware takes the USB port from the chip's built-in USB-Serial-JTAG controller. This has the following effects:

- USB JTAG debugging is not available.
- The console is a CDC serial port (`fugu_console.py -p <port>` works as before). Log output starts once the USB
  stack is up; earlier boot messages only go to UART0.
- Flashing takes two steps. esptool's reset sequence on the CDC port makes the firmware restart into the ROM
  download mode, which uses USB-Serial-JTAG again. The CDC port disappears, so esptool stops with a serial error.
  Flash again on the port that appeared (on macOS it has a different `/dev/cu.usbmodem*` name). Holding BOOT while
  pressing RESET reaches the same download mode.
