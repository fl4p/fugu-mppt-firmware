---
name: project_coredump
description: Coredump-to-flash partition (needs serial flash to deploy), `coredump get` streaming + raw decode, BLE truncation, SHA-gate bypass, `peek` memory reads
metadata:
  type: project
---

Archived-ELF decode (`elf_archive.py decode`): see CLAUDE.md "ELF archive".

## Coredump-to-flash partition
2026-05-29. `partitions.csv` appends `coredump,data,coredump,,64k` AFTER littlefs (0x3d9000 on 4 MB) so all older offsets stay identical. sdkconfig: `ESP_COREDUMP_ENABLE_TO_FLASH`, `DATA_FORMAT_ELF`, `CHECK_BOOT`, `STACK_SIZE=2304` (1024 left 8 B free and double-faulted while saving; ead28ed).
- A partition-table change cannot be OTA'd: needs one serial flash (bootloader+table+otadata+app, omit littlefs 0x3b9000). flat done 2026-05-29 (full backup `flash-backups/flat-full-20260529-114130.bin`); fry was still pending then.
- `coredump info` on an armed empty partition returns `none (ESP_ERR_INVALID_SIZE)`; "partition not found" means it was never serial-flashed.
- `CHECK_BOOT` boot line is serial-only (early boot not mirrored).
- Test panic: `crash null|abort|stack` (explicit subtype required).

## Retrieval and decode
`coredump [info|get|erase]` (guarded by `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`, not the umbrella flag). `get` base64-streams the partition between `==COREDUMP-BEGIN size=N==` / `==COREDUMP-END==`. Decode: `base64 -d` -> `esp-coredump info_corefile --core-format raw -c dump.bin <elf>` (raw: stream includes the partition header). E2E validated on bench 139C.
- esp-coredump hard-fails on app-SHA mismatch with no skip flag. Fallbacks: `xtensa-esp32s3-elf-addr2line -pfiC -e <elf> <PCs>` (SHA-independent), or for a `-dirty` build of the same commit temporarily neutralise the `core_sha_trimmed != app_sha_trimmed` raise in esp_coredump `corefile/loader.py`, then restore.

## BLE truncation
`get` over BLE used to cut at 5952 B: the loop ran in the same task as `bleTxDrain()`, overflowing the 8 KB TX FIFO. Fix 33ac4e40 + 265f7881 (`bleConsoleAwaitTxDrain(2048, 4000)` per line) — not HW-confirmed over BLE; a >4 s stall still drops the tail. MQTT/telnet stay safest ([[reference_device_access]]).

## peek
2026-05-25. `peek <addr> [len]` reads 1-256 B of RAM/DROM/IRAM/IROM (`strtoul(..,0)` address; len 1/2/4/8 prints a scalar, else hex dump; executable regions need 4-byte aligned addr+len). Resolve symbols host-side (`fugu_console.py` peek/sym), decode structs host-side — use it instead of adding ad-hoc dump commands.
