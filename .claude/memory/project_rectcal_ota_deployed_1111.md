---
name: project_rectcal_ota_deployed_1111
description: "fry+flat OTA'd 2026-11-11 to d574bb6 (rect-cal, flash probe, bootloader-OTA cmd, staged ConfFile); branch integ-ota-1110 NOT merged to main yet"
metadata:
  node_type: memory
  type: project
  originSessionId: 5705d5de-ac35-448c-b5a3-418631e9f63a
  modified: 2026-11-11T14:01:08.565Z
---

2026-11-11 ~13:50 WET: flat then fry OTA'd to `fry-brk1-428-gd574bb60` (branch `integ-ota-1110`, image served from havan 192.168.1.200:9000). Both rollback=VALID, logs clean, ELF archived. `rect_cal_auto=0` was set on both before the OTA; rect-cal has NOT been run on real hardware yet (status: needed=1, offset 0).

bootinfo facts:
- flat: flash jedec 5e4016, probe `supported` tsus 11 µs, erase 61 ms, idf-cap=0; bootloader IDF 5.5.1, May 29.
- fry: jedec 5e4016, tsus 17 µs, erase 54 ms, idf-cap=0; bootloader IDF 5.5.1, May 29.
- idf-cap=0 means IDF's generic chip driver doesn't list suspend, so turning on CONFIG_SPI_FLASH_AUTO_SUSPEND would need a chip-driver change.

fry looped `highD-bflowOpen` mid-sweep hundreds of times per hour on 11-11 from 07:00 on the OLD firmware too. It happens at about 1 W with PV collapsing during the sweep (LS at its minimum), so it is not an LS overrun, and the backstop correctly stays quiet.

**Why:** this is the first field deployment of rect-cal; on 11-11 the integration branch had not been merged into main.
**How to apply:** next steps are a manual `rect-cal` on flat at a steady low-power DCM moment (0.3-4 A), merging integ-ota-1110 into main (the main checkout has other sessions' dirty files plus an untracked idf-export.sh to move aside), and updating the fugu skill's idf-export notes. Related: [[project_flat_mcpwm_sweep_loop_rect_offset]], [[project_flash_write_rt_stall_measured]], [[project_bflow_highduty_trip_accepted]].
