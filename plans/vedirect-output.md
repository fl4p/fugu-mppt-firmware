*this document is an LLM generated placeholder*

# VE.Direct output: status and next steps (2026-11-24)

Parent plan: `plans/ext-serial-port.md` (§4 status note). Commit `6d40aaac`, not pushed, **untested on HW**.

## Done

- Service `extport` (`src/tele/extport_service.h`), `CONFIG_FUGU_WITH_EXTPORT` (default n). Formatter
  `src/tele/vedirect_text.h`. Meter getters `DailyRingStorage::last()`, `DailyEnergyMeter::days()`.
- `extport.conf`: `mode` (off|vedirect), `tx_pin` (required, no default), `rx_pin` (-1 = no guard), `pid`
  (default 0xA056 = SmartSolar MPPT 100/30, from VE.Direct Protocol 3.34 PID table), `enabled`, `log_level`.
- Text block once a second on UART1, 19200 8N1: PID, FW, SER#, V, I, VPV, PPV, CS, MPPT, OR, ERR, H19–H23, HSDS,
  Checksum. Field mapping: `website/docs/reference/config/extport.md`.
- Guard: any RX byte outside `:`, `0-9`, `A-F`, `\n` latches TX off until `svc rs extport` (`svc list` shows
  `refused: rx 0x..`).
- Docs, config editor (META, FILE_KEYS, TYPE_KEYS, DEFAULTS, SERVICES), CLAUDE.md flag list.

## Verified

- Host test `test/host-stub/vedirect-test.cpp`: layout, checksum over all 256 checksum values, buffer bounds. Passes.
- emonhub's `EmonHubVEDirectInterfacer.input()` state machine parses the output (3/3 blocks, checksum valid).
- Firmware builds with the option on (`build/extport/`); symbols present. Cost ~10.4 KB flash, ~7.5 KB of it the
  UART driver (new to the S3 image, console is USB-JTAG). Slot headroom ~108 KB.

## Loose end

- `etc/config-tool/conf-editor.html`: the `"extport.conf":"extport"` entry in `APPLY` is uncommitted, because the
  `APPLY` table itself is another session's uncommitted edit. Check it went in with their commit.

## Next steps

1. **Bench test** on a v2.3 board via J5: `idf.py app-flash` an image with `CONFIG_FUGU_WITH_EXTPORT=y`, then
   `set-config extport.conf mode vedirect`, `tx_pin 36`, `rx_pin 37`, `svc on extport`. Read with a USB-serial
   adapter (3.3 V) and with emonhub (`com_baud = 19200`; its default 9600 is wrong for VE.Direct).
2. **Guard calibration**: connect a CAN transceiver to J5, confirm the echo trips the guard on the first block. Confirm
   a floating or idle RX does not trip it (internal pull-up is enabled).
3. **Check values** against telemetry: V/I sign while charging, CS transitions (bulk → absorption → float), OR codes
   while stopped, H20/H21 vs `E_today`/`P_max_today`.
4. **Venus OS / GX** (parent plan Phase 0.4): does it accept a text-only device? If not, HEX mode (ping, product ID,
   firmware version, register reads) is the next increment, then DVCC writes through the BMS safety arbiter (§4.3).
5. Open choices: `ERR` mapping (OV → 2, temperature → 17, OC → 18, Vin high → 33); H22/H23 currently report the
   last stored day, not strictly yesterday; `pid` choice per board rating.
