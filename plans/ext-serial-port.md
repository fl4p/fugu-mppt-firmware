*this document is an LLM generated placeholder*

# External comms port: one VE.Direct-compatible port, CAN and RS-485 via adapters

> **Revision 2 (2026-09-29).** This revision folds in two independent reviews: a Claude agent and a Codex
> (gpt-6-astra) run with network access and a browser. The Codex output is kept at
> `~/codex-reviews/ext-serial-port/final.md`. Revision 1's wrong-adapter guard was fail-open. Its Phase A
> would have fed CAN limits through paths that cannot express "no charge", and it would have reused the
> MQTT stale-data fallback.

> **Scope (2026-09-29).** Hardware now lives in `/Users/fab/dev/ee/hw/Fugu2/plans/ext-comms-port.md`,
> and that plan supersedes the hardware sections below (§3, §5). The firmware sections here are a draft;
> software will be scoped later.

## Goal

One 4-pin port on the next Fugu2 board revision. It carries logic-level TX/RX and is used in one of these
modes:

| Mode | ESP32 peripheral on the pins | What's plugged in |
|---|---|---|
| `vedirect` | UART 19200 8N1 | Victron GX, VE.Direct-USB cable or ESPHome, plugged in directly |
| `modbus` | UART (8E1 by default) | isolated RS-485 adapter (auto-direction) |
| `can` | TWAI | isolated CAN adapter |
| `off` (default) | none; pins are inputs and V+ is off | anything |

The adapters are dumb: a transceiver plus isolation, with no MCU on them. This mirrors Victron's approach,
where the port on the device is not isolated and the cable or adapter provides the isolation.

## Decisions

| | |
|---|---|
| Target | Next Fugu2 PCB revision. No retrofit onto fry, flat or fbuck. |
| Modes, in priority order | CAN BMS input → VE.Direct TX → Modbus RTU slave → CAN multi-charger |
| Connector | 4-pin JST-PH in VE.Direct pinout. Plain Victron cables work. **No ID pin.** |
| Wrong-adapter protection | **In software only.** Fab, 2026-09-28: *"4pin, ignore the safety issue, we will handle this in software properly"*. Both reviews rated a no-ID port a blocker. The accepted residual risk is spelled out in §4.1. |

## 1. Verified facts

**VE.Direct**, from the Protocol rev 3.34 PDF and the VE.Direct FAQ on victronenergy.com:
- The connector is a JST PH, 2.0 mm pitch, 4 pins: 1 = GND, 2 = RX, 3 = TX, 4 = Power +.
- The pinout is the same on producers and consumers, and a producer's TX must reach the consumer's RX.
- The link runs at 19200 baud, 8N1.
- Text-mode blocks end in a `Checksum` field such that all bytes of the block sum to 0 mod 256. Several
  blocks per 1 s cycle are allowed.
- Logic level depends on the product: 5 V on most MPPTs, 3.3 V on the BMV-700 and RS series. Victron's
  panels and interfaces adapt their TX voltage automatically.
- The power pin allows at most 10 mA on average, with bursts up to 20 mA for 5 ms.
- The port on the product itself is not isolated; Victron's USB and RS232 cables provide the isolation.
  The isolated side of those cables is powered from the port.
- On an MPPT, the RX pin can double as a remote on/off input.

**Pylontech low-voltage CAN**, from the v1.2 2018 document (manufacturer PDF hosted in
`maxx-ukoo/jk-bms2pylontech`). The link runs at 500 kbit/s and multi-byte values are little-endian.

| ID | Content |
|---|---|
| `0x351` | Charge voltage limit (CVL): u16, 0.1 V. Charge current limit (CCL) and discharge current limit (DCL): s16, 0.1 A |
| `0x355` | State of charge and state of health: u16, 1 % |
| `0x356` | Pack voltage, current and temperature: s16 at 0.01 V / 0.1 A / 0.1 °C |
| `0x359` | Protection and alarm flags, module count |
| `0x35C` | Bit 7 = charge enable, bit 6 = discharge enable, plus charge-request flags |
| `0x35E` | Manufacturer name |
| `0x305` | Inverter heartbeat, once a second, 8 zero bytes in this edition |

Heartbeat behaviour differs by BMS: REC ignores `0x305`. The `0x373` cell min/max frame appears in the REC
manual, but its encoding and whether JK/Seplos send it are **not verified**.

**ESP32-S3 and ESP-IDF 5.5**, from the installed IDF source and the S3 datasheet v2.2:
- TWAI RX/TX can be routed through the GPIO matrix to any suitable pin (`twai.c:306`).
- The strapping pins are 0, 3, 45 and 46.
- An interrupt is allocated on the core that installs it.
- The TWAI driver is **compiled into the image, not in ROM**, so it costs flash.
- The S3 can send dominant error bits while in listen-only mode unless
  `CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y` is set. It is set today (`sdkconfig:1064`).

**Parts**, from TI datasheets:
- ISO1042: the TXD→RXD loop delay is at most 215 ns including the isolation barrier. A guaranteed TXD high
  needs at least 0.7×VCC1, which is 3.5 V at a 5 V VCC1. The bus side draws up to 73.4 mA while dominant.
  The dominant-timeout is 1.2–3.8 ms.
- THVD1426: the driver actively drives a '1' for only 4–14 µs and then releases the bus. With /RE tied high
  the receiver is off while transmitting; with /RE = 0 the receiver stays on.
- TPS2553: reverse-voltage shutdown trips at 95–190 mV after 3–7 ms. Current flows backwards until then.
- SN74LVC1G17: 5.5 V-tolerant inputs, with partial-power-down protection.

## 2. Phase 0: still to verify

Each item gets a primary source or a measurement. Nothing later in this plan builds on a guess.

1. **VE.Direct cable wiring.** Is the cable straight pin-for-pin, or does it cross 2↔3? Measure a real
   cable.
2. **Pin 4.** What voltage and how much current do a GX and the VE.Direct-USB cable put on pin 4, and what
   do they draw from it? Does their TX level follow Fugu's V+ level?
3. **BMS behaviour, on the target models.** Pick them first (open question). For each, record:
   - Whether it needs `0x305` before it starts sending.
   - Its frame period.
   - Whether it sends `0x373`, and in which encoding.
   - Its behaviour when it gets no ACK.
   - The sign convention of the `0x356` current.
4. **Venus OS.** Does it accept a VE.Direct device that only sends text mode? Test on the rpi. **With DVCC
   enabled**, log any HEX writes (charge voltage and current) that the GX sends to an "MPPT".
5. **Fugu2 schematic and rails.** Answered in §3.0: GND is the battery-terminal negative, there is no 5 V
   rail, and 3.3 V comes from a 0.5 A LM5163. Still open:
   - Draw the full grounding topology: Fugu, GX, USB, chassis, B− and P− across a low-side BMS FET.
   - Measure the 3.3 V rail headroom.
6. **Flash size.** Do a fresh build of the current tree to get the real headroom. The last measured
   figure is 82,992 B free in a 1,871,872 B slot, from an older binary. Record the growth per mode.

## 3. Hardware: Fugu2 revision

### 3.0 What Fugu2 v2.3 already has

Source: the `/Users/fab/dev/ee/hw/Fugu2` netlist, exported with kicad-cli on 2026-09-29.

- **The port already exists.** `J5 "UART (CAN) Conn"` is a 1×4 socket at 2.54 mm pitch:
  1 = +3.3V, 2 = GND, 3 = `CAN_RX` (GPIO37), 4 = `CAN_TX` (GPIO36).
  - The pins go straight to the ESP32, with no series resistor, clamp or buffer.
  - V+ is the unswitched 3.3 V rail.
  - The pin order differs from VE.Direct, which is GND, RX, TX, V+.
  - **The rev keeps GPIO36/37** and replaces J5 with the JST-PH 4 in VE.Direct pinout, adding the
    protection described below.
  - Phase A firmware can be developed on **existing** v2.3 boards through J5 with an isolated CAN
    module. This is bench use only and not a retrofit onto fry/flat.
- **MCU module:** ESP32-S3-WROOM-1-**N4** (4 MB flash, no PSRAM). GPIO35–37 are free only because there
  is no octal PSRAM. For 8 MB, use **N8 or N16 without octal PSRAM (not R8/R16V)**: on octal-PSRAM
  variants, IO35–37 go to the PSRAM. Check this against the WROOM-1 datasheet.
- **Free GPIOs:**
  - Unconnected: 3, 5, 6, 9, 15, 16, 17, 18, 38, 39, 45, 48.
  - Labelled `SPI*` but unrouted: 10–13.
  - Candidates for `tx_oe` and `vplus_en`: 38, 39, 48. For `vplus_sense`, an ADC1 channel: 5, 6 or 9.
    ADC2 conflicts with WiFi.
- **GPIO7 is tied to +3.3V** (U2 pad 7). This looks unintended; check it in the rev.
- **Ground:** logic `GND` is the **battery-terminal negative** (J9.2) and is also bonded to the USB-C
  shell. The 0.5 mΩ shunt R26 sits low-side between `BuckGND` (the solar negative, J10.1) and `GND`.
  - For the BMS FET question, Fugu's GND is on the **P−** side of a low-side BMS FET.
  - A non-isolated link to anything referenced to B− bridges that FET. That rules out a direct CAN
    connection to the BMS; the isolated adapters are required.
  - A GX powered from the same P− bus is fine over a plain VE.Direct cable.
- **Rails:** there is no 5 V rail.
  - +3.3V comes from the external `psu/buck100` board: an LM5163 rated **0.5 A**, fed through a diode-OR
    of Bat+ (D12), Solar+ (Q8/D11) and Vusb (D7), with an SMAJ85A TVS and a 500 mA fuse.
  - +12V for the gate driver comes from an AP3012 boost off 3.3 V.
  - **The adapter's V+ competes on this rail with WiFi TX peaks and the AP3012 gate-drive boost.** The
    "upstream of the MCU regulator" rail in the next paragraph is a 6–85 V node, not a usable supply.
  - **Open:** either measure the 3.3 V load under worst-case switching plus WiFi TX and add about 170 mA,
    or give the port its own small regulator.

### 3.1 New design

**Pins.** Use two free GPIOs, with TX output-capable. Avoid these:

| Pins | Why |
|---|---|
| 0, 3, 45, 46 | strapping |
| 19, 20 | USB |
| 43, 44 | UART0: the ROM boot log would reach the bus on every reset |
| 26–32 | flash |
| 33–37 | octal PSRAM, if fitted |

Also avoid pins with a documented power-up glitch. Validate the reset state on the chosen pins; the S3
doesn't make every pin float at reset.

**Signal path.**
- **RX:** SN74LVC1G17 powered from 3.3 V, with a series resistor and a low-capacitance TVS diode on the
  connector side.
- **TX:** a buffer that tolerates voltage while unpowered, with an output-enable pin (74LVC1G125). The ESP
  pad is never exposed to the connector. This covers three cases: a straight cable to a 5 V producer, a
  5 V pull-up in an adapter, and a Fugu that is powered off.
- The buffer's /OE is pulled to disabled, so TX is high-impedance at reset. Firmware enables it only after
  the guard passes.
- **Level:** 3.3 V push-pull. Victron adapts to that level, but check it on real gear under Phase 0.2.

**V+ (pin 4).**
- Feed it from a rail **upstream of the MCU regulator**, so a hot-plug inrush or a short cannot brown out
  a live converter.
- Use a current-limited load switch with its limit set for **at least 250 mA** (TPS2553 RILIM chosen to
  match). Budget: the ISO1042 bus side at 73.4 mA dominant, through an isolated DC-DC converter, is about
  170 mA at 3.3 V.
- V+ is 3.3 V, so Fugu looks like a 3.3 V product such as the BMV/RS series. Revisit only if Phase 0.2
  says otherwise.
- **Pin 4 connected to another source.** The TPS2553 reverse-voltage protection is not instantaneous, and
  it cannot stop Fugu from feeding a lower-voltage peer. Qualify every source/sink pairing both ways, powered
  and unpowered: Fugu↔GX, Fugu↔USB cable, Fugu↔Victron MPPT. Add an ideal diode if the measurements call
  for one.
- V+ is **on in `vedirect` mode**, because Victron cables power their isolated side from the port. It is
  off in `off` mode and at reset (EN pulled down).
- **Measure V+ current with the ADC.** It helps the guard in §4.1: an isolated adapter draws far more than
  a Victron cable's ≤10 mA.

**Grounding.** In `vedirect` mode, a non-isolated cable is only safe when both devices sit on the same side
of the BMS FET. Otherwise the cable's GND wire bypasses the FET when it opens. Document this rule in the
user docs: use an isolated cable, or put both devices on the same negative. The item 0.5 topology drawing
decides it.

**Adapters must not end up on Victron ports.** A CAN or RS-485 adapter plugged straight into a Victron
product would draw far more than that port's 10 mA allowance. Label the adapters and their cable ends
"Fugu only".

## 4. Firmware

- The feature sits behind `CONFIG_FUGU_WITH_EXTPORT`, default n until it is validated.
- Build prerequisite: `CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y`. Set it in `sdkconfig.defaults` and
  assert it at compile time.
- **Size:** the new board should get an 8 MB flash module and a new partition table. Until then, BLE and
  EXTPORT are mutually exclusive unless the item 0.6 measurement says both fit.
- Do not pull in `esp-modbus`.
- **Conf file `extport.conf`:**
  - Port: `mode` (off | vedirect | modbus | can), `tx_pin`, `rx_pin`, `tx_oe_pin`, `vplus_en_pin`,
    `vplus_sense_ch`.
  - CAN: `can_bitrate` (500000), `can_role` (observer | sole_receiver).
  - Modbus: `modbus_addr`, `modbus_baud`, `modbus_parity` (default E).
  - Keep `website/docs/reference/config/<file>.md` and `etc/config-tool/conf-editor.html` in sync with these keys.
- **`ExtPortService`** (`src/service.h`) runs on core 0 only. Install and free the drivers from a core-0
  task, so their interrupts are allocated there and never touch `loopRT`.
- **Mode switch sequence:**
  1. Disable TX via /OE.
  2. Uninstall the driver.
  3. Re-route the GPIO matrix.
  4. Switch V+ as the new mode needs, and wait for it to settle.
  5. Install the new driver.
  6. Run the guard (§4.1).
  7. Enable /OE.

  Only one peripheral is attached at a time.

### 4.1 Wrong-adapter guard (software, accepted residual risk)

**What the guard cannot do.** It cannot tell adapters apart by listening passively. A quiet CAN adapter, a
quiet RS-485 adapter and a quiet GX all look like an idle line. A 19200-baud UART samples about once every
26 CAN bits, so a 500k CAN frame (110–220 µs) often reads as a clean byte rather than a framing error.

The rules below reduce the risk; they do not remove it. **Residual risk accepted by Fab (2026-09-28):**
- In `vedirect` mode, a CAN adapter on a silent bus gets at most one byte onto the bus before the echo
  check trips.
- An RS-485 adapter wired with /RE high is indistinguishable from a GX, apart from its V+ current.

**Rules:**

1. **`vedirect` mode:**
   - **V+ current signature.** If V+ draws more than a threshold (clearly above Victron's 10 mA; set after
     measuring), assume an adapter and refuse TX.
   - **Per-byte echo check, running continuously**, not only at start-up. A CAN transceiver echoes TXD to
     RXD within 215 ns; a Victron device and a THVD1426 with /RE high do not echo. Compare each byte sent
     with RX. On the first echo, disable /OE and latch "wrong adapter" until the mode changes.
   - **RX content.** Accept only VE.Direct HEX frames or silence. Anything else, such as bytes arriving
     faster than VE.Direct allows or Modbus-looking frames, means refuse TX.
2. **`modbus` mode** is close to safe by construction: a slave transmits only after a valid request with
   the right address and CRC. The V+ current check still applies: if the draw is below the adapter
   threshold, no adapter is present, so refuse TX.
3. **`can` mode:**
   - The V+ draw must be inside the CAN adapter band.
   - Start in listen-only mode, relying on the errata fix above.
   - Leaving listen-only is the moment Fugu starts driving the bus, even without an application
     transmit, because normal mode sends ACKs and error frames. So this is an explicit **role** decision,
     not a heuristic:
     - `observer`: stay in listen-only for good. Another receiver, such as the inverter, provides the
       ACKs. Fugu never transmits.
     - `sole_receiver`: switch to normal mode only after valid Pylontech frames have been seen, then send
       `0x305` if the target BMS needs it (Phase 0.3). If the BMS stays silent until it hears `0x305`,
       sending it first is allowed only in this role.

     Rev 1's "retransmissions mean Fugu is the only receiver" heuristic is **dropped**. CAN frames carry
     no retry marker, and a BMS repeating the same values is normal.
   - **Bus-off:** run `twai_initiate_recovery`, go back to listen-only, and prove the bus again. Back off
     exponentially and cap the attempts. Watch the error-passive, ACK-error and bus-off alerts and report
     them in `status`.
4. **Hot swap.** When the V+ current signature changes during operation, disable /OE and run the mode's
   guard again.
5. Every refusal is logged at WARN and shown in `status`. The verdict is never stored in NVS.

### 4.2 Guard checklist (CLAUDE.md)

1. **Unevaluable input.**
   - `vedirect` stays fail-open on a silent line; this is Fab's accepted residual risk (§4.1).
   - CAN in `observer` role never transmits.
   - Missing or stale mandatory BMS data inhibits charging (§4.3). It does not fall back.
2. **Monotonicity.** Enforced by the safety arbiter (§4.3):
   - More echoed bytes never re-enables TX.
   - A higher cell voltage or a lower CVL/CCL never raises the charge target.
3. **Preconditions.** At runtime, check that:
   - The driver installed successfully.
   - The GPIO routing reads back correctly.
   - The errata Kconfig option is set.
   - The V+ sense channel reads plausibly with V+ off.
4. **Source of truth.** Every BMS field carries its own frame timestamp, and the arbiter reads a coherent
   snapshot.
5. **Persistence.** Guard verdicts and BMS limits are never persisted.
6. **Provenance.** `status` shows:
   - The BMS source: can, mqtt or none.
   - The age of each field.
   - The arbiter state and which limit is currently binding.
7. **Known-bad calibration.** Run each fault on the bench against an independent bus monitor:
   - A CAN adapter on a live bus and on a silent bus, in `vedirect` mode.
   - Pulling the BMS cable in `can` mode.
   - CCL = 0.
   - Charge-enable cleared.
   - A malformed frame and a frame with a short DLC.
   - One missing ID.
   - Timestamp wrap-around.
   - MQTT and CAN updating at the same time.
8. **Fix vs mute.** Not applicable.

### 4.3 Prerequisite: BMS safety arbiter in `charger.h`

The existing BMS paths cannot carry "no charge", and their stale-data handling relaxes the limits instead
of tightening them. The concrete problems:

- **The current floor.** `Iout_max()` never drops below `IOUT_LIM_FLOOR` = 0.25 A (`charger.h:310`,
  `:738`). So CCL = 0 or a cleared charge-enable would still let about 0.25 A through.
- **The stale-data fallback.** When BMS data goes stale (`charger.h:595`), the voltage target glides to
  `Vbat_fallback` without keeping the BMS ceiling. `releaseVoutPinning()` (`:698`) raises the pin to
  `Vbat_max`. A CVL applied through `vpack_pin` would be overwritten.
- **`ibat_lim_topic`.** It writes `params.Ibat_lim` with no expiry (`:664`).
- **Timestamp wrap (a pre-existing bug).** `haveValidCellVoltage()` (`:123`) compares 32-bit microsecond
  timestamps. Data that has been stale for 2³² µs (about 71.6 min) reads as fresh again for 180 s, and this
  repeats every 71.6 min. The same pattern probably affects `ibat_t` and `temp_t`.
- **The termination path.** Termination needs `_bmsCellSource` (set only in `beginMqtt`, `:622`), a fresh
  pack current (`updateBatCurrent`) and temperature (`setTemp`). Feeding only a cell maximum would never
  terminate the charge.

**Design:**

1. **Fix the timestamp wrap first**, as its own commit, since it affects the MQTT path today. Use 64-bit
   stamps with a proper single-writer publish, or a sequence counter. An expired value must stay invalid
   until a genuinely new frame arrives.
2. **Use one source at a time.** Each source keeps its own state. `bms_source` is can, mqtt or auto. In
   auto, choose one source, switch with hysteresis, and never feed the shared pack-current EWMA from both.
3. **Publish a coherent snapshot.** Wrap CVL, CCL, charge-enable, the flags, pack V/I/T, the cell maximum
   and the per-field ages in one struct. Publish it once per frame set through a seqlock or a double buffer
   that is bounded and never blocks. Core 1 only reads it.
4. **Add a final safety arbiter** that runs after all other charger logic. Local termination, MQTT, conf
   writes and pin release may tighten its envelope but never loosen it.
   - **Voltage:** `V_limit = min(vout_max from conf, CVL)`. Apply it to the pack-voltage target and check
     it against the pack voltage in `0x356`, because a raw Vout cap would inherit flat's ~0.35 V
     under-read.
   - **Current:** CCL caps the **pack-current** target through the existing pack-current regulation
     (`_holdIbatTarget` / `_loadFollowStep`, `:437-470`), using pack current from `0x356`, not Iout.
     `0x356` measures total pack current, including any inverter or other charger on the same pack. So
     "pack current ≤ CCL" holds for the whole system. This **replaces rev 1's static `1/N` share**, which
     broke as soon as an inverter was also charging.
   - **Charge permission.** Charging is inhibited outright when any of these hold: charge-enable in
     `0x35C` is 0, CCL ≤ 0, a protection flag in `0x359` is set, or a mandatory field is missing or stale.
     The inhibit is checked before PWM start-up, during sweeps and restarts, and continuously. It does not
     rely on a controller setpoint of zero. It also applies whether or not cell data exists.
   - **Staleness.** In CAN-managed mode, a mandatory field older than about 10 s (set from the measured
     frame period, Phase 0.3) **inhibits charging**. It does not fall back to `Vbat_fallback`. The MQTT
     fallback is not inherited.
   - **Termination.** Use the cell maximum from `0x373` when present, plus pack current, temperature and
     `_bmsCellSource`. With no cell data, leave termination to CVL and the charge-enable flag.

**Latency caveat.** Pack current arrives from the BMS at about 1 Hz. After a sudden load drop on the bus,
pack current can briefly exceed CCL until the next frame arrives. Check whether the BMS's CCL leaves margin
for that; otherwise rate-limit the charge target when frames are sparse.

### Phase A: CAN BMS input (priority 1)

- Parse the IDs in §1, with per-ID checks on DLC and range. Frames feed the §4.3 snapshot.
- **Tests:**
  - Unit tests for frame decoding and the arbiter, on the host stub or Unity. Cover every §4.2.7 case,
    including repeated timestamp wraps.
- **Bench:**
  - Use an S3 bench board with an **isolated** CAN module; a bare SN65HVD230 creates a ground loop and
    doesn't test the real path.
  - Test against the target BMS, with an independent CAN monitor on the bus.
  - Measure the guard's CPU cost and core-1 loop latency at full bus load and during flash writes.

### Phase B: VE.Direct TX (priority 2)

- Once per second, send text-mode blocks with the SmartSolar field set: PID, FW, SER#, V, I, VPV, PPV, CS,
  MPPT, ERR, H19–H23, HSDS and Checksum. Build them with `snprintf`. H19–H23 need persistent yield counters.
- **PID:** a conf value. Do not advertise a product ID whose control features (DVCC limit writes) Fugu
  does not implement. Either implement those writes, or accept that the GX will believe it is enforcing
  limits it isn't. Settle this with Phase 0.4.
- Add HEX-mode replies only if Phase 0.4 shows they are required.
- **Optional:** a VE.Direct consumer mode that reads a SmartShunt/BMV as the pack-current source for the
  arbiter.
- **Test:** Venus OS on the rpi with DVCC on and off, and the ESPHome `victron-vedirect` component.

### Phase C: Modbus RTU slave (priority 3)

- Hand-rolled: function codes 03/04/06/16, CRC16.
- **Framing** (Modbus over serial v1.02):
  - Default 8E1; with no parity, use two stop bits. The framing is independent of VE.Direct's 8N1.
  - Detect both the 1.5-character invalid-frame gap and the 3.5-character end of frame. Above 19200 baud,
    use the fixed 750 µs / 1.75 ms timers.
  - Handle broadcast (address 0 gets no reply).
  - Handle UART overflow and errors.
- **Echo policy** depends on how the adapter wires /RE. Fix it in the adapter design.
- **Response latency:** don't serve requests from the network-loop tick. WiFi or OTA stalls could exceed
  the master's timeout. Use a small core-0 task, or a large RX queue with timestamps.
- **Writes:** `set-config` performs no range validation (`cli.cpp:1437`, `conf.h:117`), so Modbus writes
  go through **typed setters** that check range, finiteness and cross-field consistency. Multi-register
  writes are applied atomically. Runtime setpoints are kept apart from persistent config. No write can move
  the §4.3 envelope.
- **Register map:** open question.
- **Test:** `pymodbus` through a USB-RS485 dongle and the adapter.

### Phase D: CAN multi-charger (priority 4)

- Fugu units publish their power, Iout, state and termination flag on IDs checked against the Pylontech,
  SMA, Victron VE.Can and common inverter ID ranges. This needs the `sole_receiver` role, or a new
  `participant` role with the same guard.
- The §4.3 arbiter already keeps pack current at or below CCL system-wide. Phase D improves how the chargers
  **share** that current, and addresses the voutAuthority asymmetry on the shared bus.
- A share left by a peer that has gone quiet is **not** handed to other units until that peer is known to
  have stopped. A lost message does not prove the peer's output is zero.

## 5. Adapters (separate small PCBs)

Both adapters use a 4-pin JST-PH on the Fugu side and are labelled "Fugu only".

| | CAN adapter | RS-485 adapter |
|---|---|---|
| Isolation and power | isolated 5 V → ISO1042 | isolated 5 V → 2-channel isolator → THVD1426 |
| Logic side | VCC1 = 3.3 V from V+, TXD pulled up to 3.3 V (at VCC1 = 5 V the 3.5 V TX-high threshold would fail) | 3.3 V |
| Idle state | TXD pull-up | pull-up on the isolator input carrying R (R goes high-impedance while D is low); isolator defaults high |
| Termination and bias | 120 Ω split termination on a DIP switch, **off by default** because the bus is usually already terminated | 120 Ω termination and fail-safe bias on DIP switches. Bias is effectively required: the driver releases the bus 4–14 µs after a '1' |
| Bus side | GND2 referenced to the remote bus ground | — |
| Connectors | RJ45 plus screw terminal | RJ45 plus screw terminal |

- The RS-485 adapter decides /RE wiring and therefore the echo policy.
- The CAN adapter's RJ45 pinout must be checked against each target BMS/inverter; the screw terminals
  sidestep the question.
- **Protection and timing:** specify the ESD parts, clamp voltages and return paths. State the maximum
  length of the logic-level cable (≤1 m) and of the CAN bus, and the allowed stubs.
- **Timing budget:** at 500 kbit/s a bit is 2 µs, and the IDF preset samples at 75 % = 1.5 µs. The ISO1042
  adds ≤215 ns. That leaves margin, but it is no guarantee on cable length. **Qualify the real path**:
  GPIO matrix, buffers, protection, cable, isolator and bus.

Design these with the `kicad-design` skill.

## 6. Order of work

1. **Phase 0** verification (§2), and fix the pre-existing timestamp wrap (§4.3.1).
2. **§4.3 safety arbiter and snapshot**, with unit tests, before any CAN code.
3. **Electrical interface and adapter prototypes**, qualified on the bench over the real signal path
   (§3, §5), together with the **Phase A** firmware and the §4.1 guard.
4. **Freeze the Fugu2 PCB.** Adapter rails, TX buffering, V+ sensing and cable timing all feed into the
   host connector, so this comes after step 3.
5. **Phase B**, then **Phase C**.
6. **Phase D.**
7. Only after bench validation, run a real converter. Confirm with Fab before flashing fry or flat.

## Open questions

- How should V+ be powered: from the 3.3 V rail, which needs a headroom measurement, or from its own
  regulator? (Fugu2 is at `/Users/fab/dev/ee/hw/Fugu2`; see §3.0.)
- Which BMS models must Phase A support first?
- Which Modbus register map: custom, SunSpec, or EPEver-compatible?
- Should Phase D run on the BMS↔inverter bus, or on a separate bus for Fugu units only?
- Phase B: implement DVCC limit writes, or report a PID that doesn't imply them?
