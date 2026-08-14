---
name: nRF52840 USB dongle HCI compatibility for rpi BLE
description: Which nRF52840 USB dongles work as BlueZ HCI adapters: Seeed MDK works (needs hci_usb flash), Ezurio BL654 USB does not (locked to smartBASIC). Also: common non-nRF USB BT dongles that work plug-and-play.
created: 2026-08-10T10:19:37.888Z
metadata:
  node_type: memory
  generator: opencode-claude-memory
  type: reference
  originSessionId: ses_014cdf9d4ffeKqtHkDnVd9FsuO
---

Evaluation of nRF52840 USB dongles as replacement BLE HCI adapters for the rpi (see [[project_rpi_ble_connect_scan_contention]] for why a separate USB adapter is needed).

**Ezurio BL654 USB dongle (451-00003):** Does NOT work as a BlueZ HCI adapter. Datasheet explicitly states "Nordic SDK is not supported on the BL654 USB dongle, part #451-00003" — it's locked to Ezurio's smartBASIC AT-command firmware and cannot be reflashed with hci_usb. The SMD variants (451-00001/00002) can be reflashed via SWD but require a carrier board.

**Seeed nRF52840 MDK USB Dongle (SKU 113990714):** Works, but NOT out of the box. Ships with OpenThread NCP firmware + USB bootloader. Must reflash with Zephyr `hci_usb` sample (or Nordic USB HCI sample) via nRFUtil/nRF Connect before BlueZ will enumerate it as /dev/hciN. Freely programmable — no vendor firmware lock, no JTAG needed. Has integrated chip antenna, +8 dBm, Coded PHY long-range capable.

**Current working adapter:** Espressif HCI USB CDC-ACM, MAC 10:20:BA:05:4C:8E — presents as standard HCI controller to BlueZ out of the box.

**Common non-nRF USB BT dongles (plug-and-play, no firmware flashing needed):**
- TP-Link UB500 — Realtek RTL8761B, BT 5.4, ~€9, in-kernel `btusb` driver, Amazon's Choice with 21K+ reviews. Safe default.
- ASUS USB-BT500 — Realtek RTL8761B, BT 5.0, ~€9.
- StarTech USBA-BLUETOOTH-V5-C2 — Realtek, BT 5.0, ~€15, explicitly lists Linux support.
- CSR8510-based generics — BT 4.0 (no Coded PHY), ~€6, gold standard for Linux, mainline since kernel ~3.x.

**How to apply:** If the rpi's current USB BT adapter fails and a replacement is needed, the cheapest path is a TP-Link UB500 or any CSR8510 dongle — both are plug-and-play. The Seeed nRF52840 MDK dongle also works but requires flashing hci_usb firmware first. Avoid the Ezurio BL654 USB dongle — it cannot serve as an HCI adapter.
