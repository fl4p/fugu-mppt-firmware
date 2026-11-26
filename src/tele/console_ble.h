#pragma once

#include <string>
#include <cstdint>

#include "../util.h"

// BLE serial console over the Nordic UART Service (NUS). A new transport for handleCommand(),
// reachable without Wi-Fi. ESP32-S3 supports BLE only (no classic SPP). All functions are
// no-ops unless the firmware is built with CONFIG_FUGU_WITH_BLE enabled in Kconfig.
//
// security: "none" | "justworks" | "passkey".
//   none      - open, no pairing (any client may issue commands).
//   justworks - encrypted link, no passkey (reliable with Web Bluetooth).
//   passkey   - MITM-authenticated pairing using `passkey` (000000..999999); for native clients.
// With justworks/passkey a new peer can pair only while the pairing window is open; it opens for
// pairWindowS at the first begin (boot) and on blePairOpen(). Bonded peers are not affected.

void bleConsoleBegin(const std::string &deviceName, const std::string &security, uint32_t passkey,
                     uint32_t pairWindowS);

void bleConsoleEnd();

void bleConsoleLoop(time_ms nowMs); // drives the RX queue through loopConsole(); call on core 0

bool bleConsoleConnected();

// Shut the BLE stack and controller down and return their memory to the heap. BLE stays off until
// the next reboot. False (stack kept) if it was never started or a client did not disconnect in 5 s.
bool bleConsoleRelease();
// A BLE OTA push released Wi-Fi and then failed; the caller must reboot.
bool bleOtaRebootDue();
// Free blocks in NimBLE's small mbuf pool (msys_1), the one notifications are built from.
int bleMsys1Free();

size_t bleConsoleChunk();       // usable notify payload: negotiated MTU-3, else 20
bool bleConsoleLinkSettled();   // connected + post-connect settle window elapsed
void bleConsoleResumeAdv();     // (re)start the connectable advertisement (tele_adv reconciler)
const char *bleConsoleName();   // advertised device name ("" before begin)
// Flags + local name (shortened to fit) into a 31-byte advert buffer; returns the length.
size_t bleNameAdv(uint8_t *ad);
// Connectable adv interval (ms, 0 = stack default); false (not applied) before begin or with a client.
bool bleConsoleSetAdvMs(uint16_t ms);
uint16_t bleConsoleAdvMs();

// Pump the TX FIFO until its backlog drops below `lowWater` bytes (or `timeoutMs` elapses / the client
// disconnects), yielding so NimBLE can transmit and free mbufs. A long command that emits more than
// TX_BUF_CAP bytes in one go (e.g. `coredump get`) must call this between writes, else the FIFO
// overflows and the tail is silently dropped. No-op without a connected BLE client (or CONFIG_FUGU_WITH_BLE enabled).
void bleConsoleAwaitTxDrain(unsigned lowWater, unsigned timeoutMs);

bool blePairGated();          // security needs pairing, so the window applies
uint32_t blePairRemainingS(); // 0 = window closed
// `ble` console command: "" (status) | pair [s] | bonds | unpair <addr|all>. nullptr = OK, else error.
const char *bleCommand(const char *sub, const char *arg);
