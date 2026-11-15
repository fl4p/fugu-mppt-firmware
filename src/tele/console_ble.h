#pragma once

#include <string>
#include <cstdint>

#include "../util.h"

// BLE serial console over the Nordic UART Service (NUS). A new transport for handleCommand(),
// reachable without Wi-Fi. ESP32-S3 supports BLE only (no classic SPP). All functions are
// no-ops unless the firmware is built with -DWITH_BLE (see main/CMakeLists.txt).
//
// security: "none" | "justworks" | "passkey".
//   none      - open, no pairing (any client may issue commands).
//   justworks - encrypted link, no passkey (reliable with Web Bluetooth).
//   passkey   - MITM-authenticated pairing using `passkey` (000000..999999); for native clients.

void bleConsoleBegin(const std::string &deviceName, const std::string &security, uint32_t passkey);

void bleConsoleEnd();

void bleConsoleLoop(time_ms nowMs); // drives the RX queue through loopConsole(); call on core 0

bool bleConsoleConnected();

size_t bleConsoleChunk();       // usable notify payload: negotiated MTU-3, else 20
bool bleConsoleLinkSettled();   // connected + post-connect settle window elapsed
void bleConsoleResumeAdv();     // (re)start the connectable advertisement (tele_adv reconciler)
const char *bleConsoleName();   // advertised device name ("" before begin)
// Connectable adv interval (ms, 0 = stack default); false (not applied) before begin or with a client.
bool bleConsoleSetAdvMs(uint16_t ms);
uint16_t bleConsoleAdvMs();

// Pump the TX FIFO until its backlog drops below `lowWater` bytes (or `timeoutMs` elapses / the client
// disconnects), yielding so NimBLE can transmit and free mbufs. A long command that emits more than
// TX_BUF_CAP bytes in one go (e.g. `coredump get`) must call this between writes, else the FIFO
// overflows and the tail is silently dropped. No-op without a connected BLE client (or -DWITH_BLE).
void bleConsoleAwaitTxDrain(unsigned lowWater, unsigned timeoutMs);
