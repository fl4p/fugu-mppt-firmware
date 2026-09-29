#pragma once

class String; // Arduino String (handleCommand); forward-declared to avoid pulling <Arduino.h>

// Console command layer. setupCli() builds the SimpleCLI command table once at boot;
// handleCommand() parses one whole input line (from UART/USB/telnet/BLE/MQTT — same string
// protocol) through it and returns false on a parse error or handler-rejected input, true
// otherwise. See website/docs/reference/console.md.
void setupCli();
bool handleCommand(const String &inp);

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
// Stamp a wall-clock estimate of the crash time on the first synced boot after a new dump appears.
// Idempotent; safe to call every low-frequency tick. No-op once stamped or when no dump exists.
void coredumpStampIfNew();
#endif
