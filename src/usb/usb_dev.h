#pragma once

// Composite USB device on the ESP32-S3 OTG controller (CONFIG_FUGU_WITH_USB_MSC): a CDC console
// and a FAT drive emulating the conf directory. The drive is built from LittleFS when the host
// enumerates; host edits are validated and written back when the host ejects the drive.

#include <cstddef>

enum class UsbEvent { None, ConfCommitted, EnterDownload };

void usbDevBegin();

// Polled from the network loop; returns and clears the pending event.
UsbEvent usbDevPoll();

// Switches the PHY back to USB-Serial-JTAG and restarts into the ROM download mode.
[[noreturn]] void usbEnterDownload();

bool usbCdcConnected();
int usbCdcRead(char *buf, size_t len);
int usbCdcWrite(const char *buf, size_t len);
