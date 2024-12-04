//
// Created by Fabian on 29/11/2024.
//

#ifndef FUGU_FIRMWARE_CONSOLE_H
#define FUGU_FIRMWARE_CONSOLE_H

int console_write_usb(const char *buf, unsigned int len);

void loopUart(unsigned long nowMs);

extern unsigned long lastTimeOutUs;
extern bool usbConnected;

#endif //FUGU_FIRMWARE_CONSOLE_H
