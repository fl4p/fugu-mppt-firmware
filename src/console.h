#pragma once

int console_write_usb(const char *buf, unsigned int len);

void loopUart(unsigned long nowMs);

extern unsigned long lastTimeOutUs;
extern bool usbConnected;



static constexpr int UART_BUF_SIZE = 1024;
extern QueueHandle_t uart_queue;

void uartInit(int port_num);
