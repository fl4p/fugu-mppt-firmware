
#include <Arduino.h>

#include <esp_pm.h>
#include <hal/usb_serial_jtag_ll.h>
#include <sprofiler.h>
#include "version.h"
#include "util.h"

#include <esp_private/usb_console.h>
#include <USB.h>
#include <Wire.h>
#include <driver/uart.h>
#include <hal/uart_types.h>
#include "logging.h"
#include "console.h"

#include "../vfs/private_include/esp_vfs_private.h"

bool handleCommand(const String &inp);


void loopConsole(int read(char *buf, size_t len), int write(const char *buf, size_t len), unsigned long nowMs) {
    constexpr uint8_t bufSiz = 128;
    static char buf[bufSiz];
    static uint8_t buf_pos = 0;

    int length = read(&buf[buf_pos], 128 - buf_pos);
    if (length > 0) {
        if (buf_pos == 0) write("> ", 2);
        if (length + buf_pos >= bufSiz - 1)
            length = bufSiz - 1 - buf_pos;
        write(&buf[buf_pos], length); // echo
        lastTimeOutUs = loopWallClockUs(); // stop logging during user input
        buf_pos += length;
        while (buf_pos > 0 && buf[buf_pos - 1] == '\b') {
            --buf_pos;
            buf[buf_pos] = 0;
        }
        if (buf[buf_pos - 1] == '\r' or buf[buf_pos - 1] == '\n') {
            buf[buf_pos] = 0;
            String inp(buf);
            inp.trim();
            if (inp.length() > 0)
                handleCommand(inp);
            buf_pos = 0;
        } else if (buf_pos == bufSiz - 1) {
            buf[buf_pos] = 0;
            ESP_LOGW("main", "discarding command buffer %s", buf);
            buf_pos = 0;
        }
    }
}

int uartRead(char *buf, size_t len) {
    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    if (length == 0) return 0;
    length = uart_read_bytes(uart_num, buf, len, 0);
    return length;
}

int uartWrite(const char *buf, size_t len) {
    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    return uart_write_bytes(uart_num, buf, len);
}

int console_read_usb(char *buf, size_t len) {
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    return esp_vfs_usb_serial_jtag_get_vfs()->read(0, buf, len);
#else
    return 0;
#endif
}

int console_write_usb(const char *buf, size_t len) {
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    auto r = esp_vfs_usb_serial_jtag_get_vfs()->write(0, buf, len);
    usb_serial_jtag_ll_txfifo_flush();
    return r;
#else
    return 0;
#endif
}

void loopUart(unsigned long nowMs) {
    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly
    loopConsole(uartRead, uartWrite, nowMs);


    if (usbConnected) {
        //loopConsole(esp_usb_console_read_buf, esp_usb_console_write_buf, nowMs);
        loopConsole(console_read_usb, console_write_usb, nowMs);
    }
}