#pragma once

void uartInit(int port_num);

int console_write_usb(const char *buf, unsigned int len);

void loopUart(unsigned long nowMs);

extern unsigned long lastTimeOutUs;
extern bool usbConnected;



static constexpr int UART_BUF_SIZE = 1024;
extern QueueHandle_t uart_queue;



static void consoleInit() {
    Serial.begin(115200);
    //ESP_ERROR_CHECK(esp_usb_console_init()); // using JTAG

#ifndef CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    //usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
    //        .tx_buffer_size = 1024,
    //        .rx_buffer_size = 1024,
    //};
    //ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
#endif

#if CONFIG_IDF_TARGET_ESP32S3 and !CONFIG_ESP_CONSOLE_UART_DEFAULT
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    // see esp-idf vfs_console.c
    uartInit(0);
#endif

}
