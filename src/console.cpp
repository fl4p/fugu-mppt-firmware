
#include <Arduino.h>

#include <esp_pm.h>
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#include <hal/usb_serial_jtag_ll.h>
#endif
#include "util.h"
#include "app_state.h"

#include <esp_private/usb_console.h>

#include <USB.h>
#include <Wire.h>
#include <driver/uart.h>
#include <hal/uart_types.h>
#include "logging.h"
#include "console.h"
#ifdef WITH_USB_MSC
#include "usb/usb_dev.h"
#endif

#include <driver/esp_private/usb_serial_jtag_vfs.h>
//#include "../vfs/private_include/esp_vfs_private.h"

QueueHandle_t uart_queue;

bool handleCommand(const String &inp);

void (*consoleFlushHook)() = nullptr;

void consoleFlush() { if (consoleFlushHook) consoleFlushHook(); }

static constexpr auto uartPortNum = UART_NUM_0;


void loopConsole(int read(char *buf, size_t len), int write(const char *buf, size_t len), time_ms nowMs) {
    constexpr uint8_t bufSiz = 128;
    static char buf[bufSiz];
    static uint8_t buf_pos = 0;

    char chunk[bufSiz];
    int length = read(chunk, sizeof(chunk));
    if (length <= 0) return;

    lastTimeOutUs = wallClockUs(); // stop logging during user input

    // Process byte-by-byte: the raw stream interleaves printable chars, edits and line ends, and a
    // backspace may arrive in a different read() than the char it deletes, so we can't post-process
    // the whole chunk in one pass.
    for (int i = 0; i < length; ++i) {
        char c = chunk[i];

        if (c == '\b' || c == 0x7f) { // backspace (^H) or DEL (terminals/macOS send 0x7f)
            if (buf_pos > 0) {
                --buf_pos;
                write("\b \b", 3); // move back, overwrite with space, move back again
            }
            continue;
        }

        if (c == '\r' || c == '\n') {
            write("\r\n", 2);
            buf[buf_pos] = 0;
            String inp(buf);
            inp.trim();
            if (inp.length() > 0) {
                bool ok = handleCommand(inp);
                char marker[160];
                int n = snprintf(marker, sizeof(marker), "%s: %s\r\n", ok ? "OK" : "ERR", inp.c_str());
                if (n > 0) write(marker, n < (int) sizeof(marker) ? n : (int) sizeof(marker) - 1);
            }
            buf_pos = 0;
            continue;
        }

        if (buf_pos == 0) write("> ", 2); // prompt at the start of each line

        if (buf_pos < bufSiz - 1) {
            buf[buf_pos++] = c;
            write(&c, 1); // echo
        } else {
            buf[buf_pos] = 0;
            ESP_LOGW("main", "discarding command buffer %s", buf);
            buf_pos = 0;
        }
    }
}

int uartRead(char *buf, size_t len) {
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uartPortNum, (size_t *) &length));
    if (length == 0) return 0;
    length = uart_read_bytes(uartPortNum, buf, len, 10);
    return length;
}

int uartWrite(const char *buf, size_t len) {
    return uart_write_bytes(uartPortNum, buf, len);
}

int console_read_usb(char *buf, size_t len) {
#ifdef WITH_USB_MSC
    return usbCdcRead(buf, len);
#elif CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    return esp_vfs_usb_serial_jtag_get_vfs()->read(0, buf, len);
#else
    return 0;
#endif
}

int console_write_usb(const char *buf, size_t len) {
#ifdef WITH_USB_MSC
    return usbCdcWrite(buf, len);
#elif CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    auto r = esp_vfs_usb_serial_jtag_get_vfs()->write(0, buf, len);
    usb_serial_jtag_ll_txfifo_flush();
    return r;
#else
    return 0;
#endif
}

void loopUart(time_ms nowMs) {
    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly
    loopConsole(uartRead, uartWrite, nowMs);

    if (g_app.usbConnected) {
        //loopConsole(esp_usb_console_read_buf, esp_usb_console_write_buf, nowMs);
        loopConsole(console_read_usb, console_write_usb, nowMs);
    }
}


void consoleInit() {
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

void uartInit() {
    uart_port_t port_num = uartPortNum;

    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_APB,
            .flags = {.allow_pd = false, .backup_before_sleep = false },
    };
    int intr_alloc_flags = 0;

// tx=34, rx=33, stack=2048


#if CONFIG_IDF_TARGET_ESP32S3
    //const int PIN_TX = 34, PIN_RX = 33;
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif

    ESP_ERROR_CHECK(uart_set_pin(port_num, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port_num, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 10, &uart_queue, intr_alloc_flags));


/* uart_intr_config_t uart_intr = {
     .intr_enable_mask = (0x1 << 0) | (0x8 << 0),  // UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
     .rx_timeout_thresh = 1,
     .txfifo_empty_intr_thresh = 10,
     .rxfifo_full_thresh = 112,
};
uart_intr_config((uart_port_t) 0, &uart_intr);  // Zero is the UART number for Arduino Serial
*/
}
