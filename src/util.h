#include <driver/uart.h>

void scan_i2c() {
    const char *TAG = "scan_i2c";
    byte error, address;
    int nDevices;

    ESP_LOGI(TAG, "Scanning I2C...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        /*
            0	success
            1	data too long to fit in transmit buffer
            2	received NACK on transmit of address
            3	received NACK on transmit of data
            4	other error
         */

        if (error == 0) {
            ESP_LOGI(TAG, "Device found at address 0x%02hhX", address);
            nDevices++;
        } else if (error != 2) {
            ESP_LOGW(TAG, "Unknown error %hhu at address 0x%02hhX", error, address);
        }
    }
    if (nDevices == 0)
        ESP_LOGI(TAG, "No I2C devices found");
    else
        ESP_LOGI(TAG, "I2C scan done, %d devices found", nDevices);

    delay(5000);
}


void UART_LOG(const char *fmt, ...) {
    static char UART_LOG_buf[384];

    va_list args;
    va_start(args, fmt);
    vsnprintf(UART_LOG_buf, 380, fmt, args);
    va_end(args);
    auto l = strlen(UART_LOG_buf);
    UART_LOG_buf[l] = '\n';
    UART_LOG_buf[++l] = '\0';
    uart_write_bytes(0, UART_LOG_buf, l);
}