
* esp_log_write writes uses `s_log_print_func`, which defaults to `vprintf`. override with `esp_log_set_vprintf`
* if ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG is enabled, `vprintf` also writes to usb
  * probably in `vfs_console.c / console_write()`
* see `vfs_console.c`
* esp_console_new_repl_usb_serial_jtag
```

UART_LOG(...)
UART_LOG_ASYNC(...)

ESP_LOGX(...)
    uses esp_log_write (which writes to UART0 *AND* USB!)

// set custom log print function:
esp_log_set_vprintf() 


ARDUHAL_ESP_LOG(=n) "Forward ESP_LOGx to Arduino log output"
        This option will redefine the ESP_LOGx macros to Arduino's log_x macros.
        To enable for your application, add the following after your includes:
        #ifdef ARDUINO_ARCH_ESP32
        #include "esp32-hal-log.h"
        #endif
        
        
USE_ESP_IDF_LOG


```

ESP_CONSOLE_USB_CDC_SUPPORT_ETS_PRINTF(=n) "Enable esp_rom_printf / ESP_EARLY_LOG via USB CDC"
