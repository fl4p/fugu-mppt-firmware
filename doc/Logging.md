
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