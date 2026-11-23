> Original note (February 2026). The maintained version is [Binary size](https://fl4p.github.io/fugu-mppt-firmware/docs/development/binary-size/).


##

```
idf.py size-files

how to find why a specific lib/component is linked, you can add an '#error err' directive to the header file


# locale, wlocale
this got added at some point

# look for header incs
#include <locale.h>
#include <clocale>

- stringstream
    https://github.com/ARMmbed/mbed-os/issues/2311
    
    there is a reference in esp-modem CHANGELOG:
        - Update: Removed stringstream to decrease static sizes ([3ced2d9](https://github.com/espressif/esp-protocols/commit/3ced2d9))



esp32-cbor < espressif/esp_insights <
see depencyies .lock
remote dep:

espressif/arduino-esp32
    espressif/cbor
        


```

* look for headers


