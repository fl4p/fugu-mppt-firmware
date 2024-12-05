find a binary compact protocol to transmit measurements (real-time)
- maybe use compression

https://libre.solar/software/thingset.html CBOR and JSON
https://github.com/ThingSet/thingset-app 
thingset home assistant?
OpenEnergyMonitor?
openWrt
esphome?
    - discovery
    - not a standard outside HA


https://github.com/BojanJurca/Esp32_oscilloscope
https://github.com/easyvolts/espScope (40Msps)

https://github.com/Ebiroll/esp32_sigrok
pulseView

https://www.teuniz.net/edfbrowser/
https://github.com/EUA/ESP32_LogicAnalyzer

ngscopeclient
> libscopehal
>   picoscope adapter exists https://github.com/ngscopeclient/scopehal-pico-bridge
> https://github.com/EEVengers/ThunderScope/blob/master/Software/build_ngscopeclient.sh

https://pypi.org/project/adafruit-ampy/

https://github.com/analogdevicesinc/scopy?tab=readme-ov-file


scopy
    - libIIO (linux embedded)


# encoders (CBOR)
- esp-idf https://github.com/espressif/idf-extra-components/tree/master/cbor/examples/cbor
  - depends on https://github.com/intel/tinycbor/tree/d393c16f3eb30d0c47e6f9d92db62272f0ec4dc7
- thingset uses zcbor
- zcbor supports common data-types and f16. no u12 or i12 (https://github.com/NordicSemiconductor/zcbor/blob/main/include/zcbor_encode.h#L64)
- https://github.com/zserge/jsmn (JSON, not CBOR)

## thingset
- uses zcbor and JSMN
- docs: https://thingset.io/thingset-node-c/
- example:  


# Compression

https://github.com/ESP32-Musings/esp_compression

- https://github.com/atomicobject/heatshrink
  - based on LZSS (LZ77)
- https://components.espressif.com/components/brianpugh/tamp (deflate inspired)
  - evolution of heatshrink?  https://www.esp32.com/viewtopic.php?t=39798
  - esp32 optimizations
- LZ4
  - https://github.com/whitecatboard/Lua-RTOS-ESP32/blob/master/components/openvpn/src/openvpn/comp-lz4.c
- miniz (already in esp-idf)
  - https://github.com/lbernstone/miniz-esp32
  - https://github.com/espressif/esp-idf/blob/master/components/esp_rom/include/miniz.h