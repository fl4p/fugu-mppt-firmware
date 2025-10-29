idf_component_register(
        SRCS
        ESPTelnet/src/ESPTelnet.cpp
        ESPTelnet/src/ESPTelnetBase.cpp
        INCLUDE_DIRS
            ESPTelnet/src
        REQUIRES
            #arduino
            # Adafruit_BusIO
            arduino-esp32
)

project(ESPTelnet)
