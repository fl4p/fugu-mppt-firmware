idf_component_register(
        SRCS
        Adafruit_ADS1X15/Adafruit_ADS1X15.cpp
        INCLUDE_DIRS
        Adafruit_ADS1X15
        REQUIRES
        Adafruit_BusIO
        # arduino-esp32
        #esp_partition # TODO remove, missing in arduino-esp
)

project(Adafruit_ADS1X15)
