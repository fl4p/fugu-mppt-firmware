idf_component_register(
        SRCS
        InfluxDB-Client-for-Arduino/src/Point.cpp
        InfluxDB-Client-for-Arduino/src/util/helpers.cpp
        INCLUDE_DIRS
        "InfluxDB-Client-for-Arduino/src"
        REQUIRES
        #Adafruit_BusIO
        arduino-esp32
        #arduino
        #esp_partition # TODO remove, missing in arduino-esp
)

project(InfluxDB-Client-for-Arduino)
