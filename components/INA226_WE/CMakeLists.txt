idf_component_register(
        SRCS
        INA226_WE/src/INA226_WE.cpp
        INCLUDE_DIRS
        INA226_WE/src
        REQUIRES
        Adafruit_BusIO
        # arduino-esp32
)

project(INA226_WE)
