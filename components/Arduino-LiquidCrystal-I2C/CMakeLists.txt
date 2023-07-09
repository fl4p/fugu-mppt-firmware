idf_component_register(
        SRCS
        Arduino-LiquidCrystal-I2C-library/LiquidCrystal_I2C.cpp
        INCLUDE_DIRS
        Arduino-LiquidCrystal-I2C-library
        REQUIRES
        arduino
        #arduino-esp32
        #esp_partition # TODO remove, missing in arduino-esp
)

project(Arduino-LiquidCrystal-I2C)