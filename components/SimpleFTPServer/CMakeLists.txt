idf_component_register(
        SRCS
            SimpleFTPServer/FtpServer.cpp
        INCLUDE_DIRS
            SimpleFTPServer
        REQUIRES
            #Adafruit_BusIO
            arduino
            #arduino-esp32
)

#add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX)

project(SimpleFTPServer)

component_compile_definitions(
    DEFAULT_STORAGE_TYPE_ESP32=7
    DEFAULT_FTP_SERVER_NETWORK_TYPE_ESP32=6
)


component_compile_options(-Wno-error=address) # https://github.com/xreef/SimpleFTPServer/issues/71

# add_compile_definitions(STORAGE_TYPE=7 DEFAULT_FTP_SERVER_NETWORK_TYPE_ESP32=6 FTP_SERVER_DEBUG=1)
#set_source_files_properties(
#        ${SOURCES}
#        PROPERTIES COMPILE_FLAGS "-DLFS_CONFIG=lfs_config.h"
#)

#if(CONFIG_LITTLEFS_FCNTL_GET_PATH)
#    target_compile_definitions(${COMPONENT_LIB} PUBLIC -DF_GETPATH=${CONFIG_LITTLEFS_FCNTL_F_GETPATH_VALUE})
#endif()
#endif()

