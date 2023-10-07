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

#set_source_files_properties(
#        ${SOURCES}
#        PROPERTIES COMPILE_FLAGS "-DLFS_CONFIG=lfs_config.h"
#)

#if(CONFIG_LITTLEFS_FCNTL_GET_PATH)
#    target_compile_definitions(${COMPONENT_LIB} PUBLIC -DF_GETPATH=${CONFIG_LITTLEFS_FCNTL_F_GETPATH_VALUE})
#endif()
#endif()

