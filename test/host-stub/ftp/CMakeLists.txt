# Host-side e2e tests for src/tele/ftp_server.cpp, built natively against POSIX sockets and the
# host file system (../esp_log.h stands in for ESP-IDF logging).

cmake_minimum_required(VERSION 3.16)
project(ftp_e2e CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS ON)

add_executable(ftp_e2e_test
    test_ftp_e2e.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/tele/ftp_server.cpp
)
target_include_directories(ftp_e2e_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_options(ftp_e2e_test PRIVATE -Wall -Wextra -Werror=missing-field-initializers -fsanitize=address,undefined)
target_link_options(ftp_e2e_test PRIVATE -fsanitize=address,undefined)

enable_testing()
add_test(NAME ftp_e2e COMMAND ftp_e2e_test)
