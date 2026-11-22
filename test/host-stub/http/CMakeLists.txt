# Host-side e2e test of src/tele/http_conf.cpp: builds the server natively (POSIX sockets/files,
# shim/ for ESP_LOG + esp_timer) and drives it over loopback TCP.
#   cmake -S test/host-stub/http -B /tmp/http-test && cmake --build /tmp/http-test && /tmp/http-test/http_conf_test

cmake_minimum_required(VERSION 3.16)
project(http_conf_test CXX)
set(CMAKE_CXX_STANDARD 20)

add_executable(http_conf_test test_http_conf.cpp ../../../src/tele/http_conf.cpp)
target_include_directories(http_conf_test PRIVATE shim ../../../src/tele)
target_compile_options(http_conf_test PRIVATE -Wall -Wextra -Werror=missing-field-initializers)

enable_testing()
add_test(NAME http_conf COMMAND http_conf_test)
