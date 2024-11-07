#pragma once


#include <driver/uart.h>

const unsigned long &loopWallClockUs();
unsigned long loopWallClockMs();

void scan_i2c();

#define assert_throw(cond, msg) do { if(!(cond)) throw std::runtime_error(msg " (" #cond ") is false"); } while(0)
