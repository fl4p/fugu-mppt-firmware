#pragma once
struct SimLittleFS { bool begin(bool, const char *, int, const char *) { return true; } };
inline SimLittleFS LittleFS;
