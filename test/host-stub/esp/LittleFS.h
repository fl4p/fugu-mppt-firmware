#pragma once
struct LittleFSStub { bool begin(bool, const char *, int, const char *) { return true; } };
inline LittleFSStub LittleFS;
