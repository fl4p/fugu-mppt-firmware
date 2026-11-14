#pragma once
inline bool heap_caps_check_integrity_all(bool) { return true; }
#include <cstddef>
#include <cstdint>
#define MALLOC_CAP_DEFAULT (1 << 12)
inline size_t heap_caps_get_free_size(uint32_t) { return 0; }
inline size_t heap_caps_get_minimum_free_size(uint32_t) { return 0; }
