#pragma once
// Host stub: the test defines heap_caps_get_free_size.
#include <cstddef>
#include <cstdint>
#define MALLOC_CAP_DEFAULT (1 << 12)
size_t heap_caps_get_free_size(uint32_t caps);
