// Second translation unit for net-bytes-test.cpp: proves the byte counter in scope.h is one shared
// object, not a per-TU copy (the issue #71 defect).
#include "tele/scope.h"

void netBytesAddFromOtherTu(size_t n) {
    bytesSent.fetch_add((uint32_t) n, std::memory_order_relaxed);
}
