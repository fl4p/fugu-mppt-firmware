// Host regression for the shared network-byte counter in src/tele/scope.h (issue #71). The counter
// used to be `static uint64_t` in the header, so telemetry.cpp and main.cpp incremented/read two
// different objects and the status line never saw the UDP bytes. This test spans two translation
// units and races writers against the snapshot/reset reader to pin three properties:
//   1. increments from both TUs land in one counter,
//   2. concurrent fetch_add() against exchange(0) loses no increment,
//   3. the interim read (load) does not consume the window.
//
// Build & run:
//   test/host-stub/host-tests.sh net-bytes-test

#include "tele/scope.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

void netBytesAddFromOtherTu(size_t n); // net-bytes-other-tu.cpp

static int failures = 0;
#define CHECK(c, ...)                                                        \
    do {                                                                     \
        if (!(c)) {                                                          \
            ++failures;                                                      \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__);                 \
            std::printf(__VA_ARGS__);                                        \
            std::printf("\n");                                               \
        }                                                                    \
    } while (0)

// 1. One counter across TUs: a per-TU copy would make this 1000, not 3345.
static void testSharedAcrossTus() {
    bytesSent.exchange(0, std::memory_order_relaxed);
    bytesSent.fetch_add(1000, std::memory_order_relaxed);
    netBytesAddFromOtherTu(2345);
    uint32_t got = bytesSent.exchange(0, std::memory_order_relaxed);
    CHECK(got == 3345, "cross-TU total %u (want 3345)", got);
}

// 2. Concurrent accumulation vs snapshot/reset: every byte written must appear in exactly one
//    exchanged window. A lossy read-then-zero would drop the increments racing it.
static void testConcurrentNoLoss() {
    bytesSent.exchange(0, std::memory_order_relaxed);
    constexpr int kThreads = 4;
    constexpr uint32_t kPerThread = 250000;
    std::atomic<uint32_t> readTotal{0};
    std::atomic<bool> stop{false};

    std::vector<std::thread> writers;
    for (int i = 0; i < kThreads; ++i)
        writers.emplace_back([i] {
            for (uint32_t k = 0; k < kPerThread; ++k) {
                if (i & 1) netBytesAddFromOtherTu(1);
                else bytesSent.fetch_add(1, std::memory_order_relaxed);
            }
        });

    std::thread reader([&] {
        while (!stop.load(std::memory_order_relaxed))
            readTotal.fetch_add(bytesSent.exchange(0, std::memory_order_relaxed), std::memory_order_relaxed);
    });

    for (auto &t : writers) t.join();
    stop.store(true, std::memory_order_relaxed);
    reader.join();
    readTotal.fetch_add(bytesSent.exchange(0, std::memory_order_relaxed), std::memory_order_relaxed);

    const uint32_t expected = (uint32_t) kThreads * kPerThread;
    CHECK(readTotal.load() == expected, "lost bytes: read %u, wrote %u", readTotal.load(), expected);
}

// 3. Interim snapshot is read-only: two peeks see the same window, and the take still returns it.
static void testPeekDoesNotConsume() {
    bytesSent.exchange(0, std::memory_order_relaxed);
    bytesSent.fetch_add(500, std::memory_order_relaxed);
    uint32_t a = bytesSent.load(std::memory_order_relaxed);
    uint32_t b = bytesSent.load(std::memory_order_relaxed);
    CHECK(a == 500 && b == 500, "peek consumed the window: %u then %u", a, b);
    CHECK(bytesSent.exchange(0, std::memory_order_relaxed) == 500, "take after peek");
    CHECK(bytesSent.exchange(0, std::memory_order_relaxed) == 0, "second take should be empty");
}

int main() {
    testSharedAcrossTus();
    testConcurrentNoLoss();
    testPeekDoesNotConsume();
    if (failures) std::printf("net-bytes-test: %d FAILED\n", failures);
    else std::printf("net-bytes-test: all passed\n");
    return failures ? 1 : 0;
}
