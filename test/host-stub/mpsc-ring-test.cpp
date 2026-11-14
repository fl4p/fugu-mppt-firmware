// Host test for src/etc/mpsc_ring.h (RT log ring and rt_defer): concurrent producers, full ring,
// eviction of low-severity slots, and a claimed-but-unpublished slot.
//
// Build & run (plain and under ThreadSanitizer):
//   clang++ -std=gnu++20 -O1 -pthread -o /tmp/mpsc-ring-test test/host-stub/mpsc-ring-test.cpp && /tmp/mpsc-ring-test
//   clang++ -std=gnu++20 -O1 -g -pthread -fsanitize=thread -o /tmp/mpsc-ring-tsan test/host-stub/mpsc-ring-test.cpp && /tmp/mpsc-ring-tsan

#include "../../src/etc/mpsc_ring.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); return 1; } } while (0)

struct Item {
    uint32_t producer, seq;
    uint32_t pay[30]; // f(producer, seq): a torn slot shows as a mismatch
    void fill(uint32_t p, uint32_t s) {
        producer = p;
        seq = s;
        for (uint32_t i = 0; i < 30; ++i) pay[i] = p * 2654435761u ^ s * 40503u ^ i;
    }
    bool ok() const {
        for (uint32_t i = 0; i < 30; ++i) if (pay[i] != (producer * 2654435761u ^ seq * 40503u ^ i)) return false;
        return true;
    }
};

constexpr uint32_t Set = 1, Low = 2;

static int testFull() {
    MpscRing<Item, 16> r;
    for (uint32_t i = 0; i < 16; ++i) {
        auto *s = r.claim();
        CHECK(s);
        s->v.fill(0, i);
        r.publish(s, Set);
    }
    CHECK(!r.claim());
    CHECK(!r.claim(Low)); // nothing low to evict
    CHECK(r.dropped == 2);
    uint32_t f, n = 0;
    while (auto *s = r.front(f)) {
        CHECK(s->v.seq == n++ && s->v.ok() && f == Set);
        r.pop();
    }
    CHECK(n == 16);
    CHECK(r.claim()); // free again (wrap)
    return 0;
}

static int testEvict() {
    MpscRing<Item, 8> r;
    for (uint32_t i = 0; i < 8; ++i) {
        auto *s = r.claim();
        s->v.fill(0, i);
        r.publish(s, Set | ((i == 2 || i == 5) ? Low : 0));
    }
    // a low line on a full ring is dropped, it never evicts
    CHECK(!r.claim(0));
    // a high line takes the oldest low slot (position 2), then position 5, then nothing
    for (uint32_t k : {100u, 101u}) {
        auto *s = r.claim(Low);
        CHECK(s);
        s->v.fill(1, k);
        r.publish(s, Set);
    }
    CHECK(!r.claim(Low));
    CHECK(r.dropped == 4); // 1 low dropped + 2 evicted + 1 high without room
    const uint32_t want[8][2] = {{0, 0}, {0, 1}, {1, 100}, {0, 3}, {0, 4}, {1, 101}, {0, 6}, {0, 7}};
    uint32_t f;
    for (auto &w : want) {
        auto *s = r.front(f);
        CHECK(s && s->v.producer == w[0] && s->v.seq == w[1] && s->v.ok() && !(f & Low));
        r.pop();
    }
    CHECK(!r.front(f));
    return 0;
}

static int testStuck() {
    MpscRing<Item, 8> r;
    auto *stuck = r.claim(); // claimed, not yet published
    CHECK(stuck);
    for (uint32_t i = 1; i < 8; ++i) {
        auto *s = r.claim();
        s->v.fill(0, i);
        r.publish(s, Set);
    }
    uint32_t f;
    CHECK(!r.front(f)); // the consumer waits at the oldest position
    CHECK(!r.claim());  // others never wait: the ring is full and they drop
    // the consumer cannot take the in-progress slot by eviction either
    CHECK(!r.claim(Low));
    stuck->v.fill(0, 0);
    r.publish(stuck, Set);
    for (uint32_t i = 0; i < 8; ++i) {
        auto *s = r.front(f);
        CHECK(s && s->v.seq == i && s->v.ok());
        r.pop();
    }
    return 0;
}

// P producers, some lines low; the consumer checks integrity and per-producer order of high lines
// (evictions break the order only of the evicting line, which takes an earlier position).
static int testConcurrent(bool evict) {
    constexpr uint32_t P = 4, M = 200000;
    auto *rp = new MpscRing<Item, 16>();
    auto &r = *rp;
    std::atomic<uint32_t> done{0};
    std::atomic<uint64_t> sent{0};
    std::vector<std::thread> th;
    for (uint32_t p = 0; p < P; ++p)
        th.emplace_back([&, p] {
            for (uint32_t i = 0; i < M; ++i) {
                const bool low = evict && (i % 3) == 0;
                auto *s = r.claim(evict && !low ? Low : 0);
                if (!s) continue;
                s->v.fill(p, i);
                r.publish(s, Set | (low ? Low : 0));
                sent.fetch_add(1, std::memory_order_relaxed);
            }
            done.fetch_add(1);
        });
    uint64_t got = 0, torn = 0, disorder = 0, stuck = 0;
    auto idleSince = std::chrono::steady_clock::now();
    int64_t last[P];
    for (auto &l : last) l = -1;
    for (;;) {
        uint32_t f;
        auto *s = r.front(f);
        if (!s) {
            if (done.load() == P && r.tail.load() == r.head.load()) break;
            // all producers finished, yet a position never gets published
            if (done.load() == P && std::chrono::steady_clock::now() - idleSince > std::chrono::seconds(2)) { stuck = 1; break; }
            std::this_thread::yield();
            continue;
        }
        idleSince = std::chrono::steady_clock::now();
        const Item it = s->v;
        r.pop();
        ++got;
        if (!it.ok() || it.producer >= P) { ++torn; continue; }
        if (!evict && (int64_t) it.seq <= last[it.producer]) ++disorder;
        if ((int64_t) it.seq > last[it.producer]) last[it.producer] = it.seq;
    }
    for (auto &t : th) t.join();
    const uint64_t evicted = sent.load() - got;
    printf("  concurrent evict=%d: %llu delivered, %u dropped (%llu evicted), torn %llu, out of order %llu\n",
           evict, (unsigned long long) got, r.dropped.load(), (unsigned long long) evicted,
           (unsigned long long) torn, (unsigned long long) disorder);
    CHECK(!stuck);
    CHECK(torn == 0);
    CHECK(disorder == 0);
    // every attempt is delivered or counted as dropped (an evicted line is counted, its evictor delivered)
    CHECK(got + r.dropped.load() == (uint64_t) P * M);
    if (!evict) CHECK(evicted == 0);
    delete rp;
    return 0;
}

int main() {
    int rc = 0;
    rc |= testFull();
    rc |= testEvict();
    rc |= testStuck();
    rc |= testConcurrent(false);
    rc |= testConcurrent(true);
    printf(rc ? "mpsc-ring-test: FAIL\n" : "mpsc-ring-test: OK\n");
    return rc;
}
