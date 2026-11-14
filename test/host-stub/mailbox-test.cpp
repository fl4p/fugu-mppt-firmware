// Host test for src/etc/mailbox.h (ReqSlot, SeqLock): coalescing, stale ack, cancel, tag wrap-around,
// two producers against one consumer on std::thread, and the PSU pattern (payload seqlock + command slot).
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/mailbox-test.cpp -o /tmp/mailbox-test && /tmp/mailbox-test
// Under ThreadSanitizer:
//   clang++ -std=gnu++20 -O1 -g -fsanitize=thread -I src test/host-stub/mailbox-test.cpp -o /tmp/mailbox-tsan \
//       && /tmp/mailbox-tsan

#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#include "etc/mailbox.h"

static int failures = 0;
#define CHECK(c, ...) do { if (!(c)) { ++failures; std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
    std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static void basics() {
    ReqSlot<8> s;
    CHECK(s.idle() && !s.peek(), "starts empty");
    const uint32_t a = s.post(7);
    CHECK(a && ReqSlot<8>::payload(a) == 7 && ReqSlot<8>::tag(a) == 1, "post word %08x", a);
    CHECK(!s.idle() && s.peek() == a, "pending");
    CHECK(s.clear(a) && s.idle(), "ack empties");
    CHECK(!s.clear(a), "second ack of the same word fails");

    // payload 0 is a valid request
    const uint32_t z = s.post(0);
    CHECK(z && !s.idle() && ReqSlot<8>::payload(z) == 0, "zero payload pending");
    s.clear(z);
}

static void coalescing() {
    ReqSlot<8> s;
    const uint32_t a = s.post(1), b = s.post(2), c = s.post(3);
    CHECK(a != b && b != c, "distinct words");
    CHECK(s.peek() == c, "latest wins");
    CHECK(!s.clear(a) && !s.clear(b), "acks of replaced posts fail");
    CHECK(s.clear(c) && s.idle(), "latest acked");
}

static void staleAck() {
    ReqSlot<8> s;
    const uint32_t a = s.post(5);
    const uint32_t taken = s.peek(); // consumer reads a ...
    const uint32_t b = s.post(5);    // ... a producer re-posts the same payload meanwhile
    CHECK(taken == a && b != a, "same payload, new tag");
    CHECK(!s.clear(taken), "stale ack must fail");
    CHECK(!s.idle() && s.peek() == b, "new post stays pending");
    CHECK(s.clear(b), "next pass acks it");
}

static void cancel() {
    ReqSlot<8> s;
    const uint32_t a = s.post(9);
    CHECK(s.clear(a) && s.idle(), "producer withdraws its own post");
    const uint32_t b = s.post(9);
    const uint32_t c = s.post(4);  // another producer replaced it
    CHECK(!s.clear(b) && s.peek() == c, "cannot withdraw a replaced post");
}

static void wrapAround() {
    ReqSlot<30> s; // 2-bit tag: 1, 2, 3, 1, ...
    uint32_t prev = 0;
    for (int i = 0; i < 12; ++i) {
        const uint32_t w = s.post(0x3fffffffu);
        CHECK(ReqSlot<30>::tag(w) == (uint32_t) (i % 3 + 1), "tag %u at %d", ReqSlot<30>::tag(w), i);
        CHECK(w != prev, "consecutive posts differ across the wrap");
        CHECK(ReqSlot<30>::payload(w) == 0x3fffffffu, "payload intact");
        CHECK(s.clear(w), "ack");
        prev = w;
    }
    ReqSlot<31> one; // 1-bit tag: always 1, so an identical payload re-post is the same word (ABA, benign)
    const uint32_t a = one.post(0x7fffffffu), b = one.post(0x7fffffffu);
    CHECK(a == b && a == 0xffffffffu, "1-bit tag word %08x %08x", a, b);
    CHECK(one.post(0) == 0x80000000u, "1-bit tag, payload 0 is not empty");

    ReqSlot<8> big; // ticket-sized tags wrap past 2^24 without producing 0
    for (uint32_t i = 0; i < (1u << 24) + 5; ++i) {
        const uint32_t w = big.post(1);
        if (!ReqSlot<8>::tag(w)) { CHECK(false, "zero tag at %u", i); break; }
    }
    big.clear(big.peek());
}

// Two producers post (id << 20 | counter); the consumer applies and acks. Per producer the applied
// counters must strictly increase (latest value, never an older one after a newer), nothing torn, and
// the final post is applied.
static void twoProducers() {
    ReqSlot<22> s;
    constexpr uint32_t Posts = 200000;
    std::atomic<int> running{2};
    std::vector<uint32_t> applied;
    applied.reserve(Posts * 2);
    uint32_t lastSeen[2] = {0, 0};
    bool order = true, torn = false;

    std::thread consumer([&] {
        for (;;) {
            const bool done = running.load(std::memory_order_acquire) == 0;
            if (const uint32_t w = s.peek()) {
                const uint32_t p = ReqSlot<22>::payload(w), id = p >> 20, n = p & 0xfffff;
                if (id > 1 || !n) torn = true;
                else {
                    if (n <= lastSeen[id]) order = false;
                    lastSeen[id] = n;
                }
                applied.push_back(p);
                s.clear(w);
            } else if (done) {
                break;
            }
        }
    });
    auto producer = [&](uint32_t id) {
        for (uint32_t n = 1; n <= Posts; ++n) {
            s.post(id << 20 | n);
            if ((n & 1023) == 0) std::this_thread::yield();
        }
        running.fetch_sub(1, std::memory_order_release);
    };
    std::thread p0(producer, 0), p1(producer, 1);
    p0.join();
    p1.join();
    consumer.join();
    CHECK(!torn, "torn payload");
    CHECK(order, "applied out of order");
    CHECK(s.idle(), "drained");
    CHECK(!applied.empty() && (applied.back() & 0xfffff) == Posts, "last applied is a final post (%x)",
          applied.empty() ? 0 : applied.back());
    std::printf("two producers: %zu of %u posts applied (rest coalesced)\n", applied.size(), Posts * 2);
}

// Handshake as the console does it: post, wait for idle(), then read what the consumer wrote before
// its ack. `state` is a plain variable: TSan flags it unless clear() release / idle() acquire order it.
static void handshake() {
    ReqSlot<16> s;
    uint32_t state = 0;
    std::atomic<bool> stop{false};
    std::thread consumer([&] {
        while (!stop.load(std::memory_order_relaxed))
            if (const uint32_t w = s.peek()) {
                state = ReqSlot<16>::payload(w) * 3;
                s.clear(w);
            }
    });
    bool ok = true;
    for (uint32_t i = 1; i < 20000 && ok; ++i) {
        s.post(i);
        while (!s.idle()) {}
        ok = state == i * 3;
    }
    stop = true;
    consumer.join();
    CHECK(ok, "consumer effect not visible after idle()");
}

struct Snap {
    float a, b;
    int32_t c;
    bool on;
};

// post[i] is a plain (non-atomic) shadow written inside write i and read only after a successful read
// of it, so TSan flags it unless end() release pairs with enter() acquire. TSan ignores standalone
// fences, so begin()/still() ordering cannot be checked this way (a fence-paired shadow is reported
// even when correct).
static void seqlockThreads() {
    constexpr int Writes = 300000;
    std::vector<int> post(Writes);
    SeqLock<Snap> sl{{0, 0, 0, false}};
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        for (int i = 1; i < Writes; ++i) {
            sl.begin();
            post[i] = i;
            sl.put({(float) i, (float) i * 2, -i, (i & 1) != 0});
            sl.end();
        }
        stop = true;
    });
    std::atomic<long> good{0}, bad{0};
    auto reader = [&] {
        while (!stop.load(std::memory_order_relaxed)) {
            const uint32_t s = sl.enter();
            if (s & 1u) continue;
            const Snap v = sl.get();
            if (!sl.still(s)) continue;
            const int i = -v.c;
            if (i < 0 || i >= Writes) { ++bad; continue; }
            if (v.a != (float) i || v.b != (float) i * 2 || v.on != ((i & 1) != 0) || post[i] != i) ++bad;
            else ++good;
        }
    };
    std::thread r(reader);
    reader();
    writer.join();
    r.join();
    CHECK(!bad, "%ld mixed snapshots", bad.load());
    CHECK(good > 0, "no successful read (%ld)", good.load());
    Snap last{};
    CHECK(sl.tryRead(last) && last.c == -299999, "final snapshot");
}

// PSU pattern: producers serialised by a lock write the payload and post the command inside one seqlock
// window; the consumer takes the command only with a payload from the same window.
static void payloadWithCommand() {
    struct Req { uint32_t x; float v; uint32_t check; };
    SeqLock<Req> req;
    ReqSlot<8> cmd;
    std::mutex mux;
    std::atomic<int> running{2};
    long taken = 0, mismatch = 0, retries = 0;
    std::thread consumer([&] {
        for (;;) {
            const bool done = running.load(std::memory_order_acquire) == 0;
            if (!cmd.peek()) { if (done) break; continue; }
            const uint32_t s = req.enter();
            if (s & 1u) { ++retries; continue; }
            const uint32_t w = cmd.peek();
            if (!w) continue;
            const Req r = req.get();
            if (!req.still(s) || !cmd.clear(w)) { ++retries; continue; }
            ++taken;
            if (r.check != ~r.x || r.v != (float) r.x || ReqSlot<8>::payload(w) != (r.x & 0xff))
                ++mismatch;
        }
    });
    uint32_t next = 0; // guarded by mux
    auto producer = [&] {
        for (int i = 0; i < 100000; ++i) {
            std::lock_guard<std::mutex> g(mux);
            const uint32_t x = ++next;
            req.begin();
            req.put({x, (float) x, ~x});
            cmd.post(x & 0xff);
            req.end();
        }
        running.fetch_sub(1, std::memory_order_release);
    };
    std::thread p0(producer), p1(producer);
    p0.join();
    p1.join();
    consumer.join();
    CHECK(!mismatch, "%ld commands paired with another window's payload", mismatch);
    CHECK(taken > 0, "nothing taken");
    std::printf("payload+command: %ld taken, %ld retries\n", taken, retries);
}

int main() {
    basics();
    coalescing();
    staleAck();
    cancel();
    wrapAround();
    twoProducers();
    handshake();
    seqlockThreads();
    payloadWithCommand();
    std::printf(failures ? "mailbox-test: %d FAILED\n" : "mailbox-test: all passed\n", failures);
    return failures ? 1 : 0;
}
