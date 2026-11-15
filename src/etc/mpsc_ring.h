#pragma once

#include <atomic>
#include <cstdint>

// Bounded ring, many producers (any core), one consumer. No locks: a producer claims a position with
// a CAS on `head` and publishes it with a nonzero slot flag; the consumer takes the oldest published
// slot and frees it by advancing `tail`. A producer never waits. The consumer waits at a claimed but
// unpublished slot, so a producer must not be preempted between claim() and publish() (on the target:
// local interrupts masked), which bounds the wait by the time to fill one slot.
// When full, claim(evict) takes over the oldest published slot whose flags match `evict` (that line
// is lost and counted, the new one is delivered at its place).
template<class T, uint32_t N>
struct MpscRing {
    static_assert(N && !(N & (N - 1)), "N must be a power of two");

    struct Slot {
        std::atomic<uint32_t> f{0}; // 0: free, being written or being read; else the publisher's flags
        T v;
    };

    Slot slot[N];
    std::atomic<uint32_t> head{0}, tail{0}, dropped{0};

    inline __attribute__((always_inline)) Slot *claim(uint32_t evict = 0) {
        for (;;) {
            // tail first: head read later is >= it
            const uint32_t t = tail.load(std::memory_order_acquire);
            uint32_t h = head.load(std::memory_order_relaxed);
            if (h - t >= N) break;
            if (head.compare_exchange_weak(h, h + 1, std::memory_order_relaxed, std::memory_order_relaxed))
                return &slot[h % N];
        }
        if (evict) {
            const uint32_t t = tail.load(std::memory_order_relaxed);
            for (uint32_t i = 0; i < N; ++i) {
                Slot &s = slot[(t + i) % N];
                uint32_t f = s.f.load(std::memory_order_relaxed);
                if ((f & evict) && s.f.compare_exchange_strong(f, 0, std::memory_order_acquire,
                                                               std::memory_order_relaxed)) {
                    dropped.fetch_add(1, std::memory_order_relaxed);
                    return &s;
                }
            }
        }
        dropped.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }

    static inline __attribute__((always_inline)) void publish(Slot *s, uint32_t flags) {
        s->f.store(flags, std::memory_order_release);
    }

    // Consumer: the oldest slot with its flags, or nullptr when empty or that slot is still being
    // written. The slot stays reserved until pop().
    Slot *front(uint32_t &flags) {
        const uint32_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_relaxed)) return nullptr;
        Slot &s = slot[t % N];
        uint32_t f = s.f.load(std::memory_order_relaxed);
        if (!f || !s.f.compare_exchange_strong(f, 0, std::memory_order_acquire, std::memory_order_relaxed))
            return nullptr;
        flags = f;
        return &s;
    }

    void pop() { tail.store(tail.load(std::memory_order_relaxed) + 1, std::memory_order_release); }
};
