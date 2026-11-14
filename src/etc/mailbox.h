#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

/**
 * Latest-value request slot: any number of producers, one consumer. One word holds
 * tag << PayloadBits | payload; the tag is never 0, so 0 means empty and every post is a new word.
 * A post replaces an untaken one. The consumer applies peek() and then clear()s exactly that word;
 * clear() fails if a newer post replaced it, which then stays pending. A producer withdraws its own
 * post with the same clear(). idle() (acquire) sees everything the consumer did before clear().
 */
template<unsigned PayloadBits>
class ReqSlot {
    static_assert(PayloadBits > 0 && PayloadBits < 32);
    static constexpr uint32_t TagMask = UINT32_MAX >> PayloadBits;
    std::atomic<uint32_t> w_{0}, tags_{0};

public:
    static constexpr uint32_t PayloadMask = UINT32_MAX >> (32 - PayloadBits);
    static constexpr uint32_t payload(uint32_t w) { return w & PayloadMask; }
    static constexpr uint32_t tag(uint32_t w) { return w >> PayloadBits; }

    uint32_t post(uint32_t payload) {
        uint32_t t;
        do t = (tags_.fetch_add(1, std::memory_order_relaxed) + 1) & TagMask; while (!t);
        const uint32_t w = (t << PayloadBits) | (payload & PayloadMask);
        w_.store(w, std::memory_order_release);
        return w;
    }

    [[nodiscard]] uint32_t peek() const { return w_.load(std::memory_order_acquire); }
    [[nodiscard]] bool idle() const { return !peek(); }
    bool clear(uint32_t w) { return w_.compare_exchange_strong(w, 0, std::memory_order_acq_rel, std::memory_order_relaxed); }
};

/**
 * Seqlock snapshot of a trivially copyable T, stored as relaxed 32-bit atomics. One writer at a time
 * (callers serialise); readers never block the writer. seq is odd while a write is in progress.
 * begin()/put()/end() let a writer publish more inside the same window.
 */
template<class T>
class SeqLock {
    static_assert(std::is_trivially_copyable_v<T>);
    static constexpr size_t N = (sizeof(T) + 3) / 4;
    std::atomic<uint32_t> seq_{0};
    std::array<std::atomic<uint32_t>, N> w_{};

public:
    SeqLock() = default; // all-zero words
    explicit SeqLock(const T &v) { put(v); }

    void begin() {
        seq_.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
    }
    void put(const T &v) {
        uint32_t b[N]{};
        std::memcpy(b, &v, sizeof(T));
        for (size_t i = 0; i < N; ++i) w_[i].store(b[i], std::memory_order_relaxed);
    }
    void end() { seq_.fetch_add(1, std::memory_order_release); }
    void write(const T &v) { begin(); put(v); end(); }

    [[nodiscard]] uint32_t enter() const { return seq_.load(std::memory_order_acquire); }
    // unchecked: every word is atomic, the set may mix two writes
    [[nodiscard]] T get() const {
        uint32_t b[N];
        for (size_t i = 0; i < N; ++i) b[i] = w_[i].load(std::memory_order_relaxed);
        T v;
        std::memcpy(&v, b, sizeof(T));
        return v;
    }
    [[nodiscard]] bool still(uint32_t s) const {
        std::atomic_thread_fence(std::memory_order_acquire);
        return seq_.load(std::memory_order_relaxed) == s;
    }
    bool tryRead(T &v) const {
        const uint32_t s = enter();
        if (s & 1u) return false;
        v = get();
        return still(s);
    }
};
