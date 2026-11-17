// Host test for the idle-profile CPU guard decision (src/etc/idle_profile.h).
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -I src -o /tmp/idle-guard-test test/host-stub/idle-guard-test.cpp && /tmp/idle-guard-test

#include <cstdio>
#include <initializer_list>

#include "etc/idle_profile.h"

uint32_t g_teleMinPointUs = kTeleMinPointUs;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++fails; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); puts(""); } } while (0)

using V = IdleCpuVerdict;

int main() {
    // window rate
    CHECK(idleWinRate(1000, 1000 + 4425, 3'000'000, false) == 1475.f, "rate");
    CHECK(idleWinRate(0, 0, 3'000'000, false) == 0.f, "zero samples is a rate of 0");
    CHECK(std::isnan(idleWinRate(5000, 100, 3'000'000, false)), "counter went back");
    CHECK(std::isnan(idleWinRate(0, 10, 0, false)), "no time");
    CHECK(std::isnan(idleWinRate(0, 10, -5, false)), "negative time");
    CHECK(std::isnan(idleWinRate(0, 4425, 3'000'000, true)), "disturbed");
    CHECK(idleWinRate(0xfffffff0u, 0x10u, 1'000'000, false) == 32.f, "counter wrap");

    // unevaluable base: never lower
    for (float pre: {0.f, -1.f, NAN, INFINITY})
        CHECK(idleCpuVerdict(pre, 1475.f) == V::NoBase, "pre %f", pre);

    // unevaluable idle rate: restore
    CHECK(idleCpuVerdict(1475.f, NAN) == V::Restore, "idle NAN");
    CHECK(idleCpuVerdict(1475.f, INFINITY) == V::Restore, "idle inf");
    CHECK(idleCpuVerdict(1475.f, 0.f) == V::Restore, "idle 0");

    // flat 2026-11: 1070 at 80 MHz against 1475
    CHECK(idleCpuVerdict(1475.f, 1070.f) == V::Restore, "flat");
    CHECK(idleCpuVerdict(1475.f, 1475.f) == V::Keep, "same rate");
    CHECK(idleCpuVerdict(1475.f, 1475.f * 0.97f) == V::Keep, "at the threshold");
    CHECK(idleCpuVerdict(1475.f, 1475.f * 0.969f) == V::Restore, "below the threshold");
    CHECK(idleCpuVerdict(512.f, 511.f) == V::Keep, "fry, one sample less");

    // monotonic: once restored, every lower idle rate restores too
    for (float pre: {10.f, 512.f, 1475.f, 3000.f}) {
        bool restored = false;
        for (float r = pre * 1.5f; r >= 0; r -= pre / 997.f) {
            const V v = idleCpuVerdict(pre, r);
            if (restored) CHECK(v == V::Restore, "pre %.0f idle %.2f flipped back", pre, r);
            restored |= v == V::Restore;
        }
        CHECK(restored, "pre %.0f never restored", pre);
    }

    printf("%s (%d failures)\n", fails ? "FAILED" : "OK", fails);
    return fails != 0;
}
