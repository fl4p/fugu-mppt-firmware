// Host-side unit tests for src/etc/plot.h — Plot / Series / _plotSeries with
// low point counts.
//
// Motivation: a user-reported TLSF heap assert fired during a sweep plot where
// only 3 voltage points had been collected. These tests exercise the plot
// pipeline with N = 1..5 (the small-N corner) and a few degenerate shapes
// (collinear X, constant Y, NaN samples), to catch any out-of-bounds write
// before it lands on hardware.
//
// Build & run with AddressSanitizer (recommended — ASan catches heap-adjacent
// overwrites that the on-target TLSF assert only sees on the *next* malloc):
//   clang++ -std=gnu++17 -fexceptions -fsanitize=address -fno-omit-frame-pointer \
//       -I test/host-stub -I src \
//       -o /tmp/plot-test test/host-stub/plot-test.cpp && /tmp/plot-test
//
// Without sanitizer:
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
//       -o /tmp/plot-test test/host-stub/plot-test.cpp && /tmp/plot-test

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include "../../src/etc/plot.h"

namespace {

int g_run = 0;
int g_fail = 0;
const char *g_section = "";

void section(const char *name) {
    g_section = name;
    std::printf("[%s]\n", name);
}

#define EXPECT(cond)                                                                              \
    do {                                                                                          \
        ++g_run;                                                                                  \
        if (!(cond)) {                                                                            \
            ++g_fail;                                                                             \
            std::printf("  FAIL %s:%d  [%s]  %s\n", __FILE__, __LINE__, g_section, #cond);        \
        }                                                                                         \
    } while (0)

constexpr float NF = std::numeric_limits<float>::quiet_NaN();

// The block the RT side would append to (allocated by begin(), as on core 0 before a sweep).
PlotPoints *block(Plot &p) {
    p.begin();
    return p.collecting.load();
}

// Fill a series directly, bypassing Plot::addPoint's minimum-spacing filter so tests get N points
// regardless of x spacing.
void fill(Plot &p, int s, std::initializer_list<std::pair<float, float>> pts) {
    auto b = block(p);
    b->n[s] = 0;
    for (auto &q : pts) b->p[s][b->n[s]++] = q;
}
void fillU(Plot &p, std::initializer_list<std::pair<float, float>> pts) { fill(p, 0, pts); }
void fillD(Plot &p, std::initializer_list<std::pair<float, float>> pts) { fill(p, 1, pts); }
void pushU(Plot &p, float x, float y) { auto b = block(p); b->p[0][b->n[0]++] = {x, y}; }
void pushD(Plot &p, float x, float y) { auto b = block(p); b->p[1][b->n[1]++] = {x, y}; }

// One render cycle as in the firmware: the RT side hands the block over at the sweep end, the
// core-0 task charts and keeps it.
void doPlot(Plot &p) {
    p.handOver();
    p.plot();
}

// the block was charted and kept
bool plotted(const Plot &p) { return !p.handed.load() && !p.collecting.load() && p.block && p.block->kn == p.block->n[0]; }

} // namespace

// ------------------------------------------------------------------
//  N < 3: must early-return without touching the chart renderer.
// ------------------------------------------------------------------

void test_one_point_short_circuits() {
    section("one_point_short_circuits");
    Plot p;
    fillU(p, {{75.21f, 174.07f}});
    fillD(p, {{0.30f, 174.07f}});
    doPlot(p);
    EXPECT(plotted(p));
}

void test_two_points_short_circuits() {
    section("two_points_short_circuits");
    Plot p;
    fillU(p, {{75.21f, 174.07f}, {76.00f, 0.37f}});
    fillD(p, {{0.30f, 174.07f}, {0.45f, 0.37f}});
    doPlot(p);
    EXPECT(plotted(p));
}

// ------------------------------------------------------------------
//  N == 3: the user's actual crash scenario. minX < midX < maxX with a
//  power curve that peaks high then falls; binW is small (~0.008).
// ------------------------------------------------------------------

void test_three_points_user_scenario() {
    section("three_points_user_scenario");
    Plot p;
    // Reproduces "Grouping 3 V points (75.21,174.07)~(76.00,0.37)" from the
    // panic log. Middle point invented at 75.6V / 92W to match the visible
    // plateau in the rendered chart.
    fillU(p, {
        {75.21f, 174.07f},
        {75.60f,  92.00f},
        {76.00f,   0.37f},
    });
    fillD(p, {
        {0.30f, 174.07f},
        {0.38f,  92.00f},
        {0.45f,   0.37f},
    });
    doPlot(p);
    EXPECT(plotted(p));
}

void test_three_points_collinear_x() {
    section("three_points_collinear_x");
    // All x equal → binW == 0; the binning while-loop's `it->first < x + 0`
    // never advances `it` past the first point. Must not OOB write.
    Plot p;
    fillU(p, {
        {75.0f, 100.0f},
        {75.0f,  50.0f},
        {75.0f,  10.0f},
    });
    fillD(p, {
        {0.50f, 100.0f},
        {0.50f,  50.0f},
        {0.50f,  10.0f},
    });
    doPlot(p);
    EXPECT(plotted(p));
}

void test_three_points_constant_y() {
    section("three_points_constant_y");
    // range == 0 in ascii::Plot → fallback range=1, rows=1.
    Plot p;
    fillU(p, {
        {1.0f, 42.0f},
        {2.0f, 42.0f},
        {3.0f, 42.0f},
    });
    fillD(p, {
        {0.1f, 42.0f},
        {0.2f, 42.0f},
        {0.3f, 42.0f},
    });
    doPlot(p);
    EXPECT(plotted(p));
}

// ------------------------------------------------------------------
//  N == 4 / 5: random but deterministic, monotonically increasing x.
// ------------------------------------------------------------------

void test_four_random_points() {
    section("four_random_points");
    std::mt19937 rng(0xC0FFEE);
    std::uniform_real_distribution<float> yDist(0.0f, 200.0f);
    Plot p;
    float x = 60.0f;
    for (int i = 0; i < 4; ++i) {
        x += 0.5f + (rng() % 100) * 0.01f;
        pushU(p, x, yDist(rng));
        pushD(p, x * 0.01f, yDist(rng));
    }
    doPlot(p);
    EXPECT(plotted(p));
}

void test_five_random_points() {
    section("five_random_points");
    std::mt19937 rng(0xDEADBEEF);
    std::uniform_real_distribution<float> yDist(0.0f, 200.0f);
    Plot p;
    float x = 60.0f;
    for (int i = 0; i < 5; ++i) {
        x += 0.5f + (rng() % 100) * 0.01f;
        pushU(p, x, yDist(rng));
        pushD(p, x * 0.01f, yDist(rng));
    }
    doPlot(p);
    EXPECT(plotted(p));
}

// ------------------------------------------------------------------
//  Stress: many randomised low-N renders back-to-back. If anything
//  in the plot path writes one byte past a heap allocation, ASan or
//  the next iteration's allocator will eventually fault.
// ------------------------------------------------------------------

void test_stress_random_small_N() {
    section("stress_random_small_N");
    std::mt19937 rng(0xBADF00D);
    std::uniform_int_distribution<int> nDist(3, 5);
    std::uniform_real_distribution<float> yDist(0.001f, 250.0f);
    Plot p;
    for (int iter = 0; iter < 200; ++iter) {
        int n = nDist(rng);
        float x = 50.0f + (rng() % 1000) * 0.01f;
        for (int i = 0; i < n; ++i) {
            x += 0.1f + (rng() % 100) * 0.005f;
            pushU(p, x, yDist(rng));
            pushD(p, x * 0.013f, yDist(rng));
        }
        doPlot(p);
        EXPECT(plotted(p));
    }
}

// ------------------------------------------------------------------
//  NaN robustness: a NaN power sample shouldn't crash the chart.
//  ascii::Plot guards against NaN, but plot.h's binning uses the raw
//  value in `ya += it->second`, so all-NaN buckets propagate NaN into
//  `series`. The downstream renderer must still cope.
// ------------------------------------------------------------------

void test_three_points_with_nan_y() {
    section("three_points_with_nan_y");
    Plot p;
    fillU(p, {
        {1.0f, 100.0f},
        {2.0f, NF},
        {3.0f,  50.0f},
    });
    fillD(p, {
        {0.1f, 100.0f},
        {0.2f, NF},
        {0.3f,  50.0f},
    });
    doPlot(p);
    EXPECT(plotted(p));
}

// ------------------------------------------------------------------
//  Re-use: begin() -> add -> handOver() -> plot() repeated catches any
//  state that leaks between cycles (e.g. a block freed while still
//  referenced).
// ------------------------------------------------------------------

void test_repeated_render_cycles() {
    section("repeated_render_cycles");
    Plot p;
    for (int cycle = 0; cycle < 10; ++cycle) {
        for (int i = 0; i < 3 + (cycle % 3); ++i) {
            float x = 70.0f + cycle * 0.5f + i * 0.7f;
            float y = 10.0f + cycle + i * 13.0f;
            pushU(p, x, y);
            pushD(p, x * 0.01f, y);
        }
        doPlot(p);
        EXPECT(plotted(p));
    }
}

// ------------------------------------------------------------------
//  RT side: add() without a block drops the point, the spacing filter
//  and the capacity cap hold, restart() empties, handOver() only once.
// ------------------------------------------------------------------

void test_rt_add_and_handover() {
    section("rt_add_and_handover");
    Plot p;
    p.add(70.f, 0.5f, 100.f, 80.f); // no block yet
    EXPECT(!p.handOver());
    auto b = block(p);
    for (int i = 0; i < 1000; ++i) p.add(40.f + i * 0.04f, i * 0.001f, (float) i, 80.f);
    EXPECT(b->n[0] > 3 && b->n[0] <= PlotPoints::Cap);
    EXPECT(b->n[1] > 3 && b->n[1] <= PlotPoints::Cap);
    p.add(40.f + 999 * 0.04f, 0.999f, 1.f, 80.f); // too close to the last point
    EXPECT(b->p[0][b->n[0] - 1].second != 1.f);
    p.restart();
    EXPECT(b->n[0] == 0 && b->n[1] == 0);
    for (int i = 0; i < 10; ++i) p.add(60.f + i, 0.1f * i, 50.f + i, 80.f);
    EXPECT(p.handOver());
    EXPECT(!p.handOver()); // still pending
    p.add(70.f, 0.5f, 100.f, 80.f); // stopped collecting
    EXPECT(b->n[0] == 10);
    p.plot();
    EXPECT(plotted(p));
}

// ------------------------------------------------------------------
//  The block survives plot(): n unchanged, series 0 sorted by x and copied to k,
//  reused by the next begin(); k stays through a sweep without handOver().
// ------------------------------------------------------------------

void test_block_kept_after_plot() {
    section("block_kept_after_plot");
    Plot p;
    EXPECT(p.last() == nullptr);
    auto b = block(p);
    EXPECT(p.last() == nullptr);
    for (int i = 0; i < 50; ++i) p.add(80.f - i * 0.5f, i * 0.01f, 100.f + i, 80.f);
    const uint16_t n0 = b->n[0], n1 = b->n[1];
    const auto lastAdded = b->p[0][n0 - 1]; // smallest x
    EXPECT(n0 > 10);
    doPlot(p);
    EXPECT(plotted(p));
    EXPECT(p.last() == b);
    EXPECT(b->n[0] == n0 && b->n[1] == n1);
    bool asc = true;
    for (uint16_t i = 1; i < b->n[0]; ++i) asc &= b->p[0][i - 1].first <= b->p[0][i].first;
    EXPECT(asc);
    EXPECT(b->p[0][0] == lastAdded);
    bool same = b->kn == n0;
    for (uint16_t i = 0; i < b->kn; ++i) {
        const auto q = b->kept(i);
        same &= std::fabs(q.first - b->p[0][i].first) <= .005f && std::fabs(q.second - b->p[0][i].second) <= 1e-4f * 149;
    }
    EXPECT(same);
    EXPECT(block(p) == b); // reused
    EXPECT(b->n[0] == 0 && b->kn == n0);
    EXPECT(p.last() == b); // collecting, no handOver: the last completed sweep stays
}

// ------------------------------------------------------------------
//  An aborted sweep (begin, add, no handOver) keeps the last completed curve;
//  the next completed sweep replaces it.
// ------------------------------------------------------------------

void test_aborted_sweep_keeps_curve() {
    section("aborted_sweep_keeps_curve");
    Plot p;
    auto b = block(p);
    for (int i = 0; i < 40; ++i) p.add(70.f - i * 0.5f, i * 0.01f, 2.f + 0.1f * i, 80.f);
    p.handOver();
    p.plot(5.9f, 50.5f);
    const uint16_t kn = b->kn;
    const auto k0 = b->kept(0), kl = b->kept(kn - 1);
    EXPECT(kn > 10 && p.last() == b && b->kMpp[0] == 50.5f && b->kMpp[1] == 5.9f);
    // aborted sweep: restart(), points, no handOver(); then a second aborted one
    for (int s = 0; s < 2; ++s) {
        p.restart();
        block(p);
        for (int i = 0; i < 20; ++i) p.add(75.f - i, i * 0.02f, 100.f + i, 80.f);
        EXPECT(p.last() == b && b->kn == kn && b->kept(0) == k0 && b->kept(kn - 1) == kl);
    }
    // a completed sweep replaces it (k is x-ascending: the 129 W point first)
    p.restart();
    for (int i = 0; i < 30; ++i) p.add(75.f - i, i * 0.02f, 100.f + i, 80.f);
    p.handOver();
    p.plot(129.f, 46.f);
    EXPECT(b->kn == 30 && b->kMpp[1] == 129.f && std::fabs(b->kept(0).second - 129.f) < 0.01f);
    // negative power clamps to 0
    p.restart();
    block(p);
    for (int i = 0; i < 5; ++i) p.add(75.f - i, i * 0.02f, -1.f + i, 80.f);
    p.handOver();
    p.plot();
    EXPECT(b->kn == 5 && b->kept(4).second == 0.f && std::fabs(b->kept(0).second - 3.f) < 1e-3f);
}

// ------------------------------------------------------------------
//  main
// ------------------------------------------------------------------

int main() {
    test_one_point_short_circuits();
    test_two_points_short_circuits();
    test_three_points_user_scenario();
    test_three_points_collinear_x();
    test_three_points_constant_y();
    test_four_random_points();
    test_five_random_points();
    test_three_points_with_nan_y();
    test_repeated_render_cycles();
    test_stress_random_small_N();
    test_rt_add_and_handover();
    test_block_kept_after_plot();
    test_aborted_sweep_keeps_curve();

    std::printf("\nplot-test: %d checks, %d failures\n", g_run, g_fail);
    return g_fail == 0 ? 0 : 1;
}
