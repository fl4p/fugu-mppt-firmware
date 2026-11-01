// Tests for src/math/statmath.h pure filters: EWMA, EWMA_nPass, RunningMedian3/5,
// MeanAccumulator, EWM and the standalone TrapezoidalIntegrator. All pure-function,
// no hardware.

#include <unity.h>
#include <esp_log.h>
#include <Arduino.h>

#include "math/statmath.h"

// ---------------------------------------------------------------- EWMA
void test_ewma_seeds_on_first_finite() {
    EWMA<float> e{10};
    TEST_ASSERT_TRUE(std::isnan(e.get()));
    e.add(5.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f, e.get()); // first finite seeds y
}

void test_ewma_ignores_nan() {
    EWMA<float> e{10};
    e.add(NAN);
    TEST_ASSERT_TRUE(std::isnan(e.get())); // NaN before seed leaves y unset
    e.add(5.f);
    e.add(NAN);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f, e.get()); // NaN after seed is dropped
}

void test_ewma_converges_to_constant() {
    EWMA<float> e{20};
    for (int i = 0; i < 200; ++i) e.add(3.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.f, e.get());
}

void test_ewma_reset_and_span() {
    EWMA<float> e{9};
    TEST_ASSERT_EQUAL_UINT32(9, e.span()); // span round-trips through alpha
    e.add(1.f);
    e.reset();
    TEST_ASSERT_TRUE(std::isnan(e.get()));
}

// ---------------------------------------------------------------- EWMA_nPass
void test_ewma_npass_converges_to_constant() {
    EWMA_nPass<2, float> e{15};
    for (int i = 0; i < 300; ++i) e.add(7.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 7.f, e.get()); // get() == last pass
}

void test_ewma_npass_nan_until_first_sample() {
    EWMA_nPass<2, float> e{15};
    TEST_ASSERT_TRUE(std::isnan(e.get()));
    e.add(27.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 27.f, e.get()); // every pass seeds on the first sample
}

// ---------------------------------------------------------------- RunningMedian3
void test_median3_first_sample_passthrough() {
    RunningMedian3<float> m;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, m.next(2.f)); // unseeded window returns input
}

void test_median3_rejects_spike() {
    RunningMedian3<float> m;
    m.next(1.f);
    m.next(1.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, m.next(100.f)); // window {100,1,1} -> 1
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, m.next(1.f));
}

void test_median3_picks_middle() {
    RunningMedian3<float> m;
    m.next(2.f);
    m.next(4.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.f, m.next(3.f)); // median{3,4,2} == 3
}

// ---------------------------------------------------------------- RunningMedian5
void test_median5_picks_middle() {
    RunningMedian5<float> m;
    m.next(1.f); m.next(2.f); m.next(3.f); m.next(4.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.f, m.next(5.f)); // median{1..5} == 3
}

void test_median5_rejects_spike() {
    RunningMedian5<float> m;
    for (int i = 0; i < 4; ++i) m.next(10.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.f, m.next(1000.f)); // single outlier ignored
}

// ---------------------------------------------------------------- MeanAccumulator
void test_mean_accumulator_pop_resets() {
    MeanAccumulator a;
    a.add(2.f); a.add(4.f); a.add(6.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.f, a.getMean());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.f, a.pop()); // pop returns mean and clears
    a.add(8.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 8.f, a.getMean());
}

// ---------------------------------------------------------------- EWM
void test_ewm_mean_tracks_and_var_zero_for_constant() {
    EWM<false, float> e{20};
    for (int i = 0; i < 300; ++i) e.add(4.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.f, e.avg.get());
    TEST_ASSERT_TRUE(e.nvar() >= 0.f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.f, e.nvar()); // constant series -> ~0 variance
}

// ---------------------------------------------------------------- TrapezoidalIntegrator
void test_integrator_constant_rate() {
    // timeFactor=1 so value is the raw time-integral; maxDt=1e6 ticks
    TrapezoidalIntegrator<float, unsigned long, double> ti{1.f, 1000000UL};
    ti.add(2.f, 0);
    ti.add(2.f, 100); // (2+2)/2 * 100 = 200
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 200.0, ti.get());
}

void test_integrator_drops_gap_over_maxdt() {
    TrapezoidalIntegrator<float, unsigned long, double> ti{1.f, 1000000UL};
    ti.add(2.f, 0);
    ti.add(2.f, 100);             // value = 200
    ti.add(2.f, 100 + 2000000UL); // dt >= maxDt -> dropped
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 200.0, ti.get());
}

void test_integrator_reset_and_restore() {
    TrapezoidalIntegrator<float, unsigned long, double> ti{1.f, 1000000UL};
    ti.add(2.f, 0);
    ti.add(2.f, 100);
    ti.reset();
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, ti.get());
    ti.restore(50.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 50.0, ti.get());
}
