#include <unity.h>

#include <cstdlib>
#include "math/float16.h"


float randf() {
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

void test_float16() {
    for (auto i = 0; i < 1000; ++i) {
        auto f32 = randf();

        float16 f16 = f32;
        TEST_ASSERT_FLOAT_WITHIN(1e-6, f32, f16.toFloat());

        auto f2 = f16 * 2;
        TEST_ASSERT_FLOAT_WITHIN(1e-6, f32 * 2, f2.toFloat());

        auto f3 = f16 * f16;
        TEST_ASSERT_FLOAT_WITHIN(1e-6, f32 * f32, f3.toFloat());
    }
}
