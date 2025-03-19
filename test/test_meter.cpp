#include <unity.h>
#include "metering.h"

void test_integrator() {
    TrapezoidalIntegrator<float, unsigned long, float> intE{
            1e-6f / 3600.f,  // /us => /h
            (unsigned long) 2e6f // 2sec
    };

    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 0.0f);

    intE.add(1.0, 0);
    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 0.0f);
    intE.add(1.0, 1e6);
    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 1.f / 3600.f);

    intE.add(2.0, 4e6); // discarded (> maxDt)
    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 1.f / 3600.f);

    intE.add(2.0, 5e6);
    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 1.f / 3600.f + 2.f / 3600.f);

    intE.add(3.0, 6e6);
    TEST_ASSERT_EQUAL_FLOAT(intE.get(), 1.f / 3600.f + 2.f / 3600.f + 2.5f / 3600.f);
}

void test_meter() {
    DailyEnergyMeterState day{};
    day.energyYield = 0;
    TEST_ASSERT_FALSE(day.hasEnergy());
    day.energyYield += 0.002f;
    TEST_ASSERT_TRUE(day.hasEnergy());

    auto dayCopy = day;
    TEST_ASSERT_EQUAL(0.002f, dayCopy.energyYield.toFloat());


    DailyRingStorageState<2> ring{};
    ring.clear();

    TEST_ASSERT_TRUE(ring.totalDays == 0);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FALSE(ring.ringBuf[0].hasEnergy());

    ring.add(day);
    TEST_ASSERT_TRUE(ring.totalDays == 1);
    TEST_ASSERT_TRUE(ring.ringPtr == 1);
    TEST_ASSERT_TRUE(ring.ringBuf[0].hasEnergy());

    //ring.add(DailyEnergyMeterState<float>(2.f));
    TEST_ASSERT_TRUE(ring.totalDays == 2);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, ring.ringBuf[1].energyYield.toFloat());

    //ring.add({3.f});
    TEST_ASSERT_TRUE(ring.totalDays == 3);
    TEST_ASSERT_TRUE(ring.ringPtr == 1);
    TEST_ASSERT_TRUE(ring.ringBuf[0].energyYield == 3);

    ring.clear();
    TEST_ASSERT_TRUE(ring.totalDays == 0);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FALSE(ring.ringBuf[0].hasEnergy());


}

void test_meter_storage() {
    // always start with a fresh test partition
    TEST_ASSERT_TRUE(mountLFS("littlefs_test", true));

    {
        DailyRingStorage store{"/littlefs/test_daily"};
        TEST_ASSERT_FALSE(store.load());
        TEST_ASSERT_EQUAL(0, store.getNumTotalDays());

        DailyEnergyMeterState day{};
        day.energyYield = 2.5f;
        store.add(day);
        ESP_LOGI("dbg", "added %f", store.state.ringBuf[0].energyYield.toFloat());

        TEST_ASSERT_EQUAL(1, store.getNumTotalDays());
        TEST_ASSERT_EQUAL(1, store.getAllDays().size());
        //TEST_ASSERT_EQUAL(2.5f, store.state.getAllDays()[0].energyDay.toFloat());
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, day.energyYield.toFloat());
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, store.getAllDays().back().energyYield);
    }


    DailyRingStorage store2{"/littlefs/test_daily"};
    TEST_ASSERT_TRUE(store2.load());
    TEST_ASSERT_EQUAL(1, store2.getNumTotalDays());
    TEST_ASSERT_EQUAL(1, store2.getAllDays().size());
    TEST_ASSERT_EQUAL(2.5f, store2.getAllDays()[0].energyYield);

}