#include <unity.h>
#include "metering.h"

void test_meter() {
    DailyEnergyMeterState day{};
    day.energyDay = 0;
    TEST_ASSERT_FALSE(day.hasEnergy());
    day.energyDay += 0.002f;
    TEST_ASSERT_TRUE(day.hasEnergy());

    auto dayCopy = day;
    TEST_ASSERT_EQUAL(0.002f, dayCopy.energyDay.toFloat());


    DailyRingStorageState<2> ring{};
    ring.clear();

    TEST_ASSERT_TRUE(ring.totalDays == 0);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FALSE(ring.ringBuf[0].hasEnergy());

    ring.add(day);
    TEST_ASSERT_TRUE(ring.totalDays == 1);
    TEST_ASSERT_TRUE(ring.ringPtr == 1);
    TEST_ASSERT_TRUE(ring.ringBuf[0].hasEnergy());

    ring.add({2.f});
    TEST_ASSERT_TRUE(ring.totalDays == 2);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, ring.ringBuf[1].energyDay.toFloat() );

    ring.add({3.f});
    TEST_ASSERT_TRUE(ring.totalDays == 3);
    TEST_ASSERT_TRUE(ring.ringPtr == 1);
    TEST_ASSERT_TRUE(ring.ringBuf[0].energyDay == 3);

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
        day.energyDay = 2.5f;
        store.add(day);
        TEST_ASSERT_EQUAL(1, store.getNumTotalDays());
        TEST_ASSERT_EQUAL(1, store.getAllDays().size());
        //TEST_ASSERT_EQUAL(2.5f, store.state.getAllDays()[0].energyDay.toFloat());
        TEST_ASSERT_EQUAL(2.5f, store.getAllDays()[0].energyDay.toFloat());
    }


    DailyRingStorage store2{"/littlefs/test_daily"};
    TEST_ASSERT_TRUE(store2.load());
    TEST_ASSERT_EQUAL(1, store2.getNumTotalDays());
    TEST_ASSERT_EQUAL(1, store2.getAllDays().size());
    TEST_ASSERT_EQUAL(2.5f, store2.getAllDays()[0].energyDay.toFloat());

}