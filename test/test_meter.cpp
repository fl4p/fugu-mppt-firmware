#include <unity.h>
#include <memory>
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

    DailyEnergyMeterState<float16> day2{};
    day2.energyYield = 2.f;
    ring.add(day2);
    TEST_ASSERT_TRUE(ring.totalDays == 2);
    TEST_ASSERT_TRUE(ring.ringPtr == 0); // (1+1) % 2 wraps
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, ring.ringBuf[1].energyYield.toFloat());

    DailyEnergyMeterState<float16> day3{};
    day3.energyYield = 3.f;
    ring.add(day3);
    TEST_ASSERT_TRUE(ring.totalDays == 3);
    TEST_ASSERT_TRUE(ring.ringPtr == 1);
    TEST_ASSERT_TRUE(ring.ringBuf[0].energyYield == 3); // ringBuf[0] overwritten

    ring.clear();
    TEST_ASSERT_TRUE(ring.totalDays == 0);
    TEST_ASSERT_TRUE(ring.ringPtr == 0);
    TEST_ASSERT_FALSE(ring.ringBuf[0].hasEnergy());


}

void test_meter_storage() {
    // format + mount the dedicated test partition (64k) at /littlefs_test, isolated from
    // the production /littlefs. format=true gives a clean slate every run.
    TEST_ASSERT_TRUE(mountLFS("littlefs_test", true));

    // DailyRingStorage embeds DailyRingStorageState<1000> (~14 KB). The Arduino loop
    // task that runs the tests has only an 8 KB stack (CONFIG_ARDUINO_LOOP_STACK_SIZE),
    // so the object must live on the heap or it overflows the stack and corrupts memory.
    {
        auto store = std::make_unique<DailyRingStorage>("/littlefs_test/test_daily");
        TEST_ASSERT_FALSE(store->load());
        TEST_ASSERT_EQUAL(0, store->getNumTotalDays());

        DailyEnergyMeterState day{};
        day.energyYield = 2.5f;
        store->add(day);
        ESP_LOGI("dbg", "added %f", store->state.ringBuf[0].energyYield.toFloat());

        TEST_ASSERT_EQUAL(1, store->getNumTotalDays());
        TEST_ASSERT_EQUAL(1, store->getAllDays().size());
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, day.energyYield.toFloat());
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, store->getAllDays().back().energyYield);
    }

    auto store2 = std::make_unique<DailyRingStorage>("/littlefs_test/test_daily");
    TEST_ASSERT_TRUE(store2->load());
    TEST_ASSERT_EQUAL(1, store2->getNumTotalDays());
    TEST_ASSERT_EQUAL(1, store2->getAllDays().size());
    TEST_ASSERT_EQUAL(2.5f, store2->getAllDays()[0].energyYield);
}

// runs after test_meter_storage, which mounts /littlefs_test
void test_meter_persistent_state() {
    const char *fn = "/littlefs_test/test_stats";

    PersistentState ref{};
    ref.bootCount = 7;
    ref.totalEnergy = 123.5;
    ref.timeLastPower = 1700000000;
    ref.todayEnergy = 4.25f;
    uint8_t blob[sizeof(PersistentState)];
    memcpy(blob, &ref, sizeof(blob));
    memset(blob + sizeof(blob) - 4, 0xFF, 4);
    FILE *f = fopen(fn, "wb");
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL(1, fwrite(blob, sizeof(blob), 1, f));
    fclose(f);

    {
        FlashValueStore<PersistentState> st{fn};
        TEST_ASSERT_TRUE(st.load(true));
        const auto &v = st.getFlashValue();
        TEST_ASSERT_EQUAL(7, v.bootCount);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.25f, v.todayEnergy);
        TEST_ASSERT_EQUAL_FLOAT(0.f, DailyEnergyMeter::sanePinMax(v.todayPinMax));
    }

    ref.todayPinMax = 312.5f;
    {
        FlashValueStore<PersistentState> st{fn};
        TEST_ASSERT_TRUE(st.update(ref));
    }
    // gated like SolarEnergyMeter::flash: similar within 5 Wh, one write per 2 min
    FlashValueStore<PersistentState> st2{fn, [](const PersistentState &a, const PersistentState &b) {
        return std::abs(a.totalEnergy - b.totalEnergy) < 5;
    }, 1000 * 60 * 2};
    TEST_ASSERT_TRUE(st2.load(true));
    TEST_ASSERT_EQUAL_FLOAT(312.5f, DailyEnergyMeter::sanePinMax(st2.getFlashValue().todayPinMax));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.25f, st2.getFlashValue().todayEnergy);

    ref.totalEnergy += 1.0;
    TEST_ASSERT_FALSE(st2.update(ref));       // similar
    TEST_ASSERT_TRUE(st2.update(ref, true));  // forced
    ref.totalEnergy += 10.0;
    TEST_ASSERT_FALSE(st2.update(ref));       // write interval
    TEST_ASSERT_TRUE(st2.update(ref, true));
    FlashValueStore<PersistentState> st3{fn};
    TEST_ASSERT_TRUE(st3.load(true));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 134.5, st3.getFlashValue().totalEnergy);
}