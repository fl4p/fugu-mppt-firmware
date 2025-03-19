#include <unity.h>
#include "adc/sampling.h"

#include "adc/adc_dummy.h"

void test_LinearTransform() {
    {
        LinearTransform t0{10, 0};
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f * 10.f, t0.apply(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f * 10.f, t0.apply(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 10.f, t0.apply_inverse(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.f / 10.f, t0.apply_inverse(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, t0.apply(t0.apply_inverse(1.f)));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, t0.apply(t0.apply_inverse(2.f)));
    }

    {
        LinearTransform t1{10, 1};
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, t1.apply(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (5.f - 1) * 10.f, t1.apply(5.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 10.f + 1, t1.apply_inverse(1.f));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f, t1.apply(t1.apply_inverse(1.f)));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.f, t1.apply(t1.apply_inverse(2.f)));
    }
}

#if 0
void test_ADCSampler() {

    std::vector<std::vector<float>> samples{
            {12.4f, 12.3f, 12.2f, 12.15f},
            {0.f,   0.1f,  0.2f,  0.25f},
    };
    ADC_Dummy adc{samples};

    adc.startReading(0);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(12.4f, adc.getSample());

    adc.startReading(1);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(0.f, adc.getSample());

    adc.startReading(0);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(12.3f, adc.getSample());

    adc.startReading(1);
    TEST_ASSERT_TRUE(adc.hasData());
    TEST_ASSERT_EQUAL(0.1f, adc.getSample());

    adc.resetPointer();

    ADC_Sampler sampler{};
    sampler.setADC(&adc);
    auto &sensorU(*sampler.addSensor(0, {1, 0}, 15, {15, 1, false}, "u"));
    auto &sensorI(*sampler.addSensor(1, {2.5, 0}, 1, {1, 1, false}, "i"));
    sampler.begin(1);

    TEST_ASSERT_EQUAL(0, sensorU.adcCh);
    TEST_ASSERT_EQUAL(1, sensorI.adcCh);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 15, adc.maxExpectedVoltages[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.f / 2.5f, adc.maxExpectedVoltages[1]);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(1, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.4f, sensorU.last);
    TEST_ASSERT_EQUAL(0, sensorI.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(1, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.4f, sensorU.last);
    TEST_ASSERT_EQUAL(1, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.0f), sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(2, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.3f, sensorU.last);
    TEST_ASSERT_EQUAL(1, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.0f), sensorI.last);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_EQUAL(2, sensorU.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.3f, sensorU.last);
    TEST_ASSERT_EQUAL(2, sensorI.numSamples);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sensorI.transform.apply(.1f), sensorI.last);

    sampler.startCalibration();
    TEST_ASSERT_TRUE(sampler.isCalibrating());

    TEST_ASSERT_EQUAL(0, sensorU.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorU.last);
    TEST_ASSERT_FLOAT_WITHIN(0, 0, sensorU.calibrationAvg);
    TEST_ASSERT_EQUAL(0, sensorI.numSamples);
    TEST_ASSERT_FLOAT_IS_NAN(sensorI.last);
    TEST_ASSERT_FLOAT_WITHIN(0, 0, sensorI.calibrationAvg);


    adc.resetPointer();
    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_FALSE(sampler.update());

    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_FALSE(sampler.update());

    TEST_ASSERT_FALSE(sampler.update());
    TEST_ASSERT_TRUE(sampler.update());

    struct Med3Ewm {
        RunningMedian3<float> med3;
        EWM<float> ewm;

        Med3Ewm(int span) : ewm(span) {}

        void add(float v) { ewm.add(med3.next(v)); }
    };

    Med3Ewm ewmU{1}, ewmI{1};
    ewmU.add(samples[0][0]);
    ewmU.add(samples[0][1]);
    ewmU.add(samples[0][2]);
    ewmI.add(sensorI.transform.apply(samples[1][0]));
    ewmI.add(sensorI.transform.apply(samples[1][1]));
    ewmI.add(sensorI.transform.apply(samples[1][2]));

    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmU.ewm.avg.get(), sensorU.calibrationAvg);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmI.ewm.avg.get(), sensorI.calibrationAvg);

    TEST_ASSERT_TRUE(sampler.update());
    TEST_ASSERT_TRUE(sampler.update());

    ewmU.add(samples[0][3]);
    ewmI.add(sensorI.transform.apply(samples[1][3]));

    TEST_ASSERT_FLOAT_WITHIN(1e-6, ewmI.ewm.std.get(), sensorI.ewm.std.get());
}
#endif