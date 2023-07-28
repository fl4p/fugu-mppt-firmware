#include <unity.h>
//#include <task.h>
#include <Arduino.h>

//#warning "Test main"

void test_meter();

void test_meter_storage();

void test_LinearTransform();

void test_ADCSampler();

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_meter);
    RUN_TEST(test_meter_storage);

    RUN_TEST(test_LinearTransform);
    RUN_TEST(test_ADCSampler);

    /**
     * TODO
     *  math/statmath stuff (EWM, med3)
     *  float16
     */

    UNITY_END();
}

void loop() {
    //vTaskDelay(5);
    delay(10);
    //delay(1);
}