#include "mock.h"
#include <cassert>
#include <iostream>

#include "../../src/math/statmath.h"

float rtol(float x, float ref, float reg = 1e-12f) {
    return abs(x - ref) / (abs(ref)+reg);
}

void assertRtol(float x, float ref, float maxRtol=1e-8f) {
    assert(rtol(x, ref) <= maxRtol);
}

int main() {
    TrapezoidalIntegrator<float, unsigned long, float> totalEnergy{
        1e-6f / 3600.f, // /us => /h
        /*maxDt*/(unsigned long) 4e6f // 4sec
    };



    assert(totalEnergy.get() == 0);
    totalEnergy.add(1.0f, 1000000);
    assertRtol(totalEnergy.get(), .5f/3600.f);
    totalEnergy.add(1.0f, 2000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f);
    totalEnergy.add(1.0f, 5000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f + 3.f/3600.f);
    totalEnergy.add(1.0f, 10000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f + 3.f/3600.f);
    totalEnergy.add(5.0f, 12000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f + 3.f/3600.f + 3.f * 2.f/3600.0f);
    totalEnergy.add(-5.0f, 14000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f + 3.f/3600.f + 3.f * 2.f/3600.0f);
    totalEnergy.add(-5.0f, 16000000);
    assertRtol(totalEnergy.get(), .5f/3600.f + 1.f/3600.f + 3.f/3600.f + 3.f * 2.f/3600.0f - 5.f * 2.f/3600.0f);

    std::cout << totalEnergy.get();

    return 0;
}
