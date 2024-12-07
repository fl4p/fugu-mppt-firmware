#include "mock.h"

#include "../../src/buck.h"

int main() {
    SynchronousConverter conv{};
    conv.init(ConfFile{"test/host-stub/pins.conf"}, 50e-6);

    conv.updateSyncRectMaxDuty(60, 30, 1);
    assert(conv.inDCM());
    assert(abs(conv.rippleCurrent(60,30) - 8) < 1);

    conv.updateSyncRectMaxDuty(60, 30, 4);
    assert(conv.inDCM());

    conv.updateSyncRectMaxDuty(60, 30, 5);
    assert(!conv.inDCM());

    conv.updateSyncRectMaxDuty(60, 30, 10);
    assert(!conv.inDCM());

    float iout = 0.1;
    while(iout < 30) {
        conv.updateSyncRectMaxDuty(60,30, iout);
        iout *= 1.1;
    }
}