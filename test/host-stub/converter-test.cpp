// Host test for SynchronousConverter's ripple current and CCM/DCM decision (buck, 60 V -> 30 V, 50 uH, 39 kHz).
//   clang++ -std=gnu++20 -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       -o /tmp/converter-test test/host-stub/converter-test.cpp && /tmp/converter-test
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "esp_log.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define unlikely(x) (x)
#define OUTPUT 1
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}

#include "../../src/buck.h"

void assertPinState(uint8_t, bool, const char *, bool) {}
void UART_LOG(const char *, ...) {}

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { ++g_fail; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

int main() {
    ConfFile conv{}; conv.set("topo", "buck");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", "50e-6");
    SynchronousConverter c{};
    c.init(conv, board, coil);

    // dI = Vout/(f L) (1 - Vout/Vin), L = 0.95 L0
    const float ir = c.rippleCurrent(60, 30);
    printf("ripple %.2f A p-p\n", ir);
    CHECK(std::fabs(ir - 30.f / (39000 * 50e-6f * 0.95f) * 0.5f) < 0.01f);

    // DCM below ripple/2 (enter), CCM above ripple/1.8 (exit)
    CHECK(c.computeDCM(60, 30, 1));
    CHECK(c.computeDCM(60, 30, 4));
    CHECK(!c.computeDCM(60, 30, 5));
    CHECK(!c.computeDCM(60, 30, 10));
    CHECK(!c.computeDCM(60, 30, 4.3f)); // hysteresis: stays CCM between 4.05 and 4.5 A
    CHECK(c.computeDCM(60, 30, 3.9f));
    CHECK(c.computeDCM(60, 30, 4.3f)); // and DCM there coming from below
    CHECK(c.computeDCM(60, 30, 0.05f));

    for (float iout = 0.1f; iout < 30; iout *= 1.1f) c.updateSyncRectMaxDuty(60, 30, iout);
    CHECK(!c.inDCM());

    printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
    return g_fail != 0;
}
