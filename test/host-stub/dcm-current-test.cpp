// Host test for SynchronousConverter::dcmCurrent() against the vconv cycle model (LS off, body diode,
// as with an open backflow switch), and for the open-backflow high-duty guard it gates.
//
// Build & run (repo root):
//   clang++ -std=gnu++20 -O2 -fexceptions -DMOCK=1 -I test/host-stub -I src \
//       test/host-stub/dcm-current-test.cpp src/sim/vconv.cpp -o /tmp/dcm-current-test && /tmp/dcm-current-test

#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "esp_log.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define unlikely(x) (x)
#define OUTPUT 1
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t, int) {}

#include "../../src/buck.h"
#include "../../src/sim/vconv.h"

void assertPinState(uint8_t, bool, const char *, bool) {}
void UART_LOG(const char *, ...) {}

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { ++g_fail; printf("FAIL %s:%d %s  ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); putchar('\n'); } } while (0)

static constexpr uint32_t FSW = 39000;

static void initConv(SynchronousConverter &c, const char *l0) {
    ConfFile conv{}; conv.set("topo", "buck");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", l0);
    c.init(conv, board, coil);
}

static void setDuty(SynchronousConverter &c, uint16_t hs) {
    while (c.getCtrlOnPwmCnt() < hs) c.pwmPerturb((int16_t) std::min<int>(hs - c.getCtrlOnPwmCnt(), 500));
    while (c.getCtrlOnPwmCnt() > hs) c.pwmPerturb((int16_t) -std::min<int>(c.getCtrlOnPwmCnt() - hs, 500));
}

// the guard as in MpptController::protect(), backflow switch open
static bool trips(const SynchronousConverter &c, float vin, float vout) {
    return c.getDutyCycle() > 0.33f && !(c.dcmCurrent(vin, vout) < 1.f);
}

// Estimate vs plant over the duty range, PV stiff enough to hold Vin; LS off, diode Vf 0.7 V.
// kVin/kVout: sensor gain error applied to what the guard sees (the plant runs on true voltages).
static void testAgainstPlant(float voc, float vbat, float lReal, const char *l0, float kVin = 1.f, float kVout = 1.f) {
    SynchronousConverter c{}; initConv(c, l0);
    const uint16_t pm = c.pwmCtrlMax;
    int nDcm = 0, nTrip = 0;
    float worst = 0, firstTripD = NAN;
    for (int i = 1; i <= 120; ++i) {
        const uint16_t hs = (uint16_t) (pm * i / 120);
        VirtualConverter p{};
        p.setPv(40.f, voc, 0.78f); p.setBat(vbat, 0.03f); p.setPassives(470e-6f, 470e-6f, lReal);
        p.setVin(voc); p.setVout(vbat); p.setDiodeVf(0.7f);
        setDuty(c, hs);
        p.setPwm({c.pwmMaxDriver(), hs, 0, FSW});
        for (int k = 0; k < 400; ++k) p.stepSeconds(1.f / 434.f, FSW);
        const float vi = p.getVin() * kVin, vo = p.getVout() * kVout, io = p.getIoutAvg(), est = c.dcmCurrent(vi, vo);
        if (getenv("TRACE"))
            printf("D=%.3f vin=%.2f vout=%.2f plant=%.3f est=%.3f dcm=%d\n", c.dutyCtrlEff(), vi, vo, io, est, p.inDcm());
        if (std::isfinite(est) && kVin == 1.f && kVout == 1.f) {
            ++nDcm;
            // a configured L0 above the real coil under-reads by that ratio
            const float lErr = std::min(1.f, lReal / (std::strtof(l0, nullptr) * 0.95f));
            CHECK(est >= io * lErr * 0.98f - 0.01f, "under-estimate D=%.3f vin=%.1f vout=%.2f est=%.3f plant=%.3f",
                  c.getDutyCycle(), vi, vo, est, io);
            if (io > 0.05f) worst = std::max(worst, est / io);
        } else if (!std::isfinite(est)) {
            CHECK(c.dutyCtrlEff() > 0.85f * vo / vi, "NAN deep in DCM D=%.3f M=%.3f", c.dutyCtrlEff(), vo / vi);
        }
        // a current above the limit must never pass the guard (far tail included)
        // biased sensors shift the DCM estimate itself; allow that, but never a CCM escape
        const float iMax = (kVin == 1.f && kVout == 1.f) ? 1.f : 1.25f;
        if (c.getDutyCycle() > 0.33f && io > iMax) CHECK(trips(c, vi, vo), "passes at %.2f A D=%.3f", io, c.getDutyCycle());
        if (trips(c, vi, vo)) { ++nTrip; if (std::isnan(firstTripD)) firstTripD = c.getDutyCycle(); }
        else CHECK(std::isnan(firstTripD), "guard released again at D=%.3f (first trip %.3f)", c.getDutyCycle(), firstTripD);
    }
    printf("plant Voc=%.1f Vbat=%.1f L=%.0fuH kVin=%.2f kVout=%.2f: %d DCM points, est/plant max %.2f, first trip D=%.3f\n",
           voc, vbat, lReal * 1e6f, kVin, kVout, nDcm, worst, firstTripD);
}

static void testCases() {
    SynchronousConverter c{}; initConv(c, "80e-6");
    const uint16_t pm = c.pwmCtrlMax;
    // fry 11-11 low light: Vin 31, Vout 25.6, D 0.34..0.55 -> ~0.1 A, no trip
    for (float d: {0.34f, 0.45f, 0.55f}) {
        setDuty(c, (uint16_t) (d * pm));
        const float e = c.dcmCurrent(31.f, 25.6f);
        CHECK(e < 0.5f && !trips(c, 31.f, 25.6f), "fry low light D=%.2f est=%.3f", d, e);
    }
    // high Vin, duty past M: CCM, unbounded -> trip
    setDuty(c, (uint16_t) (0.45f * pm));
    CHECK(std::isnan(c.dcmCurrent(65.f, 25.6f)) && trips(c, 65.f, 25.6f), "CCM at Vin 65 must trip");
    // unusable voltages -> trip
    CHECK(trips(c, NAN, 25.6f) && trips(c, 25.f, 25.6f) && trips(c, 30.f, 0.f), "bad voltages must trip");
    // boost: no estimate
    SynchronousConverter b{};
    ConfFile conv{}; conv.set("topo", "boost");
    ConfFile board{}; board.set("pwm_freq", "39000"); board.set("pwm_driver_logic", "HiLi");
    board.set("pwm_hi", "21"); board.set("pwm_li", "14"); board.set("pwm_sd", "47"); board.set("skip_assert", "1");
    ConfFile coil{}; coil.set("L0", "80e-6");
    b.init(conv, board, coil);
    CHECK(std::isnan(b.dcmCurrent(60.f, 25.f)), "boost must return NAN");
    printf("cases: ok\n");
}

int main() {
    testCases();
    testAgainstPlant(66.f, 25.6f, 79.8e-6f, "80e-6"); // fry
    testAgainstPlant(36.f, 25.6f, 79.8e-6f, "80e-6"); // fry, low light
    testAgainstPlant(69.4f, 26.7f, 50.9e-6f, "56e-6"); // flat (L0 above the real coil)
    testAgainstPlant(69.4f, 26.7f, 50.9e-6f, "50e-6");
    // low Vin: the 1 A cap never binds below CCM, only the D < 0.95·M gate does
    testAgainstPlant(31.f, 25.6f, 79.8e-6f, "80e-6");
    testAgainstPlant(31.f, 25.6f, 79.8e-6f, "80e-6", 0.96f, 1.02f); // Vin low, Vout high: M over-read
    testAgainstPlant(36.f, 25.6f, 79.8e-6f, "80e-6", 0.96f, 1.02f);
    testAgainstPlant(31.f, 25.6f, 50.9e-6f, "56e-6", 0.96f, 1.02f); // + L0 above the real coil
    printf(g_fail ? "FAILED (%d)\n" : "all ok\n", g_fail);
    return g_fail != 0;
}
