// Host test for the fixed-reference Iout current-controller normalization: src/pd_control.h
// (PD_Control_SmoothSetpoint's normalize flag), src/mppt.h (IoutCtrlRefA), src/mppt.cpp (the Iout CVP).
//
// Unit boundary: this exercises the PD_Control normalization law through the production constructor
// (PD_Control_SmoothSetpoint(Kp, Kd, 200, false)) with actual and target divided by IoutCtrlRefA=40.
// This test does not instantiate MPPT or assert which limiter wins; simulator runs exercise integration.
//
// Build & run:
//   test/host-stub/host-tests.sh pd-control-test

#include <cmath>
#include <cstdio>
#ifndef likely
#define likely(x) (x)
#endif
#ifndef unlikely
#define unlikely(x) (x)
#endif
#include "math/statmath.h" // EWMA, used by pd_control.h
#include "pd_control.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); ++fails; } } while (0)

static constexpr float REF = 40.f; // MpptController::IoutCtrlRefA

// production wiring (mppt.cpp): normalize=false, both actual and target divided by the fixed reference
static float fixedResp(float actualA, float targetA) {
    PD_Control_SmoothSetpoint c(200.f, 400.f, 200, false);
    return c.update(actualA / REF, targetA / REF, 0.f);
}

int main() {
    // 1) moving-target exact sequence, span 3 (alpha 0.5), actual 10 A constant, targets 20/40/20 A:
    //    smoothed 20/30/25 A -> errors 0.25/0.5/0.375 -> cv 50/200/25 (Kp 200, Kd 400, first D 0)
    {
        PD_Control_SmoothSetpoint c(200.f, 400.f, 3, false);
        const float tgt[3] = {20.f, 40.f, 20.f};
        const float want[3] = {50.f, 200.f, 25.f};
        for (int i = 0; i < 3; ++i) {
            const float cv = c.update(10.f / REF, tgt[i] / REF, 0.f);
            CHECK(std::fabs(cv - want[i]) < 1e-3f, "moving seq step %d (tgt %.0f A): cv %.4f != %.4f", i, tgt[i], cv,
                  want[i]);
        }
    }

    // 2) constant 40 A exact sequence (D-term equivalence): errors 0.025/0.05/0/-0.025 -> cv 5/20/-20/-15
    {
        PD_Control_SmoothSetpoint c(200.f, 400.f, 200, false);
        const float act[4] = {39.f, 38.f, 40.f, 41.f};
        const float want[4] = {5.f, 20.f, -20.f, -15.f};
        for (int i = 0; i < 4; ++i) {
            const float cv = c.update(act[i] / REF, 40.f / REF, 0.f);
            CHECK(std::fabs(cv - want[i]) < 1e-4f, "const seq step %d (act %.0f A): cv %.5f != %.5f", i, act[i], cv,
                  want[i]);
        }
    }

    // 3) the same 1 A error gives the same response at 1/4/40 A targets (fixed reference)
    for (float t: {1.f, 4.f, 40.f}) {
        const float cv = fixedResp(t - 1.f, t);
        CHECK(std::fabs(cv - 200.f / REF) < 1e-3f, "fixed 1 A error at %.0f A: cv %.4f != %.4f", t, cv, 200.f / REF);
    }

    // 4) at a settled 40 A target the fixed law agrees the old relative law
    PD_Control rel(200.f, 400.f, true);
    CHECK(std::fabs(fixedResp(39.f, 40.f) - rel.update(39.f, 40.f, 0.f)) < 1e-3f, "40 A: fixed %.4f == relative %.4f",
          fixedResp(39.f, 40.f), rel.update(39.f, 40.f, 0.f));

    // 5) zero target: the fixed law is finite and still responds
    const float fz = fixedResp(5.f, 0.f);
    CHECK(std::isfinite(fz) && fz < 0.f, "fixed zero target: cv %.4f finite negative", fz);

    // 6) other controllers keep the relative default (normalize omitted = true)
    PD_Control_SmoothSetpoint power(20, 5, 4); // default normalize=true
    CHECK(std::fabs(power.update(30.f, 60.f, 0.f) - 20.f * 0.5f) < 1e-3f, "power controller default stays relative");

    // 7) limiter sign/past semantics: over the limit cv < 0 and Kp*_prevE < 0 ("past"); under it, both > 0
    PD_Control io(200, 400, false);
    const float over = io.update(45.f / REF, 40.f / REF, 0.f);
    CHECK(over < 0.f && 200.f * io._prevE < 0.f, "over-limit: cv %.4f < 0, prevE %.4f past", over, io._prevE);
    PD_Control iu(200, 400, false);
    const float under = iu.update(35.f / REF, 40.f / REF, 0.f);
    CHECK(under > 0.f && 200.f * iu._prevE > 0.f, "under-limit: cv %.4f > 0, prevE %.4f not past", under, iu._prevE);

    std::printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails ? 1 : 0;
}
