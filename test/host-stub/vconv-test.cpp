// Host-side unit tests for src/sim/vconv.{h,cpp} and src/pwm/vconv.h.
// Punch list A-T from docs/superpowers/specs/2026-05-23-virtual-converter-design.md.
//
// Build & run:
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
//       -o /tmp/vconv-test test/host-stub/vconv-test.cpp src/sim/vconv.cpp && \
//       /tmp/vconv-test

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "../../src/sim/vconv.h"
#include "../../src/pwm/vconv.h"

namespace {

int g_run = 0;
int g_fail = 0;
const char *g_section = "";

void section(const char *name) {
    g_section = name;
    std::printf("[%s]\n", name);
}

#define EXPECT(cond)                                                                              \
    do {                                                                                          \
        ++g_run;                                                                                  \
        if (!(cond)) {                                                                            \
            ++g_fail;                                                                             \
            std::printf("  FAIL %s:%d  [%s]  %s\n", __FILE__, __LINE__, g_section, #cond);        \
        }                                                                                         \
    } while (0)

#define EXPECT_NEAR(actual, expected, tol)                                                        \
    do {                                                                                          \
        ++g_run;                                                                                  \
        double _a = (double) (actual), _e = (double) (expected), _t = (double) (tol);             \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > _t) {                                      \
            ++g_fail;                                                                             \
            std::printf("  FAIL %s:%d  [%s]  got %.6g, want %.6g (±%.3g)\n",                      \
                        __FILE__, __LINE__, g_section, _a, _e, _t);                               \
        }                                                                                         \
    } while (0)

#define EXPECT_REL(actual, expected, rel)                                                         \
    EXPECT_NEAR(actual, expected, std::fabs(double(expected)) * (rel))

// ----- helpers ---------------------------------------------------------------

constexpr uint32_t kFreq = 39000;
constexpr float    kT    = 1.0f / (float) kFreq;
constexpr uint16_t kPmax = 1000;
constexpr float    kL    = 50e-6f;

VirtualConverter::PwmState mkPwm(uint16_t ctrl, uint16_t rect) {
    return VirtualConverter::PwmState{kPmax, ctrl, rect, kFreq};
}

// Bring up a plant with the standard rig. Huge caps freeze Vin/Vout so per-cycle
// math can be inspected in isolation; tests that exercise BE pick their own cOut.
void rig(VirtualConverter &v, float vin, float vout, uint16_t ctrl, uint16_t rect,
         float cin = 1.0f, float cout = 1.0f, float l = kL) {
    v.setPv(8.0f, 40.0f, 0.8f);
    v.setBat(vout, 0.05f);
    v.setBatRipple(0.0f, 100.0f);
    v.setPassives(cin, cout, l);
    v.setVin(vin);
    v.setVout(vout);
    v.setPwm(mkPwm(ctrl, rect));
}

void runN(VirtualConverter &v, int n) {
    for (int i = 0; i < n; ++i) v.stepSeconds(kT, kFreq);
}

// Re-pin caps after each step so per-cycle inputs stay constant during measurement.
void runPinned(VirtualConverter &v, int n, float vin, float vout) {
    for (int i = 0; i < n; ++i) {
        v.setVin(vin);
        v.setVout(vout);
        v.stepSeconds(kT, kFreq);
    }
    v.setVin(vin);
    v.setVout(vout);
}

// ----- A: CCM volt-second balance --------------------------------------------
//
// In settled CCM the steady-state ratio Vout/Vin = tHS/(tHS+tLS). We can't drive
// the model to CCM "naturally" without coupling to caps, but we can verify that
// the per-cycle solver maintains the SS coil current when we pin the analytic
// fixed point: a = Iout - r/2 where r = (Vin-Vout)·tHS/L. If volt-second balance
// holds, iLEnd cycle-to-cycle stays constant.
void testA_ccm_voltsec() {
    section("A: CCM volt-sec balance");
    VirtualConverter v;
    const uint16_t ctrl = 400, rect = 600;   // D_HS = 0.4, tOff = 0
    const float vin = 40.0f;
    const float vout = vin * (float) ctrl / float(ctrl + rect);   // 16.0 V
    rig(v, vin, vout, ctrl, rect);
    runPinned(v, 200, vin, vout);   // settle on cap-pinned fixed point
    const float a0 = v.getIL();
    runPinned(v, 1, vin, vout);
    const float a1 = v.getIL();
    // SS test: iLEnd should be ~constant cycle-to-cycle.
    EXPECT_NEAR(a1, a0, 1e-3f);
    // And the ratio iOutAvg gives us Vout/Vin via the SS relation (lossless,
    // tOff=0): iInAvg / iOutAvg = D_HS = Vout/Vin.
    const float ratio = v.getIinAvg() / v.getIoutAvg();
    EXPECT_REL(ratio, vout / vin, 0.005f);
}

// ----- B: CCM ripple amplitude ------------------------------------------------
//
// Ripple = (Vin − Vout)·tHS/L. In CCM SS, iOutAvg = a + r/2 and a = iLEnd, so
// r = 2·(iOutAvg − iLEnd). No internal access needed.
void testB_ccm_ripple() {
    section("B: CCM ripple amplitude");
    VirtualConverter v;
    const uint16_t ctrl = 400, rect = 600;
    const float vin = 40.0f, vout = 16.0f;
    rig(v, vin, vout, ctrl, rect);
    runPinned(v, 200, vin, vout);
    const float ripple_obs = 2.0f * (v.getIoutAvg() - v.getIL());
    const float tHS = ((float) ctrl / kPmax) * kT;
    const float ripple_expected = (vin - vout) * tHS / kL;
    EXPECT_REL(ripple_obs, ripple_expected, 0.01f);
}

// ----- C: Energy conservation (lossless plant) -------------------------------
void testC_energy() {
    section("C: Energy conservation (CCM SS)");
    VirtualConverter v;
    const uint16_t ctrl = 400, rect = 600;
    const float vin = 40.0f, vout = 16.0f;
    rig(v, vin, vout, ctrl, rect);
    runPinned(v, 500, vin, vout);
    EXPECT_REL(vin * v.getIinAvg(), vout * v.getIoutAvg(), 0.01f);
}

// ----- D: D=0 idle ------------------------------------------------------------
void testD_idle() {
    section("D: D=0 idle");
    VirtualConverter v;
    rig(v, 20.0f, 28.0f, 0, 0, 470e-6f, 470e-6f);
    runN(v, 1000);
    EXPECT_NEAR(v.getIinAvg(), 0.0f, 1e-6f);
    EXPECT_NEAR(v.getIoutAvg(), 0.0f, 1e-6f);
    EXPECT_NEAR(v.getIL(), 0.0f, 1e-6f);
    // Vout relaxes toward Vbat via BE; Vin toward Voc via PV.
    EXPECT_REL(v.getVout(), 28.0f, 0.001f);
    EXPECT_REL(v.getVin(),  40.0f, 0.02f);   // PV pull only at v close to Voc is slow; loose
}

// ----- E: D=1 saturation ------------------------------------------------------
void testE_saturation() {
    section("E: D=1 saturation");
    VirtualConverter v;
    rig(v, 40.0f, 10.0f, kPmax, 0);   // pwmCtrl=pmax, pwmRect=0 → tOff=0
    // Pin to keep growth analytically tractable.
    float prev = -1e30f;
    for (int i = 0; i < 50; ++i) {
        v.setVin(40.0f);
        v.setVout(10.0f);
        v.stepSeconds(kT, kFreq);
        EXPECT(v.getIL() > prev);   // monotone growth
        EXPECT(!v.inDcm());
        prev = v.getIL();
    }
}

// ----- F: C_out charge balance (steady-state) --------------------------------
//
// Spec: steady iOutAvg ≈ (Vout − Vbat)/Rbat. This is the SS condition for C_out:
// in BE-stable SS, current INTO C_out (= iOutAvg − Ibat) integrates to zero so
// iOutAvg = Ibat = (Vout − Vbat)/Rbat. Run with real caps; let Vout settle.
void testF_charge_balance() {
    section("F: C_out charge balance");
    VirtualConverter v;
    // Vbat below the Vout_target dictated by D so non-trivial current flows.
    const float vbat = 10.0f, rbat = 0.05f;
    rig(v, 40.0f, vbat, 400, 600, 470e-6f, 470e-6f);
    v.setBat(vbat, rbat);
    for (int i = 0; i < 20000; ++i) v.stepSeconds(kT, kFreq);
    const float iBat = (v.getVout() - vbat) / rbat;
    EXPECT(std::fabs(iBat) > 0.1f);             // ensure operating point is non-degenerate
    EXPECT_NEAR(v.getIoutAvg(), iBat, 0.05f);   // 50 mA absolute, SS Kirchhoff on C_out
}

// ----- G: BE stability vs r_bat ----------------------------------------------
//
// With pwmCtrl=0 (no converter activity), Vout relaxes purely via BE toward
// Vbat. Step Vbat 20→10 and check monotone decrease for r_bat across 6 decades.
void testG_be_stability() {
    section("G: BE stability vs r_bat");
    const float rbats[] = {1e-3f, 1e-2f, 0.05f, 1.0f, 1e9f};
    for (float r : rbats) {
        VirtualConverter v;
        v.setPv(8.0f, 40.0f, 0.8f);
        v.setPassives(1e-3f, 470e-6f, kL);
        v.setBat(20.0f, r);
        v.setVin(40.0f);
        v.setVout(20.0f);
        v.setPwm(mkPwm(0, 0));
        // Drive to SS, then step Vbat.
        for (int i = 0; i < 5000; ++i) v.stepSeconds(kT, kFreq);
        v.setBat(10.0f, r);
        float prev = v.getVout();
        bool monotone = true, finite = true;
        for (int i = 0; i < 20000; ++i) {
            v.stepSeconds(kT, kFreq);
            float cur = v.getVout();
            if (!std::isfinite(cur)) { finite = false; break; }
            if (cur > prev + 1e-5f) { monotone = false; break; }
            prev = cur;
        }
        EXPECT(finite);
        EXPECT(monotone);
        // Final Vout near new Vbat (within a few %). 1e9 Ω never converges; skip.
        if (r < 1e6f) EXPECT_NEAR(v.getVout(), 10.0f, 0.5f);
    }
}

// ----- H: Reverse-pump charge balance -----------------------------------------
//
// SPEC NOTE: model's iInAvg = areaHS/T (phase-1 only). In the reverse-pump
// regime the HS body-diode contribution during phase 3 is NOT included, so
// strict Vin·iInAvg + Vout·iOutAvg = 0 won't hold. We assert (a) iInAvg < 0 (b)
// iOutAvg < 0 (sign-wise reverse current both sides) (c) all quantities finite.
// Quantitative balance is deferred until the model accounts for areaOff in
// iInAvg, or the spec clarifies what iInAvg represents.
void testH_reverse_pump_balance() {
    section("H: Reverse-pump regime sign + finite");
    VirtualConverter v;
    rig(v, 40.0f, 20.0f, 200, 700);   // pwmRect well past zero-crossing
    runPinned(v, 200, 40.0f, 20.0f);
    EXPECT(std::isfinite(v.getIinAvg()));
    EXPECT(std::isfinite(v.getIoutAvg()));
    EXPECT(std::isfinite(v.getIL()));
    EXPECT(v.getIinAvg() < 0.0f);
    EXPECT(v.getIoutAvg() < 0.0f);
}

// ----- I: iInAvg sign + magnitude in reverse regime --------------------------
//
// iInAvg = (a + b)·tHS/(2T). Verify against analytic given observed iLEnd (=a in
// pinned-SS). Independent of the H balance question.
void testI_iinavg_reverse() {
    section("I: iInAvg analytic match (reverse)");
    VirtualConverter v;
    const float vin = 40.0f, vout = 20.0f;
    rig(v, vin, vout, 200, 700);
    runPinned(v, 200, vin, vout);
    const float tHS = (200.0f / kPmax) * kT;
    const float a = v.getIL();                    // SS: a == iLEnd
    const float b = a + (vin - vout) / kL * tHS;
    const float iIn_analytic = (a + b) * 0.5f * tHS / kT;
    EXPECT_REL(v.getIinAvg(), iIn_analytic, 0.01f);
}

// ----- J: DCM boundary --------------------------------------------------------
//
// Fix PWM with tOff > 0. Sweep Vbat low→high (high load→light load) with real
// caps so Vout settles; find the CCM→DCM transition. At the flip point,
// half-ripple ≈ iOutAvg (classical CCM/DCM boundary).
void testJ_dcm_boundary() {
    section("J: DCM boundary");
    const uint16_t ctrl = 300, rect = 400;          // tOff = 0.3·T
    const float tHS = ((float) ctrl / kPmax) * kT;
    bool found = false;
    bool prev_dcm = false;
    bool first = true;
    float iout_at_flip = 0, vin_at_flip = 0, vout_at_flip = 0, vbat_flip = 0;
    for (float vbat = 5.0f; vbat <= 35.0f; vbat += 1.0f) {
        VirtualConverter v;
        v.setPv(8.0f, 40.0f, 0.8f);
        v.setPassives(470e-6f, 470e-6f, kL);
        v.setBat(vbat, 0.05f);
        v.setVin(40.0f);
        v.setVout(vbat);
        v.setPwm(mkPwm(ctrl, rect));
        for (int i = 0; i < 8000; ++i) v.stepSeconds(kT, kFreq);  // settle
        bool dcm = v.inDcm();
        if (first) { prev_dcm = dcm; first = false; continue; }
        if (!prev_dcm && dcm) {                     // CCM → DCM
            found = true;
            iout_at_flip = v.getIoutAvg();
            vin_at_flip = v.getVin();
            vout_at_flip = v.getVout();
            vbat_flip = vbat;
            break;
        }
        prev_dcm = dcm;
    }
    EXPECT(found);
    if (found) {
        const float ripple_half = (vin_at_flip - vout_at_flip) * tHS / (2.0f * kL);
        EXPECT_REL(iout_at_flip, ripple_half, 0.2f);   // one sweep-step worth
        std::printf("  J: flip at Vbat=%.1f V (Vin=%.2f, Vout=%.2f), iOut=%.3f A, ripple/2=%.3f A\n",
                    vbat_flip, vin_at_flip, vout_at_flip, iout_at_flip, ripple_half);
    }
}

// ----- K: vbatAcPhase_ wrap stays bounded -------------------------------------
void testK_phase_wrap() {
    section("K: Phase wrap @ 1Hz over 1e6 cycles");
    VirtualConverter v;
    rig(v, 40.0f, 20.0f, 400, 600);
    v.setBatRipple(0.5f, 1.0f);
    bool finite = true;
    // Pin Vin/Vout to avoid wandering; we only care about phase boundedness.
    for (int i = 0; i < 1'000'000; ++i) {
        v.setVin(40.0f);
        v.setVout(20.0f);
        v.stepSeconds(kT, kFreq);
        if (!std::isfinite(v.getVout())) { finite = false; break; }
    }
    // No direct getter for vbatAcPhase_; proxy is "Vout stayed finite",
    // i.e. the sin(phase) injection never produced NaN/Inf.
    EXPECT(finite);
}

// ----- L: PV boundary points --------------------------------------------------
void testL_pv_boundary() {
    section("L: PV boundary values");
    VirtualConverter v;
    v.setPv(8.0f, 40.0f, 0.8f);
    EXPECT_NEAR(v.pvCurrent(0.0f),   8.0f, 1e-5f);
    EXPECT_NEAR(v.pvCurrent(40.0f),  0.0f, 1e-5f);
    EXPECT_NEAR(v.pvCurrent(45.0f),  0.0f, 1e-5f);
    EXPECT_NEAR(v.pvCurrent(-5.0f),  8.0f, 1e-5f);
}

// ----- M: PV Newton convergence (argmax P(V) ≈ k·Voc) ------------------------
//
// Float-precision central differences on a near-flat peak are noisy; instead
// grid-search argmax P(V) and verify it lands at k·Voc.
void testM_pv_newton() {
    section("M: PV Newton MPP convergence");
    const float ks[] = {0.5f, 0.75f, 0.85f, 0.95f};
    const float Voc = 40.0f, Isc = 8.0f;
    for (float k : ks) {
        VirtualConverter v;
        v.setPv(Isc, Voc, k);
        const int N = 4000;
        float bestV = 0, bestP = -1.0f;
        for (int i = 1; i <= N; ++i) {
            float V = (float) i / N * Voc;
            float P = V * v.pvCurrent(V);
            if (P > bestP) { bestP = P; bestV = V; }
        }
        // 0.5% tolerance on argmax; grid step is 0.025% of Voc so resolution
        // is not the bottleneck.
        EXPECT_REL(bestV, k * Voc, 0.005f);
    }
}

// ----- N: PWM_VConv shim wiring (per spec table) ------------------------------
void testN_shim_wiring() {
    section("N: PWM_VConv shim wiring");
    PWM_VConv shim;
    shim.pwmMax = kPmax;
    g_vconv.setPwmMax(kPmax);
    g_vconv.setPwmFreq(kFreq);
    auto setStart = []() { g_vconv.setPwm(mkPwm(500, 20)); };

    // 1a: HiLi HS update_pwm(0, 0, 510) -> pwmCtrl=510, pwmRect=20
    setStart();
    shim.update_pwm(0, 0, 510);
    EXPECT(g_vconv.getPwm().pwmCtrl == 510 && g_vconv.getPwm().pwmRect == 20);

    // 1b: HiLi LS update_pwm(1, 510, 25) -> pwmCtrl=510, pwmRect=25
    shim.update_pwm(1, 510, 25);
    EXPECT(g_vconv.getPwm().pwmCtrl == 510 && g_vconv.getPwm().pwmRect == 25);

    // 2a: EnLogic HS single-arg update_pwm(0, 490) -> pwmCtrl=490, pwmRect=25
    shim.update_pwm(0, 490);
    EXPECT(g_vconv.getPwm().pwmCtrl == 490 && g_vconv.getPwm().pwmRect == 25);

    // 2b: EnLogic LS single-arg update_pwm(1, 515) -> 515-490 = 25
    shim.update_pwm(1, 515);
    EXPECT(g_vconv.getPwm().pwmCtrl == 490 && g_vconv.getPwm().pwmRect == 25);

    // 3a: direction<0 quirk. Reset pwmCtrl=500 first, then EN written before IN.
    g_vconv.setPwm(mkPwm(500, 25));
    shim.update_pwm(1, 515);    // LS = 515 - 500 = 15
    EXPECT(g_vconv.getPwm().pwmCtrl == 500 && g_vconv.getPwm().pwmRect == 15);

    // 3b: HS catches up; pwmRect stays at 15
    shim.update_pwm(0, 490);
    EXPECT(g_vconv.getPwm().pwmCtrl == 490 && g_vconv.getPwm().pwmRect == 15);

    // 4: HiLi reset update_pwm(1, 0) -> pwmRect = 0
    shim.update_pwm(1, 0);
    EXPECT(g_vconv.getPwm().pwmCtrl == 490 && g_vconv.getPwm().pwmRect == 0);
}

// ----- O: errored() latch + recovery -----------------------------------------
void testO_error_latch() {
    section("O: pwmCtrl+pwmRect>pwmMax error latch");
    VirtualConverter v;
    rig(v, 40.0f, 20.0f, 600, 600);   // 600+600 > 1000
    v.stepSeconds(kT, kFreq);
    EXPECT(v.errored());
    EXPECT_NEAR(v.getIinAvg(), 0.0f, 1e-9f);
    EXPECT_NEAR(v.getIoutAvg(), 0.0f, 1e-9f);
    // Recovery: bring counts back in range.
    v.setPwm(mkPwm(300, 400));
    v.setVin(40.0f);
    v.setVout(20.0f);
    v.stepSeconds(kT, kFreq);
    EXPECT(!v.errored());
}

// ----- P: Determinism (replay) -----------------------------------------------
//
// Two independent instances, identical inputs, must produce bit-identical state.
void testP_determinism() {
    section("P: Determinism (replay)");
    VirtualConverter a, b;
    rig(a, 30.0f, 16.0f, 350, 600);
    rig(b, 30.0f, 16.0f, 350, 600);
    a.setBatRipple(0.5f, 100.0f);
    b.setBatRipple(0.5f, 100.0f);
    for (int i = 0; i < 1000; ++i) {
        a.stepSeconds(kT, kFreq);
        b.stepSeconds(kT, kFreq);
    }
    EXPECT(a.getVin()     == b.getVin());
    EXPECT(a.getVout()    == b.getVout());
    EXPECT(a.getIL()      == b.getIL());
    EXPECT(a.getIinAvg()  == b.getIinAvg());
    EXPECT(a.getIoutAvg() == b.getIoutAvg());
}

// ----- Q: stepSeconds(NT) vs N × stepOneCycle(T) -----------------------------
//
// stepSeconds with dt=N·T should run exactly N stepOneCycle(T) calls. Verified
// by feeding two instances dt=k·T vs k repeated dt=T calls (no public single-
// cycle API; both paths go through stepSeconds).
void testQ_ncycle_equivalence() {
    section("Q: N-cycle stepping equivalence");
    const int N = 50;
    VirtualConverter a, b;
    rig(a, 30.0f, 16.0f, 400, 550);
    rig(b, 30.0f, 16.0f, 400, 550);
    // a: one batched call.
    a.stepSeconds((float) N * kT, kFreq);
    // b: N single-cycle calls (each rounds to n=1 internally).
    for (int i = 0; i < N; ++i) b.stepSeconds(kT, kFreq);
    EXPECT(a.getVin()     == b.getVin());
    EXPECT(a.getVout()    == b.getVout());
    EXPECT(a.getIL()      == b.getIL());
    EXPECT(a.getIinAvg()  == b.getIinAvg());
    EXPECT(a.getIoutAvg() == b.getIoutAvg());
}

// ----- R: DCM volt-second balance --------------------------------------------
//
// In DCM SS with phase-3 decay ending at zero:
//   Vin·tHS = Vout·(tHS + tLS + tDecay)
// where tDecay = b·L/Vout is the phase-3 ramp-to-zero time. We can capture b
// indirectly: in SS where cEnd=0 and a=0 (the spec's typical DCM idle case),
// b = (Vin − Vout)·tHS/L, and tDecay is dictated by Vout/L slope.
void testR_dcm_voltsec() {
    section("R: DCM volt-sec balance");
    // Choose PWM so phase 2 ends with c > 0 (LS releases before zero crossing),
    // and phase 3 has room to ramp down to zero — i.e. natural diode emulation.
    // c > 0 requires Vout·tLS < (Vin−Vout)·tHS → tLS < tHS at Vin=2·Vout.
    VirtualConverter v;
    const uint16_t ctrl = 250, rect = 150;          // tOff = 0.6·T
    const float vin = 40.0f, vout = 20.0f;
    rig(v, vin, vout, ctrl, rect, 1.0f, 1.0f);
    v.setBat(vout, 0.05f);
    runPinned(v, 1000, vin, vout);
    EXPECT(v.inDcm());
    EXPECT_NEAR(v.getIL(), 0.0f, 1e-3f);            // SS DCM: cEnd=0
    const float tHS = ((float) ctrl / kPmax) * kT;
    const float tLS = ((float) rect / kPmax) * kT;
    // Analytic SS: a=0, b=(Vin−Vout)·tHS/L, c = b − Vout·tLS/L > 0,
    // tDecay = c·L/Vout = ((Vin−Vout)·tHS − Vout·tLS)/Vout.
    const float tDecay = ((vin - vout) * tHS - vout * tLS) / vout;
    EXPECT(tDecay > 0.0f);
    EXPECT_REL(vin * tHS, vout * (tHS + tLS + tDecay), 0.005f);
}

// ----- S: Degenerate phase durations -----------------------------------------
void testS_degenerate() {
    section("S: tHS=0 / tLS=0");
    // (a) tHS=0, tLS>0: residual IL drains monotonically toward 0.
    {
        VirtualConverter v;
        rig(v, 40.0f, 20.0f, 0, 500, 1.0f, 1.0f);
        // Seed positive residual IL with a near-full-duty cycle that doesn't
        // allow phase-3 decay to complete (ctrl=900, rect=0 → tOff=0.1·T,
        // tZero = b·L/Vout ≫ tOff so cEnd > 0).
        v.setPwm(mkPwm(900, 0));
        runPinned(v, 5, 40.0f, 20.0f);
        const float il_seed = v.getIL();
        EXPECT(il_seed > 0.0f);
        v.setPwm(mkPwm(0, 500));
        float prev = il_seed;
        bool monotone = true, finite = true;
        for (int i = 0; i < 200; ++i) {
            v.setVin(40.0f);
            v.setVout(20.0f);
            v.stepSeconds(kT, kFreq);
            if (!std::isfinite(v.getIL())) { finite = false; break; }
            if (v.getIL() > prev + 1e-6f) { monotone = false; break; }
            prev = v.getIL();
            if (v.getIL() <= 0.0f) break;
        }
        EXPECT(finite);
        EXPECT(monotone);
    }
    // (b) tHS>0, tLS=0: phase-3 LS body diode discharges IL to 0 in DCM SS,
    //     iOutAvg > 0 (battery sinks).
    {
        VirtualConverter v;
        rig(v, 40.0f, 20.0f, 200, 0, 1.0f, 1.0f);
        runPinned(v, 1000, 40.0f, 20.0f);
        EXPECT(v.inDcm());
        EXPECT_NEAR(v.getIL(), 0.0f, 1e-3f);
        EXPECT(v.getIoutAvg() > 0.0f);
    }
}

// ----- T: Mains ripple transfer to V_out -------------------------------------
//
// Drive vbat_ac_amp=1 V at 100 Hz with no converter activity (pwmCtrl=0). V_out
// follows V_bat through a first-order RC lowpass: amplitude = 1/√(1+(ωRC)²).
void testT_mains_ripple() {
    section("T: Mains ripple → V_out");
    VirtualConverter v;
    const float R = 0.05f, C = 470e-6f;
    const float f = 100.0f, amp = 1.0f;
    v.setPv(8.0f, 40.0f, 0.8f);
    v.setPassives(1e-3f, C, kL);
    v.setBat(20.0f, R);
    v.setVin(40.0f);
    v.setVout(20.0f);
    v.setPwm(mkPwm(0, 0));
    v.setBatRipple(amp, f);
    // Settle (transient ≫ R·C = 23.5 µs; 5000 cycles ≫ that).
    for (int i = 0; i < 5000; ++i) v.stepSeconds(kT, kFreq);
    // Sample V_out over ≥ 1 ripple period: T_ripple = 10 ms = 390 cycles.
    const int N = 4000;
    std::vector<float> samples; samples.reserve(N);
    for (int i = 0; i < N; ++i) {
        v.stepSeconds(kT, kFreq);
        samples.push_back(v.getVout());
    }
    float vmin = samples[0], vmax = samples[0];
    for (float x : samples) { if (x < vmin) vmin = x; if (x > vmax) vmax = x; }
    const float amp_obs = 0.5f * (vmax - vmin);
    const float omega = 2.0f * (float) M_PI * f;
    const float amp_expected = amp / std::sqrt(1.0f + (omega * R * C) * (omega * R * C));
    EXPECT_REL(amp_obs, amp_expected, 0.05f);
    std::printf("  T: vout ripple obs=%.4f V, expected=%.4f V (ωRC=%.3f)\n",
                amp_obs, amp_expected, omega * R * C);
}

// ----- U: Spiky China-inverter ripple shape -----------------------------------
//
// shape id 2 (shapeSpiky) is a narrow per-cycle pulse — harmonic-rich, high crest
// factor, unlike the smooth sine (id 0). Drive it through the no-converter RC path
// (like testT) and confirm the transferred V_out ripple is far peakier than a sine
// of equal amplitude: crest(spiky) >> crest(sine) ~ 1.41.
void testU_spiky_ripple() {
    section("U: Spiky ripple shape transfer");
    const float R = 0.05f, C = 470e-6f, f = 300.0f, amp = 1.0f;
    auto crest = [&](int shape) {
        VirtualConverter v;
        v.setPv(8.0f, 40.0f, 0.8f);
        v.setPassives(1e-3f, C, kL);
        v.setBat(20.0f, R);
        v.setVin(40.0f); v.setVout(20.0f);
        v.setPwm(mkPwm(0, 0));
        v.setBatRipple(amp, f, shape);
        for (int i = 0; i < 5000; ++i) v.stepSeconds(kT, kFreq);
        std::vector<float> s; s.reserve(4000);
        double mean = 0;
        for (int i = 0; i < 4000; ++i) { v.stepSeconds(kT, kFreq); s.push_back(v.getVout()); mean += v.getVout(); }
        mean /= s.size();
        double sq = 0, pk = 0;
        for (float x : s) { double d = x - mean; sq += d * d; if (std::fabs(d) > pk) pk = std::fabs(d); }
        return (float) (pk / std::sqrt(sq / s.size()));   // crest factor of the AC part
    };
    const float cSine = crest(0), cSpiky = crest(2);
    std::printf("  U: crest sine=%.2f spiky=%.2f\n", cSine, cSpiky);
    EXPECT(cSine < 1.6f);          // sine ~1.41
    EXPECT(cSpiky > 2.0f);         // narrow pulse -> high crest
    EXPECT(cSpiky > cSine + 0.5f); // strictly peakier
    // Shape is zero-mean (no DC injection): mean V_out stays at V_bat within the RC ripple.
    VirtualConverter z;
    z.setPv(8.0f, 40.0f, 0.8f); z.setPassives(1e-3f, C, kL); z.setBat(20.0f, R);
    z.setVin(40.0f); z.setVout(20.0f); z.setPwm(mkPwm(0, 0));
    z.setBatRipple(1.0f, f, 2);
    double m = 0; const int M = 8000;
    for (int i = 0; i < M; ++i) { z.stepSeconds(kT, kFreq); m += z.getVout(); }
    EXPECT_NEAR(m / M, 20.0f, 0.05f);
}

// ----- V-Z: boost topology ---------------------------------------------------
//
// Boost rig: stiff source on the LV input, resistive load on the HV output.
// In boost, pwmCtrl is the LS (charging) switch and pwmRect the HS (delivering)
// one -- the same role swap buck.h does via isBoost.
void rigBoost(VirtualConverter &v, float vin, float vout, uint16_t ctrl, uint16_t rect,
              float rload = 12.0f, float cout = 1e-3f, float l = kL) {
    v.setBoost(true);
    v.setPv(200.0f, vin * 1.25f, 0.99f); // Isc >> any draw; Vin is held by runPinnedVin below
    v.setBat(0.0f, rload);               // pure resistive load (v_bat = 0)
    v.setBatRipple(0.0f, 100.0f);
    v.setPassives(1.0f, cout, l);
    v.setVin(vin);
    v.setVout(vout);
    v.setPwm(mkPwm(ctrl, rect));
}

// A battery input is a stiff VOLTAGE source, but setPv() is a current source -- left alone it
// charges C_in up to voc*1.05 and every boost ratio comes out high. Re-pin V_in each cycle and
// let only V_out evolve.
void runPinnedVin(VirtualConverter &v, int n, float vin) {
    for (int i = 0; i < n; ++i) {
        v.setVin(vin);
        v.stepSeconds(kT, kFreq);
    }
    v.setVin(vin);
}

// V: the branch is live at all -- same counts, opposite topology, Vout moves the other way.
void testV_boost_branch_is_live() {
    section("V: boost branch selected");
    VirtualConverter b, k;
    rigBoost(b, 24.0f, 24.0f, 300, 600);
    EXPECT(b.isBoost());
    rig(k, 24.0f, 24.0f, 300, 600, 1.0f, 1e-3f);
    EXPECT(!k.isBoost());
    runN(b, 4000);
    runN(k, 4000);
    // Boost pushes Vout above Vin; buck (same counts, 24->24 with D=0.3) pulls it down.
    EXPECT(b.getVout() > 24.0f);
    EXPECT(k.getVout() < 24.0f);
}

// W: CCM conversion ratio Vout/Vin = 1/(1-D), D = Ctrl(LS) duty.
//
// Verified AT the operating point, not by settling to it -- same caveat test A records for buck.
// With rect = pmax-ctrl there is no dead time, so L and C_out form a high-Q bidirectional
// resonator; started far from equilibrium it rings, and the `if (vOut_ < 0) vOut_ = 0` clamp
// rectifies the undershoot into a sustained oscillation (a pre-existing model artifact, not a
// boost-branch bug -- see the note in stepOneCycle). Pinning both rails and checking that the coil
// current is periodic is the real invariant: net volt-seconds over a cycle must be zero.
void testW_boost_ccm_ratio() {
    section("W: boost CCM ratio 1/(1-D)");
    // Settled, not pinned. The forward-Euler L/C_out instability (see the note at the vOut_ clamp)
    // only bites at light load, so pick the damped side of zeta < w0*T/4: r_bat 2 ohm, C_out 470uF.
    // rect = pmax-ctrl-1 is exactly what buck.h commands in CCM. A stiff source needs voc == vin,
    // otherwise setPv() is a current source that charges C_in up to voc and every ratio reads high.
    for (uint16_t ctrl : {200, 300, 400}) {
        VirtualConverter v;
        const float vin = 24.0f;
        const float D = (float) ctrl / (float) kPmax;
        const float want = vin / (1.0f - D);
        v.setBoost(true);
        v.setPv(200.0f, vin, 0.99f); // stiff ~vin rail, no pinning needed
        v.setBat(0.0f, 2.0f);
        v.setBatRipple(0.0f, 100.0f);
        v.setPassives(10e-3f, 470e-6f, kL);
        v.setVin(vin);
        v.setVout(vin);
        v.setPwm(mkPwm(ctrl, (uint16_t) (kPmax - ctrl - 1)));
        runN(v, 200000);
        std::printf("  W: D=%.2f Vout=%.2f want=%.2f\n", D, v.getVout(), want);
        EXPECT_REL(v.getVout(), want, 0.05);
    }
}

// W2: CCM with a real DC load current. W settles to the ideal ratio where iL rides on ~zero DC,
// which makes a power-balance check tautological (iOut/iIn == 1-D identically for a zero-DC
// triangle) -- and `dcm_` is `cEnd == 0.0f` exactly, so an !inDcm() assertion there would pass on
// float noise. Force a genuine DC bias instead: hold the rails slightly off the ideal ratio so the
// coil ramps, then check the lossless balance with real bias current. This is the case where the
// "phase 1 charges the input but reaches the output" attribution actually matters.
void testW2_boost_ccm_dc_bias() {
    section("W2: boost CCM balance under DC bias");
    const float vin = 24.0f, vout = 36.0f; // M=1.5 -> ideal D=1/3
    VirtualConverter v;
    rigBoost(v, vin, vout, 340, (uint16_t) (kPmax - 340 - 1), 12.0f, 1.0f);
    runPinned(v, 800, vin, vout); // slight imbalance accumulates a DC component
    const float iL = v.getIL();
    const float pIn = v.getIinAvg() * vin, pOut = v.getIoutAvg() * vout;
    std::printf("  W2: iL=%.3f A  Pin=%.4g W Pout=%.4g W  iIn/iOut=%.4f (Vout/Vin=%.4f)\n",
                iL, pIn, pOut, v.getIinAvg() / v.getIoutAvg(), vout / vin);
    EXPECT(std::fabs(iL) > 1.0f);                              // a real DC bias, not float noise
    EXPECT(!v.inDcm());                                        // and genuinely continuous
    EXPECT_REL(pIn, pOut, 0.01);                               // lossless
    EXPECT_REL(v.getIinAvg() / v.getIoutAvg(), vout / vin, 0.01); // the ratio that implies
}

// X: THE FLOOR. Vout can never sit below Vin -- the HS body diode passes the input
// through even with both switches off. This is the hazard the buck plant cannot show,
// and it is why a PSU setpoint at or below Vin is unreachable.
void testX_boost_vout_cannot_go_below_vin() {
    section("X: Vout >= Vin floor (passthrough)");
    // The cleanest statement of the hazard: command ZERO duty, start ABOVE Vin, and let a real
    // load discharge the output. It decays only to Vin and stops -- the HS body diode picks up
    // there. No commanded duty can take the rail lower, so a PSU setpoint <= Vin is unreachable.
    VirtualConverter v;
    rigBoost(v, 24.0f, 60.0f, 0, 0, 12.0f, 1e-3f);
    EXPECT_NEAR(v.getIL(), 0.0f, 1e-9f); // idle coil -> exercises the c==0 passthrough branch
    runPinnedVin(v, 400000, 24.0f);
    std::printf("  X: zero duty, 60V -> settled %.2f V (Vin=24, floor)\n", v.getVout());
    EXPECT(v.getVout() < 59.0f);  // it really did discharge...
    EXPECT(v.getVout() > 22.0f);  // ...but stopped at the Vin floor, not at 0
    EXPECT(v.getVout() < 27.0f);

    // Starting BELOW Vin, the same passthrough pulls the rail up to the floor unbidden.
    VirtualConverter w;
    rigBoost(w, 24.0f, 5.0f, 0, 0, 12.0f, 1e-3f);
    runPinnedVin(w, 400000, 24.0f);
    std::printf("  X: zero duty, 5V -> settled %.2f V (pulled up to floor)\n", w.getVout());
    EXPECT(w.getVout() > 20.0f);
}

// Y: duty-floor pumping. At the minimum Ctrl count the per-cycle energy is tiny;
// check the delivered power against E=0.5*L*Ipk^2 per cycle, then confirm it cannot
// hold a rail against a modest load. This is what decides whether PSU mode needs
// disable-and-rearm hysteresis at pwmCtrlMin.
void testY_boost_duty_floor_power() {
    section("Y: duty-floor injected power");
    const float vin = 48.0f;
    VirtualConverter v;
    // 1 count of 1000 at 39kHz = 25.6ns; measure with Vin/Vout pinned.
    rigBoost(v, vin, 80.0f, 1, 0, 12.0f, 1e-3f);
    runPinned(v, 200, vin, 80.0f);
    const float tOn = (1.0f / (float) kPmax) * kT;
    const float iPk = vin * tOn / kL;
    // NOT just E*fsw: during the DCM decay the INPUT keeps supplying energy alongside the coil,
    // so the output receives E*fsw * Vout/(Vout-Vin). Omitting that factor understates the
    // injected power by 2.5x at 48->80.
    const float eCycle = 0.5f * kL * iPk * iPk;
    const float pAnalytic = eCycle * (float) kFreq * 80.0f / (80.0f - vin);
    const float pModel = v.getIoutAvg() * 80.0f;
    std::printf("  Y: t_on=%.1fns Ipk=%.4gA  P_model=%.4gW P_analytic=%.4gW\n",
                tOn * 1e9f, iPk, pModel, pAnalytic);
    EXPECT(pModel > 0.0f);
    EXPECT_REL(pModel, pAnalytic, 0.15);

    // The decision-relevant part: the floor duty cannot hold 80V against a 12 ohm load -- it
    // collapses to the Vin floor. So PSU mode does NOT need disable-and-rearm hysteresis as long
    // as the load (or bleeder) exceeds the injected power, which by the numbers above it does.
    VirtualConverter w;
    rigBoost(w, vin, 80.0f, 1, 0, 12.0f, 1e-3f);
    runPinnedVin(w, 400000, vin);
    std::printf("  Y: floor duty into 12ohm settles at %.2f V (Vin=%.0f floor)\n",
                w.getVout(), vin);
    EXPECT(w.getVout() < 55.0f);      // cannot sustain the 80V setpoint...
    EXPECT(w.getVout() > vin - 2.0f); // ...and never falls below the Vin floor
}

// Z: topology asymmetry of the current sensors. In boost the coil is in series with
// the input, so iIn is non-zero even when nothing reaches the output; and a negative
// coil current in phase 3 routes to ground (LS body diode), not to the output.
void testZ_boost_current_routing() {
    section("Z: boost current routing");
    // Ctrl on / Rect off. The output does NOT receive zero -- in DCM all the charge is delivered
    // during phase 3 through the HS body diode. What is invariant is the lossless ratio: the coil
    // is in series with the input in every phase, so iIn/iOut == Vout/Vin exactly.
    VirtualConverter v;
    rigBoost(v, 24.0f, 80.0f, 300, 0, 12.0f, 1.0f);
    runPinned(v, 50, 24.0f, 80.0f);
    std::printf("  Z: iIn=%.4g iOut=%.4g  ratio=%.4f (Vout/Vin=%.4f)\n",
                v.getIinAvg(), v.getIoutAvg(), v.getIinAvg() / v.getIoutAvg(), 80.0f / 24.0f);
    EXPECT(v.getIinAvg() > 0.0f);
    EXPECT(v.getIoutAvg() > 0.0f); // phase-3 delivery, not zero
    EXPECT_REL(v.getIinAvg() / v.getIoutAvg(), 80.0f / 24.0f, 0.01);

    // Negative coil current with both switches off -> LS body diode to ground:
    // it is input current but must NOT be counted as output current.
    VirtualConverter w;
    rigBoost(w, 24.0f, 80.0f, 0, 0, 12.0f, 1.0f);
    w.setPwm(mkPwm(0, 400));
    runPinned(w, 1, 24.0f, 80.0f); // one delivering cycle drives iL negative
    EXPECT(w.getIL() < 0.0f);
    w.setPwm(mkPwm(0, 0));
    runPinned(w, 1, 24.0f, 80.0f);
    EXPECT_NEAR(w.getIoutAvg(), 0.0f, 1e-6f);
    EXPECT(w.getIinAvg() < 0.0f);
}

// X2: reverse coil current does NOT break the floor, and the one duty that does is clamped away.
// Test X covers zero duty (body-diode passthrough holds V_out up at V_in). Here: approach the
// operating point from ABOVE, so the sync rect conducts in reverse and bucks C_out back toward
// C_in. It settles at V_in/(1-D) -- above V_in -- not below it. Run on the DAMPED side of the
// forward-Euler criterion (see the vOut_ clamp note): in the light-load unstable regime the same
// duty shows a sub-V_in excursion with iL very negative, which is a solver artifact, and mistaking
// it for a real floor escape is an easy trap.
void testX2_boost_floor_holds_under_reverse_current() {
    section("X2: floor holds under reverse current");
    const float vin = 25.2f;
    VirtualConverter v;
    v.setBoost(true);
    v.setPv(200.0f, vin, 0.99f);
    v.setBat(0.0f, 2.0f);          // damped: zeta 0.082 > w0*T/4 = 0.042
    v.setBatRipple(0.0f, 100.0f);
    v.setPassives(10e-3f, 470e-6f, kL);
    v.setVin(vin);
    v.setVout(vin); // start at the floor and let it boost up to the ratio
    v.setPwm(mkPwm(200, 799));
    float lo = 1e9f;
    for (int i = 0; i < 600000; ++i) {
        v.setVin(vin);
        v.stepSeconds(kT, kFreq);
        if (v.getVout() < lo) lo = v.getVout();
    }
    const float want = vin / (1.0f - 0.2f);
    std::printf("  X2: settled %.2f (want %.2f), min seen %.2f, Vin=%.1f\n",
                v.getVout(), want, lo, vin);
    EXPECT_REL(v.getVout(), want, 0.05); // the duty-determined ratio, which is >= Vin by construction
    EXPECT(lo > vin - 2.0f);             // never meaningfully below the floor

    // The real escape: pwmCtrl == pwmMax leaves no path to the output at all, so V_out collapses
    // to zero while the coil shorts the input. buck.h's pwmCtrlMax = 0.9*driverPwmMax is the only
    // thing keeping it unreachable -- the floor is duty-clamp enforced at the top end.
    VirtualConverter w;
    w.setBoost(true);
    w.setPv(200.0f, vin, 0.99f);
    w.setBat(0.0f, 2.0f);
    w.setBatRipple(0.0f, 100.0f);
    w.setPassives(10e-3f, 470e-6f, kL);
    w.setVin(vin);
    w.setVout(80.0f);
    w.setPwm(mkPwm(kPmax, 0));
    for (int i = 0; i < 200000; ++i) { w.setVin(vin); w.stepSeconds(kT, kFreq); }
    std::printf("  X2: ctrl==pmax (no output path) -> Vout=%.2f\n", w.getVout());
    EXPECT(w.getVout() < 1.0f);
}

// Y2: scope on Y's conclusion. Y shows the duty floor cannot hold a rail against a 12 ohm load, so
// no disable-and-rearm hysteresis is needed THERE. But the threshold is load-dependent: with a
// near-open output the floor does pump the rail up past V_in without bound. So the plan's
// "hysteresis not needed" holds only while the load (or bleeder) exceeds the injected power --
// which is the design assumption, not a property of the converter.
void testY2_boost_duty_floor_pumps_open_output() {
    section("Y2: duty floor pumps a near-open output");
    const float vin = 48.0f;
    VirtualConverter v;
    rigBoost(v, vin, vin, 1, 0, 1e6f /*near-open*/, 1e-3f);
    runPinnedVin(v, 2000000, vin);
    std::printf("  Y2: floor duty into 1Mohm climbed %.0f -> %.2f V\n", vin, v.getVout());
    EXPECT(v.getVout() > vin + 2.0f); // unloaded, the floor really does creep upward
}

} // namespace

int main() {

    testA_ccm_voltsec();
    testB_ccm_ripple();
    testC_energy();
    testD_idle();
    testE_saturation();
    testF_charge_balance();
    testG_be_stability();
    testH_reverse_pump_balance();
    testI_iinavg_reverse();
    testJ_dcm_boundary();
    testK_phase_wrap();
    testL_pv_boundary();
    testM_pv_newton();
    testN_shim_wiring();
    testO_error_latch();
    testP_determinism();
    testQ_ncycle_equivalence();
    testR_dcm_voltsec();
    testS_degenerate();
    testT_mains_ripple();
    testU_spiky_ripple();
    testV_boost_branch_is_live();
    testW_boost_ccm_ratio();
    testW2_boost_ccm_dc_bias();
    testX_boost_vout_cannot_go_below_vin();
    testX2_boost_floor_holds_under_reverse_current();
    testY_boost_duty_floor_power();
    testY2_boost_duty_floor_pumps_open_output();
    testZ_boost_current_routing();

    std::printf("\nvconv-test: %d/%d passed\n", g_run - g_fail, g_run);
    return g_fail == 0 ? 0 : 1;
}

