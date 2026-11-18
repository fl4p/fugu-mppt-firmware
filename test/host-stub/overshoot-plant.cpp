// Battery-open overshoot, plant only: vconv buck (per-PWM-cycle) against a fine-step reference ODE with ideal
// switches and ideal body diodes, at frozen duty after the event. Stiff Vin, no ring, no losses.
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/overshoot-plant.cpp src/sim/vconv.cpp -o /tmp/ovp
//   /tmp/ovp L_uH C_uF rect(sync|off) [ms]
#include "sim/vconv.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr int kPmax = 4103;
constexpr uint32_t kFreq = 38996;
constexpr float kT = 1.f / kFreq;
constexpr float kVin = 64.f, kVbat = 28.f, kRbat = 0.02f;

// fine-step reference: coil i, Vout v; HS on [0,tH), LS on [tH, tH+tL), else body diodes
struct Ref {
    double i, v, L, C;
    double peak = 0;
    void cycle(double tH, double tL, double rbat, double vbat, int n) {
        const double dt = kT / n;
        for (int k = 0; k < n; ++k) {
            // node voltage averaged over the substep, so the switch edges need not sit on the grid
            const double t0 = k * dt, t1 = t0 + dt;
            const double fH = std::clamp((tH - t0) / dt, 0.0, 1.0);
            const double fL = std::clamp((std::min(t1, tH + tL) - std::max(t0, tH)) / dt, 0.0, 1.0);
            const double fO = 1.0 - fH - fL;
            const double vd = i > 0 ? 0 : i < 0 ? kVin : v; // ideal diodes, coil idles at zero
            const double vn = fH * kVin + fO * vd;
            double di = (vn - v) / L * dt;
            if (fO > 0.5 && ((i > 0 && i + di < 0) || (i < 0 && i + di > 0))) di = -i;
            const double iMid = i + 0.5 * di; // midpoint for the cap
            i += di;
            v += (iMid - (v - vbat) / rbat) / C * dt;
            if (v > peak) peak = v;
        }
    }
};
}

int main(int argc, char **argv) {
    if (argc < 4) return fprintf(stderr, "usage: L_uH C_uF sync|off [ms]\n"), 2;
    const float L = atof(argv[1]) * 1e-6f, C = atof(argv[2]) * 1e-6f;
    const char *mode = argv[3];
    const float ms = argc > 4 ? atof(argv[4]) : 20.f;
    const float iOut = 28.f;
    const float D = (kVbat + iOut * kRbat) / kVin;
    const int hs = (int) lroundf(D * kPmax);
    // sync: LS fills the rest of the cycle; de: LS until the steady-state valley current would hit zero at 0 A load
    // (the pre-event rect stays frozen); off: LS never on
    int ls = !strcmp(mode, "sync") ? kPmax - hs - 1 : 0;

    VirtualConverter p;
    p.setPv(100.f, kVin * 1.0001f, 0.999f);
    p.setPvBlocking(true);
    p.setPassives(10.f, C, L); // stiff: the plant moves Vin within the cycle, setVin only resets its start
    p.setPwmMax(kPmax), p.setPwmFreq(kFreq);
    p.setVin(kVin), p.setVout(kVbat);
    p.setBat(kVbat, kRbat);
    p.setPwmCount(0, hs), p.setPwmCount(1, ls);

    Ref r{iOut, kVbat, L, C};
    // steady state at the operating point
    for (int k = 0; k < 4000; ++k) {
        p.setVin(kVin);
        p.stepSeconds(kT, kFreq);
        r.cycle(hs * (double) kT / kPmax, ls * (double) kT / kPmax, kRbat, kVbat, 400);
    }
    printf("# L=%.0fuH C=%.0fuF %s  pre: vconv Vout %.3f  ref Vout %.3f i %.2f\n", L * 1e6, C * 1e6, mode, p.getVout(),
           r.v, r.i);
    p.setBat(0, 1e9f);
    r.peak = r.v;
    // de: the frozen LS window equals the pre-event one (CCM at 28 A); off: switch the LS off at the event
    if (!strcmp(mode, "off")) ls = 0, p.setPwmCount(1, 0);
    const int n = (int) lroundf(ms * 1e-3f / kT);
    float pk = p.getVout();
    printf("# t_ms vconv_V ref_V\n");
    for (int k = 1; k <= n; ++k) {
        p.setVin(kVin);
        p.stepSeconds(kT, kFreq);
        r.cycle(hs * (double) kT / kPmax, ls * (double) kT / kPmax, 1e9, 0, 400);
        if (p.getVout() > pk) pk = p.getVout();
        if (k % 8 == 0) printf("%6.3f %7.3f %7.3f\n", k * kT * 1e3, p.getVout(), r.v);
    }
    printf("# peak vconv %.2f  ref %.2f\n", pk, r.peak);
}
