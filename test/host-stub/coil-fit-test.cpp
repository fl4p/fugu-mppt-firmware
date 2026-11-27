// Host test of the coil-inductance estimator (src/selftest/coil_fit.h) on synthetic sweeps from the
// VirtualConverter plant with the switch-node ring on: known L, ring 200..400 kHz, Q 10..40, sensor
// noise, stiff and soft (low-light PV) sources. Compares the old estimator (10 steps, median of the
// per-point L) with the new one (48 steps, ring fit); checks the L(Ipk) split (plant L = L0*(1 - k*Ipk):
// false positives at k = 0, recovery of -1 and -3 %/A) and the Iout zero reads (a stale zero offset biases
// L by +14 % with a small stated error unless corrected; a drifting zero flags the run; an irradiance ramp
// does not bias), and the stated errors (noise, systematic) against the actual ones, also with on-time offsets and
// Iout gain errors, and the value persistL() lets `apply` write (never the corrected L, nothing without evidence).
// `--fit <file>` instead fits the datasets of a whitespace file (rows "D Vin Vout Iout", blank
// line between datasets, first line of each "fsw zsig ton") and prints "L f Lmed se sys k kse ksys nb
// [lo hi Lb seb sysb]..." per dataset (parity check against etc/measure_coil.py, see etc/test_measure_coil_fit.py).
//
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/coil-fit-test.cpp src/sim/vconv.cpp -o /tmp/coil-fit-test
//   /tmp/coil-fit-test [trials] [split trials] [offset trials]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "sim/vconv.h"
#include "selftest/coil_fit.h"

static constexpr uint16_t kPmax = 4103;
static constexpr uint32_t kFsw = 39000;

struct Pt { float d, vin, vout, iout; };

// steady plant point at HS count h: LS count as the firmware sets it in DCM (ratio from the voltages,
// 1 % worst-case error each side), the PV source sags with the load through c_in
static Pt point(VirtualConverter &v, uint16_t h, bool soft, int hoff = 0) {
    const float r = v.getVin() / v.getVout() * (0.99f / 1.01f) - 1.f;
    const uint16_t ls = (uint16_t) std::min<long>(std::lround(h * std::max(0.f, r)), kPmax - h - 1 - hoff);
    v.setPwmCount(0, h + hoff), v.setPwmCount(1, ls);
    v.stepSeconds(soft ? 0.25f : 0.01f, kFsw);
    double vi = 0, vo = 0, io = 0;
    const int n = 400;
    for (int k = 0; k < n; ++k) {
        v.stepSeconds(1.f / kFsw, kFsw);
        vi += v.getVin(), vo += v.getVout(), io += v.getIoutAvg();
    }
    return {(float) h / kPmax, (float) (vi / n), (float) (vo / n), (float) (io / n)};
}

// k: plant L DC bias (1/A); ioff: Iout zero error at the sweep start (A), decaying with tau (s, 0 = constant);
// irr: PV irradiance factor reached at the last band step; zero: the firmware's Iout zero reads before and
// after the sweep (z[0], z[1]) and their correction
// hoff: HS on-time offset (counts of 6.25 ns), gain: Iout gain error
struct Opt { float k = 0, ioff = 0, tau = 0, irr = 1; bool zero = false; int hoff = 0; float gain = 0; };
struct Zero { float z0 = 0, z1 = 0, sig = 0; };
static constexpr float kStepS = 1.3f; // firmware time per step

// one sweep as measure_coil does it: band from the idle voltages, stop on CCM or Iout > 2 A, floor
static std::vector<Pt> sweep(std::mt19937 &rng, float L, float fr, float q, float voc, float vbat, float isc,
                             int steps, const Opt &o = {}, Zero *z = nullptr) {
    std::normal_distribution<float> N(0, 1);
    VirtualConverter v;
    v.setPv(isc, voc, 0.86f);
    v.setBat(vbat, 0.03f);
    v.setPassives(940e-6f, 470e-6f, L);
    v.setDiodeVf(0.7f);
    v.setRing(1 / (39.478418f * fr * fr * L), q);
    v.setPwmMax(kPmax), v.setPwmFreq(kFsw);
    v.setVin(voc), v.setVout(vbat);
    v.setLBias(o.k);
    const bool soft = isc < 5;
    int lo, hi, step;
    std::vector<Pt> all, out;
    if (!coilfit::band(voc - 0.3f, vbat, kPmax, kPmax - 330, steps, lo, hi, step)) return out;
    const float off = 0.004f * N(rng); // residual current-sensor offset after zero calibration
    auto zoff = [&](float ts) { return o.ioff * (o.tau > 0 ? std::exp(-ts / o.tau) : 1.f); };
    float iMax = 0;
    int j = 0;
    for (int h = lo; h <= hi; h += step, ++j) {
        if (o.irr != 1) v.setPv(isc * (1 + (o.irr - 1) * (float) (h - lo) / (float) (hi - lo)), voc, 0.86f);
        Pt p = point(v, (uint16_t) h, soft, o.hoff);
        if (p.d * p.vin / p.vout > 0.97f) break; // firmware reports CCM
        p.iout = p.iout * (1 + o.gain + 0.005f * N(rng)) + 0.003f * N(rng) + off + zoff(kStepS * (float) (j + 1));
        p.vin += 0.03f * N(rng), p.vout += 0.01f * N(rng);
        all.push_back(p);
        iMax = std::max(iMax, p.iout);
        if (p.iout > 2.0f) break;
    }
    if (o.zero) {
        const float z0 = off + zoff(0) + 0.001f * N(rng), z1 = off + zoff(kStepS * (float) (j + 2)) + 0.001f * N(rng);
        std::vector<float> io(all.size());
        for (size_t i = 0; i < all.size(); ++i) io[i] = all[i].iout;
        const float sig = coilfit::zeroCorrect(io.data(), (int) io.size(), z0, z1);
        iMax = 0;
        for (size_t i = 0; i < all.size(); ++i) all[i].iout = io[i], iMax = std::max(iMax, io[i]);
        if (z) *z = {z0, z1, sig};
    }
    const float floor = std::max(0.15f, 0.2f * iMax);
    for (auto &p: all) if (p.iout >= floor && p.vin > p.vout) out.push_back(p);
    return out;
}

static coilfit::Result fitPts(const std::vector<Pt> &p, float zsig = 0, float ton = coilfit::kOnTimeMin) {
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    const int n = std::min<int>((int) p.size(), coilfit::kMaxPts);
    for (int i = 0; i < n; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
    return coilfit::fit(d, vi, vo, io, n, (float) kFsw, zsig, ton);
}

// actual / stated error (noise and systematic in quadrature), rms over an ensemble
struct Ratio {
    std::vector<float> q, se, sys;
    void add(float err, const coilfit::Result &r) {
        if (r.se == r.se) q.push_back(err / std::hypot(r.se, r.sys)), se.push_back(r.se), sys.push_back(r.sys);
    }
    static float med(std::vector<float> v) { std::sort(v.begin(), v.end()); return v.empty() ? NAN : v[v.size() / 2]; }
    float rms() const;
    void print() const { printf(" | stated +-%.2f%% noise +-%.2f%% syst (median), actual/stated rms %.2f", 100 * med(se), 100 * med(sys), rms()); }
    bool ok() const { return rms() > 0.5f && rms() < 2; }
};

struct Rnd { // random board and source
    float L, fr, q, voc, vbat;
    explicit Rnd(std::mt19937 &rng) {
        std::uniform_real_distribution<float> U(0, 1);
        L = 40e-6f + 60e-6f * U(rng), fr = 200e3f + 200e3f * U(rng), q = 10 + 30 * U(rng);
        voc = 55 + 25 * U(rng), vbat = 25.5f + 3 * U(rng);
    }
};

// coilfit::persistL(); also builds against the former persistL(r0, r, st), so the known-bad cases can be run there
template<class R> static float persist(const R &r0, const R &r, int st) {
    if constexpr (requires { coilfit::persistL(r0, r, st); }) return coilfit::persistL(r0, r, st);
    else return coilfit::persistL(r0, st);
}

static float rmsOf(const std::vector<float> &e) {
    float s = 0;
    for (float x: e) s += x * x;
    return e.empty() ? NAN : std::sqrt(s / (float) e.size());
}

float Ratio::rms() const { return rmsOf(q); }

static float meanOf(const std::vector<float> &e) {
    float s = 0;
    for (float x: e) s += x;
    return e.empty() ? NAN : s / (float) e.size();
}

// L(Ipk) split: false positives at k = 0, recovery of -1 and -3 %/A (soft and stiff source, zero-corrected)
static int testSplit(int trials) {
    int fail = 0;
    const float ks[] = {0.f, 0.01f, 0.03f}, iscs[] = {1.5f, 8.f};
    for (float k: ks)
        for (float isc: iscs) {
            std::mt19937 rng(777);
            std::vector<float> e, zs;
            int sig = 0, nbSum = 0, nbMin = 99;
            for (int t = 0; t < trials; ++t) {
                const Rnd b(rng);
                Opt o;
                o.k = k, o.zero = true;
                Zero z;
                const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, isc, 48, o, &z);
                if ((int) p.size() < coilfit::kMinFitPts) continue;
                const auto sp = fitPts(p, z.sig);
                if (!(sp.k == sp.k)) continue;
                e.push_back((sp.k + k) * 100), zs.push_back((sp.k + k) / sp.kse);
                sig += std::fabs(sp.k) > 2 * sp.kse;
                nbSum += sp.nb, nbMin = std::min(nbMin, sp.nb);
            }
            const float rate = (float) sig / (float) e.size();
            printf("split k=%.0f %%/A, %s source, %zu sweeps: slope err mean %+.2f rms %.2f %%/A, rms z %.2f, "
                   "%s %.1f%%, bins %.1f (min %d)\n", -k * 100, isc < 5 ? "soft" : "stiff", e.size(), meanOf(e),
                   rmsOf(e), rmsOf(zs), k == 0 ? "false positive" : "significant", 100 * rate,
                   (float) nbSum / (float) e.size(), nbMin);
            // slope within 0.5 %/A; -1 %/A is near the 2-sigma limit, -3 %/A must be found
            const bool ok = std::fabs(meanOf(e)) < 0.5f && rmsOf(e) < 0.5f && rmsOf(zs) > 0.5f && rmsOf(zs) < 2 &&
                            (k == 0 ? rate <= 0.1f : k < 0.02f || rate > 0.95f);
            if (!ok) ++fail, printf("  FAIL\n");
        }
    return fail;
}

// Iout zero error during the sweep (stale offset calibration, thermal drift) and a PV irradiance ramp
static int testDisturbed(int trials) {
    struct Case { const char *name; Opt o; float isc; bool flag; };
    const Case cases[] = {
        {"stale zero -0.12 A", {0, -0.12f, 0, 1, false}, 8, false},
        {"stale zero -0.12 A, zero-corrected", {0, -0.12f, 0, 1, true}, 8, false},
        {"zero -0.12 A decaying 20 s, zero-corrected", {0, -0.12f, 20, 1, true}, 8, true},
        {"irradiance -40 %, soft, zero-corrected", {0, 0, 0, 0.6f, true}, 1.5f, false},
        {"irradiance -40 %, stiff, zero-corrected", {0, 0, 0, 0.6f, true}, 8, false},
    };
    int fail = 0;
    for (const auto &c: cases) {
        std::mt19937 rng(4242);
        std::vector<float> e, se;
        Ratio ra;
        int flagged = 0;
        for (int t = 0; t < trials; ++t) {
            const Rnd b(rng);
            Zero z;
            const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, c.isc, 48, c.o, &z);
            if ((int) p.size() < coilfit::kMinFitPts) continue;
            const auto r = fitPts(p, z.sig);
            e.push_back((r.L / b.L - 1) * 100), se.push_back(100 * std::hypot(r.se, r.sys));
            ra.add(r.L / b.L - 1, r);
            flagged += std::fabs(z.z1 - z.z0) > coilfit::kZeroTol;
        }
        const float rate = (float) flagged / (float) e.size();
        std::vector<float> ae = e;
        for (float &x: ae) x = std::fabs(x);
        std::sort(ae.begin(), ae.end());
        const float p95 = ae[(size_t) (0.95f * (float) (ae.size() - 1))];
        printf("%s: %zu sweeps, L err mean %+.1f%% p95 %.1f%%, flagged disturbed %.0f%%", c.name, e.size(), meanOf(e),
               p95, 100 * rate);
        ra.print(), printf(" (no on-time/gain error)\n");
        bool ok;
        if (!c.o.zero) ok = meanOf(e) > 5 && meanOf(se) < 3; // reproduces the biased-but-confident result
        else if (c.flag) ok = rate > 0.95f;
        else ok = p95 < 4 && rate < 0.05f;
        if (!ok) ++fail, printf("  FAIL\n");
    }
    return fail;
}

// stated error against the actual one on the ensembles above, each sweep with an on-time offset and an Iout gain error
// drawn within the bounds the fit assumes (|ton| <= 50 ns, |gain| <= 1 %); fixed on-time offsets: the apparent slope
// against the bound ksys
static int testOffsets(int trials) {
    struct Case { const char *name; Opt o; float isc; };
    const Case cases[] = {
        {"soft 0.6 A", {0, 0, 0, 1, true}, .6f},
        {"soft 1.5 A", {0, 0, 0, 1, true}, 1.5f},
        {"stiff", {0, 0, 0, 1, true}, 8},
        {"stale zero -0.12 A, zero-corrected", {0, -0.12f, 0, 1, true}, 8},
        {"zero -0.12 A decaying 20 s (disturbed)", {0, -0.12f, 20, 1, true}, 8},
        {"irradiance -40 %, soft", {0, 0, 0, 0.6f, true}, 1.5f},
        {"irradiance -40 %, stiff", {0, 0, 0, 0.6f, true}, 8},
    };
    int fail = 0;
    for (const auto &c: cases) {
        std::mt19937 rng(99);
        std::uniform_int_distribution<int> H(-8, 8);
        std::uniform_real_distribution<float> G(-coilfit::kGainTol, coilfit::kGainTol);
        std::vector<float> e;
        Ratio ra;
        for (int t = 0; t < trials; ++t) {
            const Rnd b(rng);
            Opt o = c.o;
            o.hoff = H(rng), o.gain = G(rng);
            Zero z;
            const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, c.isc, 48, o, &z);
            if ((int) p.size() < coilfit::kMinFitPts) continue;
            const auto r = fitPts(p, z.sig);
            e.push_back(r.L / b.L - 1), ra.add(r.L / b.L - 1, r);
        }
        printf("on-time +-50 ns, gain +-1 %%, %s, %zu sweeps: L err mean %+.2f%% rms %.2f%%", c.name, e.size(),
               100 * meanOf(e), 100 * rmsOf(e));
        ra.print(), printf("\n");
        if (!ra.ok()) ++fail, printf("  FAIL: actual/stated error not of order 1\n");
    }
    for (int hoff: {0, 5, 10}) {
        std::mt19937 rng(31);
        std::vector<float> k, ks, kn;
        int covered = 0;
        for (int t = 0; t < trials; ++t) {
            const Rnd b(rng);
            Opt o;
            o.zero = true, o.hoff = hoff;
            Zero z;
            const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, 8, 48, o, &z);
            if ((int) p.size() < coilfit::kMinFitPts) continue;
            const auto r = fitPts(p, z.sig);
            if (!(r.k == r.k)) continue;
            k.push_back(100 * r.k), ks.push_back(100 * r.ksys), kn.push_back(100 * r.kse);
            covered += std::fabs(r.k) <= r.ksys + 2 * r.kse;
        }
        printf("on-time offset %+.0f ns, coil slope 0, %zu sweeps: apparent slope mean %+.2f rms %.2f %%/A, offset bound"
               " +-%.2f %%/A, noise +-%.2f %%/A (median); within bound + 2 noise: %d/%zu\n", hoff * 6.25f, k.size(),
               meanOf(k), rmsOf(k), Ratio::med(ks), Ratio::med(kn), covered, k.size());
        if (covered < 0.95f * (float) k.size()) ++fail, printf("  FAIL: slope bound does not cover the offset\n");
    }
    return fail;
}

// on-time offset estimate (fitOnTime): plant HS on-time offset hoff (6.25 ns counts) on a coil without (k = 0) or with
// a DC-bias slope; per case the recovered offset, uncorrected and corrected L errors, the stated offset error vs the actual
// one and the share of sweeps the offset was applied in. 600 ns: out of range, reported and never applied
static int testOnTime(int trials) {
    struct Case { int hoff; float k, isc; };
    const Case cases[] = {{0, 0, 8}, {16, 0, 8}, {-16, 0, 8}, {32, 0, 8}, {16, 0, 1.5f}, {16, 0.005f, 8}, {0, 0.01f, 8},
                         {96, 0, 8}};
    int fail = 0;
    for (const auto &c: cases) {
        std::mt19937 rng(555);
        std::vector<float> e0, e1, ed, zd;
        int applied = 0, kept = 0, corr = 0, n = 0, st[5] = {};
        for (int t = 0; t < trials; ++t) {
            const Rnd b(rng);
            Opt o;
            o.zero = true, o.hoff = c.hoff, o.k = c.k;
            Zero z;
            const auto p = sweep(rng, b.L, b.fr, b.q, b.voc, b.vbat, c.isc, 48, o, &z);
            if ((int) p.size() < coilfit::kMinFitPts) continue;
            float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
            const int m = std::min<int>((int) p.size(), coilfit::kMaxPts);
            for (int i = 0; i < m; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
            const auto r0 = coilfit::fit(d, vi, vo, io, m, (float) kFsw, z.sig);
            int s = 0;
            const auto r1 = coilfit::fitOnTime(d, vi, vo, io, m, (float) kFsw, z.sig, r0, s);
            const float pl = persist(r0, r1, s);
            ++n, ++st[s], kept += pl == pl, corr += pl == pl && pl != r0.L;
            // the plant's L at the points' mean Ipk (k: L = L0 (1 - k Ipk) per cycle)
            float ipk = 0;
            for (int i = 0; i < m; ++i) ipk += coilfit::ipkDcm(d[i], vi[i], vo[i], io[i]) / (float) m;
            const float lp = b.L * (1 - c.k * ipk);
            e0.push_back(r0.L / lp - 1);
            if (s == 0) {
                ++applied, e1.push_back(r1.L / lp - 1);
                const float err = r1.dl - c.hoff * 6.25e-9f;
                ed.push_back(err * 1e9f), zd.push_back(err / std::hypot(r1.dlse, r1.dlsys));
            }
        }
        printf("on-time %+.0f ns, coil slope %+.1f %%/A, %s, %d sweeps: uncorrected L err mean %+.2f%% rms %.2f%% | applied"
               " %d (status 2/3/4: %d/%d/%d): corrected L err mean %+.2f%% rms %.2f%%, offset err mean %+.0f rms %.0f ns, "
               "rms z %.2f | persisted %d\n", c.hoff * 6.25f, -c.k * 100, c.isc < 5 ? "soft" : "stiff", n, 100 * meanOf(e0),
               100 * rmsOf(e0), applied, st[2], st[3], st[4], 100 * meanOf(e1), 100 * rmsOf(e1), meanOf(ed), rmsOf(ed),
               rmsOf(zd), kept);
        if (corr) ++fail, printf("  FAIL: %d corrected L persisted (one run does not validate the offset)\n", corr);
        if (c.hoff == 0 && c.k == 0 && c.isc > 5 && 3 * kept < 2 * n) ++fail, printf("  FAIL: clean sweeps refused\n");
        if (c.hoff * 6.25e-9f > coilfit::kDlMax) { // out of range: reported (status 4), nothing persisted
            if (kept || 3 * st[4] < 2 * n) ++fail, printf("  FAIL: out-of-range offset: %d persisted\n", kept);
        } else if (c.k == 0 && c.isc > 5 && !(3 * applied >= 2 * n && std::fabs(meanOf(e1)) < 0.5f && rmsOf(e1) < 1.f &&
                                       rmsOf(zd) < 1.5f && st[4] == 0))
            ++fail, printf("  FAIL\n");
    }
    return fail;
}

// deterministic (as the review probe): 48.6 uH, 80/26.6 V, 39 kHz, 48 points at D 0.075..0.215, offset dl, relative
// Iout residual a*sin(2.134 i). Beyond kDlMax nothing may be persisted at any residual (a larger offset or error must
// not turn a refusal into a write); non-finite inputs must end and persist nothing implausible.
static int testOnTimeTail() {
    const float L = 48.6e-6f, fsw = 39000, vi0 = 80, vo0 = 26.6f;
    float d[48], vi[48], vo[48], io[48];
    auto gen = [&](float dl, float a) {
        for (int i = 0; i < 48; ++i) {
            d[i] = 0.075f + (float) i * 0.14f / 47, vi[i] = vi0, vo[i] = vo0;
            const float de = d[i] + dl * fsw;
            io[i] = (vi0 - vo0) * vi0 * de * de / (2 * vo0 * fsw * L) * (1 + a * std::sin((float) i * 2.134f));
        }
    };
    int fail = 0, nk = 0, nt = 0;
    for (float dl: {-3000e-9f, -1000e-9f, -600e-9f, -450e-9f, 450e-9f, 600e-9f, 1000e-9f, 3000e-9f})
        for (float a: {0.f, 0.01f, 0.05f, 0.15f, 0.25f, 0.4f, 0.8f, 2.f}) {
            gen(dl, a);
            const auto r0 = coilfit::fit(d, vi, vo, io, 48, fsw, 0.002f);
            int st;
            const auto r = coilfit::fitOnTime(d, vi, vo, io, 48, fsw, 0.002f, r0, st);
            const float l = persist(r0, r, st);
            ++nt;
            if (l == l) ++nk, printf("  FAIL: offset %+.0f ns, residual %.0f %%: status %d, %.2f uH persisted\n",
                                     dl * 1e9f, a * 100, st, l * 1e6f);
        }
    fail += nk > 0;
    int nf = 0;
    for (int k = 0; k < 6; ++k) { // one bad value per case; fsw inf in the last
        gen(-100e-9f, 0.01f);
        float *v[] = {vi, vo, io, d, vi, io};
        v[k][7] = k < 3 ? INFINITY : k == 3 ? NAN : -INFINITY;
        const float f = k == 5 ? INFINITY : fsw;
        const auto r0 = coilfit::fit(d, vi, vo, io, 48, f, 0.002f);
        int st;
        const auto r = coilfit::fitOnTime(d, vi, vo, io, 48, f, 0.002f, r0, st);
        const float l = persist(r0, r, st);
        if (l == l && !(std::fabs(l / L - 1) < 0.05f)) ++nf, printf("  FAIL: non-finite case %d persisted %g\n", k, l);
    }
    printf("on-time tail: %d/%d offset/residual cases persisted, non-finite inputs: %d bad\n", nk, nt, nf);
    return fail + (nf > 0);
}

// missing evidence persists nothing: 48.6 uH, 80/26.6 V, 39 kHz, D 0.075..0.215, on-time +600 ns (out of range) over
// 3, 15, 16 and 48 points (under kMinFitPts the offset is not estimated; persisting the uncorrected L there wrote
// ~36 uH), with a 10 % 320 kHz node ring (resolved from 16 points: the offset bound refuses) and without (no stated L
// error); without an offset the ringing 48-point sweep is written, within 1 %
static int testPersist() {
    const float L = 48.6e-6f, fsw = 39000, vi0 = 80, vo0 = 26.6f;
    float d[48], vi[48], vo[48], io[48];
    int fail = 0;
    for (float a: {0.1f, 0.f})
        for (float dl: {600e-9f, 0.f})
            for (int n: {3, 15, 16, 48}) {
                for (int i = 0; i < n; ++i) {
                    d[i] = 0.075f + (float) i * 0.14f / (float) (n - 1), vi[i] = vi0, vo[i] = vo0;
                    const float de = d[i] + dl * fsw;
                    io[i] = (vi0 - vo0) * vi0 * de * de / (2 * vo0 * fsw * L) *
                            (1 + a * std::cos(6.2831853f * 320e3f * (1 - de * vi0 / vo0) / fsw));
                }
                const auto r0 = coilfit::fit(d, vi, vo, io, n, fsw, 0.002f);
                int st;
                const auto r = coilfit::fitOnTime(d, vi, vo, io, n, fsw, 0.002f, r0, st);
                const float l = persist(r0, r, st);
                const bool want = a > 0 && dl == 0 && n == 48, never = n < 16 || dl > 0 || a == 0;
                const bool bad = want ? !(std::fabs(l / L - 1) < 0.01f)
                                      : never && (l == l || (a > 0 && dl > 0 && n >= 16 && !r0.nh)); // offset gate refuses
                fail += bad;
                printf("persist: ring %.0f %%, %+.0f ns, %d pts, %s: status %d, uncorrected %.2f uH, %s%.2f uH%s\n",
                       a * 100, dl * 1e9f, n, r0.nh ? "ring fit" : "no ring fit", st, r0.L * 1e6f,
                       l == l ? "persisted " : "nothing persisted ", l * 1e6f, bad ? "  FAIL" : "");
            }
    return fail;
}

static int fitFile(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return 2;
    char line[256];
    std::vector<Pt> p;
    float fsw = 0, zsig = 0, ton = coilfit::kOnTimeMin;
    auto flush = [&]() {
        if (fsw <= 0) return;
        float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
        const int n = std::min<int>((int) p.size(), coilfit::kMaxPts);
        for (int i = 0; i < n; ++i) d[i] = p[i].d, vi[i] = p[i].vin, vo[i] = p[i].vout, io[i] = p[i].iout;
        const auto r = coilfit::fit(d, vi, vo, io, n, fsw, zsig, ton);
        printf("%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d", r.L, r.f, r.Lmed, r.se, r.sys, r.k, r.kse, r.ksys, r.nb);
        for (int b = 0; b < r.nb; ++b) printf(" %.9g %.9g %.9g %.9g %.9g", r.lo[b], r.hi[b], r.Lb[b], r.seb[b], r.sysb[b]);
        printf("\n");
        p.clear(), fsw = 0;
    };
    while (fgets(line, sizeof line, f)) {
        Pt q{};
        const int k = sscanf(line, "%f %f %f %f", &q.d, &q.vin, &q.vout, &q.iout);
        if (k == 4) p.push_back(q);
        else if (k == 3 && fsw <= 0) fsw = q.d, zsig = q.vin, ton = q.vout; // header "fsw zsig ton"
        else flush();
    }
    flush();
    fclose(f);
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 2 && std::string(argv[1]) == "--fit") return fitFile(argv[2]);
    if (argc > 2 && std::string(argv[1]) == "--ontime") return testOnTime(atoi(argv[2]));
    const int trials = argc > 1 ? atoi(argv[1]) : 24;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> U(0, 1);
    const float iscs[] = {0.6f, 1.5f, 8.f};
    int fail = 0;
    { // zero interpolated over the sweep rows, not over the stored points (as etc/measure_coil.py)
        float io[3] = {1, 1, 1};
        const uint8_t row[3] = {1, 3, 4};
        coilfit::zeroCorrect(io, 3, 0, 0.05f, row, 6);
        const bool ok = std::fabs(io[0] - 0.99f) < 1e-6f && std::fabs(io[1] - 0.97f) < 1e-6f && std::fabs(io[2] - 0.96f) < 1e-6f;
        printf("zero over rows 1,3,4 of 6: %.3f %.3f %.3f%s\n", io[0], io[1], io[2], ok ? "" : "  FAIL");
        fail += !ok;
    }
    for (float isc: iscs) {
        std::vector<float> eOld, eNew, eF;
        Ratio ra;
        int nNew = 0, ring = 0;
        for (int t = 0; t < trials; ++t) {
            const float L = 40e-6f + 60e-6f * U(rng), C = 3e-9f + 5e-9f * U(rng);
            const float fr = 1 / (6.2831853f * std::sqrt(L * C)), q = 10 + 30 * U(rng);
            const float voc = 62 + 18 * U(rng), vbat = 25.5f + 3 * U(rng);
            const auto po = sweep(rng, L, fr, q, voc, vbat, isc, 10);
            const auto pn = sweep(rng, L, fr, q, voc, vbat, isc, 48);
            if (po.size() < 3 || pn.size() < 3) continue;
            eOld.push_back(fitPts(po).Lmed / L - 1);
            const auto r = fitPts(pn);
            eNew.push_back(r.L / L - 1);
            ra.add(r.L / L - 1, r);
            nNew += (int) pn.size();
            if (r.f > 0) eF.push_back(r.f / fr - 1), ++ring;
        }
        auto stat = [](std::vector<float> e, float &mean, float &rms, float &p95) {
            mean = rms = 0;
            for (float x: e) mean += x, rms += x * x;
            mean /= (float) e.size(), rms = std::sqrt(rms / (float) e.size());
            for (float &x: e) x = std::fabs(x);
            std::sort(e.begin(), e.end());
            p95 = e[(size_t) (0.95f * (float) (e.size() - 1))];
        };
        float m0, r0, p0, m1, r1, p1, mf = 0, rf = 0, pf = 0;
        stat(eOld, m0, r0, p0);
        stat(eNew, m1, r1, p1);
        if (!eF.empty()) stat(eF, mf, rf, pf);
        printf("Isc %.1f A (%s source), %zu sweeps: old 10-step median: mean %+.1f%% rms %.1f%% p95 %.1f%% | "
               "new 48-step ring fit (%.0f pts): mean %+.1f%% rms %.1f%% p95 %.1f%% | ring found %d/%zu, f err rms %.1f%%\n",
               isc, isc < 5 ? "soft" : "stiff", eNew.size(), 100 * m0, 100 * r0, 100 * p0, (float) nNew / (float) eNew.size(),
               100 * m1, 100 * r1, 100 * p1, ring, eNew.size(), 100 * rf);
        ra.print(), printf(" (no on-time/gain error)\n");
        if (!(r1 < r0) || !(p1 < 0.04f)) ++fail, printf("  FAIL: new estimator not better / p95 >= 4%%\n");
    }
    fail += testSplit(argc > 2 ? atoi(argv[2]) : 100);
    fail += testDisturbed(trials);
    fail += testOffsets(argc > 3 ? atoi(argv[3]) : 16);
    fail += testOnTime(argc > 3 ? atoi(argv[3]) : 16);
    fail += testOnTimeTail();
    fail += testPersist();
    printf("coil-fit-test: %s\n", fail ? "FAIL" : "PASS");
    return fail ? 1 : 0;
}
