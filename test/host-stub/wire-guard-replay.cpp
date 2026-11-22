// Replay recorded converter + BMS data through OutputImpedance, WireResistance and WireGuard.
// Data from etc/wire_replay_fetch.py; several directories are merged, a later one replacing the
// earlier from its first sample on.
//
//   clang++ -std=gnu++20 -O2 -I src test/host-stub/wire-guard-replay.cpp -o /tmp/wire-guard-replay
//   /tmp/wire-guard-replay [options] <datadir>...
//     -f <mOhm>@<unix s>   inject a series resistance from that time on
//     -s <unix s>          start the guard here (data before it is skipped)
//     -p                   prime: learn the baseline over the whole data first, then judge every day
//     -P <mask>            graded paths (WireGuard::paths): 1 fast, 2 Rc, 4 Ro; default 7
//     -v                   print level changes and fast-path minutes >= 1.2 W
//     -r <s,s,...|@file>   restart the guard at these unix times, as the firmware does: baseline from the
//                          last NVS save (saveDue, 600 s retry), the open day from its RTC copy (every 10 s,
//                          where the guard has one), fits, sweeps and timers lost; the guard sees uptime
//     -R                   with -r: the RTC copy is lost on every restart (power loss)
//     -d                   print every day close
//
// Levels are reported as level(), i.e. as with wire_protect=2 (the learning phase still caps them at warn).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <vector>
#include <algorithm>
#include <map>
#include "out_impedance.h"
#include "wire_guard.h"

struct M { int64_t t; float uo, i; };
struct B { int64_t t; char k; float v; };
struct S { int64_t seg, t; float uo, i; int pwm; };
struct W { int64_t t; float wf, pw; };

static std::vector<M> mp;
static std::vector<B> bms;
static std::vector<S> sw;
static std::vector<W> dev;

template<class T> static void cut(std::vector<T> &v, int64_t t) {
    v.erase(std::remove_if(v.begin(), v.end(), [t](const T &x) { return x.t >= t; }), v.end());
}

static FILE *open(const char *dir, const char *name) {
    char p[512], line[256];
    snprintf(p, sizeof p, "%s/%s", dir, name);
    FILE *f = fopen(p, "r");
    if (f && !fgets(line, sizeof line, f)) { fclose(f); f = nullptr; }
    return f;
}

static void load(const char *dir) {
    char line[256];
    std::vector<M> m2;
    FILE *f = open(dir, "mppt1s.csv");
    if (!f) { fprintf(stderr, "no %s/mppt1s.csv\n", dir); exit(2); }
    while (fgets(line, sizeof line, f)) { M m; if (sscanf(line, "%lld,%f,%f", &m.t, &m.uo, &m.i) == 3) m2.push_back(m); }
    fclose(f);
    if (m2.empty()) return;
    const int64_t t0 = std::min_element(m2.begin(), m2.end(), [](auto &a, auto &b) { return a.t < b.t; })->t;
    cut(mp, t0); cut(bms, t0); cut(sw, t0); cut(dev, t0);
    mp.insert(mp.end(), m2.begin(), m2.end());
    if ((f = open(dir, "bms.csv"))) {
        while (fgets(line, sizeof line, f)) { B b; if (sscanf(line, "%lld,%c,%f", &b.t, &b.k, &b.v) == 3) bms.push_back(b); }
        fclose(f);
    }
    if ((f = open(dir, "sweeps.csv"))) {
        while (fgets(line, sizeof line, f)) {
            S s; char uo[32], i[32], pw[32];
            if (sscanf(line, "%lld,%lld,%31[^,],%31[^,],%31s", &s.seg, &s.t, uo, i, pw) == 5 && *uo && *i &&
                strcmp(uo, "None") && strcmp(i, "None")) {
                s.uo = (float) atof(uo); s.i = (float) atof(i); s.pwm = atoi(pw);
                s.seg = s.seg < 1000000 ? -1 - s.seg : s.seg; // old files count segments from 0
                sw.push_back(s);
            }
        }
        fclose(f);
    }
    if ((f = open(dir, "tele_wf.csv"))) {
        while (fgets(line, sizeof line, f)) {
            W w{}; char pw[32] = "";
            if (sscanf(line, "%lld,%f,%31s", &w.t, &w.wf, pw) >= 2) { w.pw = *pw ? (float) atof(pw) : NAN; dev.push_back(w); }
        }
        fclose(f);
    }
}

static const char *ts(int64_t ms, const char *fmt = "%m-%d %H:%M") {
    static char b[8][32]; static int k; k = (k + 1) % 8;
    time_t t = ms / 1000; struct tm tm; gmtime_r(&t, &tm);
    strftime(b[k], 32, fmt, &tm);
    return b[k];
}

struct SweepRes { int64_t tEnd; float r; };

static std::vector<SweepRes> fitSweeps(float dR, int64_t faultMs) {
    std::vector<SweepRes> out;
    std::map<int64_t, std::vector<S>> segs;
    for (auto &s: sw) segs[s.seg].push_back(s);
    for (auto &[k, v]: segs) {
        std::sort(v.begin(), v.end(), [](auto &a, auto &b) { return a.t < b.t; });
        size_t pk = 0;
        for (size_t j = 0; j < v.size(); ++j) if (v[j].pwm > v[pk].pwm) pk = j;
        OutputImpedance z;
        z.reset();
        for (size_t j = 0; j <= pk; ++j)
            z.add(v[j].uo + (v[j].t >= faultMs ? dR * v[j].i : 0.f), v[j].i, (uint32_t) (v[j].t * 1000));
        auto fit = z.finish((uint32_t) (v[pk].t / 1000));
        if (fit.ok) out.push_back({v[pk].t, fit.r});
    }
    std::sort(out.begin(), out.end(), [](auto &a, auto &b) { return a.tEnd < b.tEnd; });
    return out;
}

struct Day {
    float pwMax = -INFINITY, iMax = 0, devPwMax = -INFINITY, rcMax = -INFINITY, roMax = -INFINITY, allMax = -INFINITY;
    int fastMin = 0, fastWarnMin = 0, lvlMin[4]{}, devWfPos = 0, anyMin = 0;
    float baseA = NAN, baseRc = NAN, baseRb = NAN, kMin = INFINITY, kMax = -INFINITY;
    bool dayWarn = false;
    int days = 0, boots = 0;
    float baseRo = NAN;
    std::vector<float> dr; // fast-path dR of the day's minutes
};

struct Result {
    uint8_t maxLvl = 0;
    int64_t first[4]{}, firstF[4]{}; // any time / at or after the fault
    std::map<int64_t, Day> days;
};

static bool g_verbose = false, g_days = false, g_rtcLost = false, g_fitLost = false;
static std::vector<int64_t> g_reboots; // [ms]

template<class G> struct Rtc {
    static constexpr bool has = requires(G &g, typename G::DayRtc &r) { g.saveDay(r); g.restoreDay(r); };
};

template<class G> struct FitRtc {
    static constexpr bool has = requires(G &g, typename G::FitRtc &r) { g.saveFit(r); g.restoreFit(r); };
};

template<class G> static void rtcSave(const G &g, std::vector<uint8_t> &buf, std::vector<uint8_t> &fit) {
    if constexpr (Rtc<G>::has) {
        typename G::DayRtc r;
        g.saveDay(r);
        buf.assign((const uint8_t *) &r, (const uint8_t *) &r + sizeof r);
    }
    if constexpr (FitRtc<G>::has) {
        typename G::FitRtc r;
        g.saveFit(r);
        fit.assign((const uint8_t *) &r, (const uint8_t *) &r + sizeof r);
    }
}

template<class G> static bool rtcRestore(G &g, const std::vector<uint8_t> &buf) {
    if constexpr (Rtc<G>::has) {
        typename G::DayRtc r;
        if (buf.size() != sizeof r) return false;
        memcpy(&r, buf.data(), sizeof r);
        return g.restoreDay(r);
    }
    return false;
}

template<class G> static bool fitRestore(G &g, const std::vector<uint8_t> &buf) {
    if constexpr (FitRtc<G>::has) {
        typename G::FitRtc r;
        if (buf.size() != sizeof r) return false;
        memcpy(&r, buf.data(), sizeof r);
        return g.restoreFit(r);
    }
    return false;
}

template<class G> static float daySd(const G &g) {
    if constexpr (requires { g.dayCurrentSd(); }) return g.dayCurrentSd();
    return NAN;
}

static Result run(WireGuard &g, float dR, int64_t faultMs, int64_t startMs) {
    Result r;
    WireResistance w;
    const auto conf = g.conf;
    const uint8_t paths = g.paths;
    WireGuard::Blob flash{};
    bool haveFlash = false;
    std::vector<uint8_t> rtc, fit;
    size_t ri = 0;
    int64_t bootMs = 0, rtcLastMs = 0;
    uint32_t lastTryS = 0;
    int boots = 0;
    const auto sweeps = fitSweeps(dR, faultMs);
    size_t si = 0, mi = 0, bi = 0, di = 0;
    int64_t t0 = std::max(mp.front().t, startMs) / 1000 * 1000;
    while (mi < mp.size() && mp[mi].t < t0) ++mi;
    while (bi < bms.size() && bms[bi].t < t0) ++bi;
    while (si < sweeps.size() && sweeps[si].tEnd < t0) ++si;
    while (di < dev.size() && dev[di].t < t0) ++di;
    float vpack = NAN, ibat = NAN, prevI = NAN, ro = NAN, uo = NAN, io = NAN;
    int64_t vpackT = 0, ibatT = 0, wireLastT = 0, lastM = 0;
    uint32_t roStamp = 0, lastCm = 0;
    uint8_t lastLvl = 0;
    bool lastRb = false;
    float lastFastPw = NAN;
    while (ri < g_reboots.size() && g_reboots[ri] <= t0) ++ri;
    bootMs = t0 - 8000;
    for (int64_t t = t0; t <= mp.back().t; t += 3000) {
        if (ri < g_reboots.size() && g_reboots[ri] <= t) {
            ++ri;
            ++boots;
            g.~WireGuard();
            new(&g) WireGuard();
            g.conf = conf;
            g.paths = paths;
            const bool lb = haveFlash && g.load(&flash, sizeof flash), lr = !g_rtcLost && rtcRestore(g, rtc),
                       lf = !g_rtcLost && !g_fitLost && fitRestore(g, fit);
            if (g_days)
                printf("    %s restart: baseline %s (%u days), day %s, regression %s\n", ts(t), lb ? "loaded" : "none",
                       g.dayCount(), lr ? "restored" : "lost", lf ? "restored" : "lost");
            w = WireResistance{};
            ro = NAN; roStamp = 0; prevI = NAN;
            bootMs = t - 8000;
            lastTryS = 0;
            rtcLastMs = t;
        }
        while (mi < mp.size() && mp[mi].t <= t) {
            uo = mp[mi].uo + (mp[mi].t >= faultMs ? dR * mp[mi].i : 0.f);
            io = mp[mi].i; lastM = mp[mi].t; ++mi;
        }
        while (bi < bms.size() && bms[bi].t <= t) {
            if (bms[bi].k == 'V') { vpack = bms[bi].v; vpackT = bms[bi].t; }
            else { ibat = bms[bi].v; ibatT = bms[bi].t; }
            ++bi;
        }
        while (si < sweeps.size() && sweeps[si].tEnd <= t) {
            if (sweeps[si].tEnd > bootMs) { ro = sweeps[si].r; roStamp = (uint32_t) ((sweeps[si].tEnd - bootMs) / 1000); }
            ++si;
        }
        auto &day = r.days[t / 86400000];
        while (di < dev.size() && dev[di].t <= t) {
            if (dev[di].pw > day.devPwMax) day.devPwMax = dev[di].pw;
            day.devWfPos += dev[di].wf > 0;
            ++di;
        }
        const uint32_t nowS = (uint32_t) ((t - bootMs) / 1000); // uptime, as on the device
        const bool have = t - lastM < 5000 && std::isfinite(io);
        const float iout = have ? io : 0.f;
        const bool running = have && iout > 0.3f;
        if (vpackT != wireLastT) { // BatteryCharger::updateWire
            wireLastT = vpackT;
            const bool use = running && std::llabs(vpackT - ibatT) < 2000;
            if (use && std::fabs(iout - prevI) < 0.5f) {
                w.add(uo - vpack, iout, ibat, nowS);
                g.frame(uo - vpack, iout, ibat);
            }
            prevI = use ? iout : NAN;
        }
        const float roNow = roStamp && nowS - roStamp <= OutputImpedance::STALE_S ? ro : NAN;
        const auto pre = g.blob();
        const uint8_t preN = g.daySamples(), preRo = g.daySweeps();
        const float preSd = daySd(g);
        g.tick(nowS, have ? iout : NAN, running, w.own(nowS), w.offset(nowS), w.shared(nowS), roNow, roStamp);
        if (g_days && (memcmp(&pre, &g.blob(), sizeof pre) || (preN && !g.daySamples()) || (preRo && !g.daySweeps())))
            printf("    %s day close: %u Rc samples (I sd %.1f A), %u sweeps -> days %u, Rc days %u Ro days %u, "
                   "base Rc %.2f Ro %.2f mOhm%s\n", ts(t), preN, preSd, preRo, g.dayCount(), g.blob().nRc,
                   g.blob().nRo, g.baseRc() * 1e3, g.baseRo() * 1e3, g.learning() ? " (learning)" : "");
        if (g.saveDue(running) && (!lastTryS || nowS - lastTryS >= 600)) {
            lastTryS = nowS;
            flash = g.blob();
            haveFlash = true;
            g.saved();
        }
        if (t - rtcLastMs >= 10000) {
            rtcLastMs = t;
            rtcSave(g, rtc, fit);
        }
        auto &dd = r.days[t / 86400000];
        dd.days = g.dayCount();
        dd.boots = boots;
        dd.baseRo = g.baseRo();

        day.iMax = std::max(day.iMax, iout);
        if (g.rcPath().fresh) day.rcMax = std::max(day.rcMax, g.rcPath().pw);
        if (g.roPath().fresh) day.roMax = std::max(day.roMax, g.roPath().pw);
        if (std::isfinite(g.pw())) day.allMax = std::max(day.allMax, g.pw());
        const auto f = g.fastPath();
        if (f.fresh && f.pw != lastFastPw) {
            lastFastPw = f.pw;
            ++day.fastMin;
            day.fastWarnMin += f.pw >= g.conf.pWarn && f.dr >= g.conf.drMin;
            day.pwMax = std::max(day.pwMax, f.pw);
            day.dr.push_back(f.dr);
            if (std::isfinite(g.slope())) { day.kMin = std::min(day.kMin, g.slope()); day.kMax = std::max(day.kMax, g.slope()); }
            if (g_verbose && f.pw > 1.2f)
                printf("    fast %s Pw %.2f dR %.2f I %.1f  offset %.1f mV slope %.2f mΩ\n", ts(t), f.pw, f.dr * 1e3,
                       iout, g.ofs() * 1e3, g.slope() * 1e3);
        }
        if (nowS / 60 != lastCm) {
            lastCm = nowS / 60;
            ++day.lvlMin[g.level()];
            day.anyMin += g.level() || g.rbWarn() || g.dayWarn();
        }
        day.dayWarn |= g.dayWarn();
        day.baseA = g.baseA(); day.baseRc = g.baseRc(); day.baseRb = g.baseRb();
        if (g.level() != lastLvl || g.rbWarn() != lastRb) {
            if (g_verbose) {
                auto c = g.rcPath(), o = g.roPath();
                printf("    %s lvl %u->%u rb %d  fast %d %.2fmΩ %.2fW  rc %d %.2fmΩ %.2fW  ro %d %.2fmΩ %.2fW  Iref %.1f I %.1f\n",
                       ts(t), lastLvl, g.level(), g.rbWarn(), f.fresh, f.dr * 1e3, f.pw, c.fresh, c.dr * 1e3, c.pw,
                       o.fresh, o.dr * 1e3, o.pw, g.iRef(), iout);
            }
            lastLvl = g.level(); lastRb = g.rbWarn();
        }
        for (uint8_t l = 1; l <= g.level(); ++l) {
            if (!r.first[l]) r.first[l] = t;
            if (!r.firstF[l] && t >= faultMs) r.firstF[l] = t;
        }
        r.maxLvl = std::max(r.maxLvl, g.level());
    }
    return r;
}

static float med(std::vector<float> v) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

int main(int argc, char **argv) {
    float dR = 0;
    int64_t faultMs = INT64_MAX, startMs = 0;
    bool prime = false;
    uint8_t paths = 7;
    int a = 1;
    for (; a < argc && argv[a][0] == '-'; ++a) {
        if (!strcmp(argv[a], "-f") && a + 1 < argc) {
            const char *s = argv[++a];
            dR = (float) atof(s) * 1e-3f;
            faultMs = atoll(strchr(s, '@') + 1) * 1000;
        } else if (!strcmp(argv[a], "-s") && a + 1 < argc) startMs = atoll(argv[++a]) * 1000;
        else if (!strcmp(argv[a], "-p")) prime = true;
        else if (!strcmp(argv[a], "-P") && a + 1 < argc) paths = (uint8_t) atoi(argv[++a]);
        else if (!strcmp(argv[a], "-v")) g_verbose = true;
        else if (!strcmp(argv[a], "-d")) g_days = true;
        else if (!strcmp(argv[a], "-R")) g_rtcLost = true;
        else if (!strcmp(argv[a], "-F")) g_fitLost = true;
        else if (!strcmp(argv[a], "-r") && a + 1 < argc) {
            const char *s = argv[++a];
            FILE *f = *s == '@' ? fopen(s + 1, "r") : nullptr;
            if (*s == '@' && !f) { fprintf(stderr, "no %s\n", s + 1); return 2; }
            char buf[64];
            while (f ? fscanf(f, "%63s", buf) == 1 : *s) {
                const char *x = f ? buf : s;
                g_reboots.push_back(atoll(x) * 1000);
                if (!f) { s = strchr(s, ','); s = s ? s + 1 : ""; }
            }
            if (f) fclose(f);
            std::sort(g_reboots.begin(), g_reboots.end());
        }
        else { fprintf(stderr, "bad option %s\n", argv[a]); return 2; }
    }
    if (a >= argc) { fprintf(stderr, "usage: %s [-f mOhm@unixS] [-s unixS] [-p] [-P mask] [-v] datadir...\n", argv[0]); return 2; }
    for (; a < argc; ++a) load(argv[a]);
    auto byT = [](auto &x, auto &y) { return x.t < y.t; };
    std::sort(mp.begin(), mp.end(), byT);
    std::sort(bms.begin(), bms.end(), byT);
    std::sort(dev.begin(), dev.end(), byT);
    fprintf(stderr, "mppt %zu bms %zu sweep samples %zu device wf %zu, %s .. %s\n", mp.size(), bms.size(), sw.size(),
            dev.size(), ts(mp.front().t), ts(mp.back().t));

    WireGuard g;
    g.conf.mode = 2;
    g.paths = paths;
    if (prime) {
        WireGuard p;
        p.conf.mode = 2;
        const auto rb = g_reboots;
        g_reboots.clear();
        run(p, 0, INT64_MAX, startMs);
        g_reboots = rb;
        auto b = p.blob();
        b.days = std::max<uint8_t>(b.days, WireGuard::LEARN_DAYS);
        g.load(&b, sizeof b);
        printf("primed baseline: Rc %.2f a %.1fmV Rb %.2f Ro %.2f mΩ (Rc days %u, Ro days %u)\n", g.baseRc() * 1e3,
               g.baseA() * 1e3, g.baseRb() * 1e3, g.baseRo() * 1e3, b.nRc, b.nRo);
    }
    const auto r = run(g, dR, faultMs, startMs);
    printf("day    Imax  max Pw: fast    Rc    Ro   all | fast min>=warn  fastMin  lvl>=1  any>=1 [min] | slope [mΩ] med dR dayWarn | device Pw wf>0 | base a[mV] Rc Rb Ro [mΩ] days boots\n");
    float pwMax = -INFINITY;
    int warnMin = 0, lvlMin = 0, anyMin = 0;
    for (auto &[d, x]: r.days) {
        const int l = x.lvlMin[1] + x.lvlMin[2] + x.lvlMin[3];
        printf("%s  %5.1f  %12.2f %5.2f %5.2f %5.2f | %14d %8d %7d %7d       | %5.1f..%5.1f %6.2f %3d     | %9.2f %4d | %6.1f %5.2f %5.2f %5.2f %4d %5d\n",
               ts(d * 86400000, "%m-%d"), x.iMax, x.pwMax, x.rcMax, x.roMax, x.allMax, x.fastWarnMin, x.fastMin, l,
               x.anyMin, x.kMin * 1e3, x.kMax * 1e3, med(x.dr) * 1e3, (int) x.dayWarn, x.devPwMax, x.devWfPos, x.baseA * 1e3,
               x.baseRc * 1e3, x.baseRb * 1e3, x.baseRo * 1e3, x.days, x.boots);
        anyMin += x.anyMin;
        pwMax = std::max(pwMax, x.pwMax);
        warnMin += x.fastWarnMin;
        lvlMin += l;
    }
    printf("total: fast Pw max %.2f W, fast minutes >= warn %d, minutes at level >= 1: %d (any warning incl. shared/day: %d), max level %u, first warn %s derate %s trip %s\n",
           pwMax, warnMin, lvlMin, anyMin, r.maxLvl, r.first[1] ? ts(r.first[1]) : "-", r.first[2] ? ts(r.first[2]) : "-",
           r.first[3] ? ts(r.first[3]) : "-");
    if (faultMs != INT64_MAX)
        for (int l = 1; l <= 3; ++l)
            printf("  +%.1f mΩ from %s: level %d %s%.1f min\n", dR * 1e3, ts(faultMs), l, r.firstF[l] ? "after " : "never ",
                   r.firstF[l] ? (r.firstF[l] - faultMs) / 60000. : NAN);
    return 0;
}
