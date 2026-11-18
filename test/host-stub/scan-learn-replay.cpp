// Replays field sweeps through PeakScan's substring learner (sweepSample, onSweepMpp, learn), one PeakScan per
// device. stdin: "S <dev> <boot> <time> <Voc> <Vmpp> <Pmpp>", then "<Vin> <P>" per sample, Vin falling, then "E".
// A new <boot> starts a fresh PeakScan with the saved count. Saved at the next sweep start (converter off), and at the
// reboot (systemRestart()) unless arg "crash" (every reboot unplanned).
// Build:
//   clang++ -std=gnu++20 -O2 -include test/host-stub/esp/sim_prelude.h -I test/host-stub/esp -I src \
//       test/host-stub/scan-learn-replay.cpp -o /tmp/scan-learn-replay
#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include "esp_log.h"
#include "peak_scan.h"

int main(int argc, char **argv) {
    const bool crash = argc > 1 && !strcmp(argv[1], "crash");
    const float pMin = 25 * 0.002f * 1000; // 25 sweepMinPower() at p_max 1000 W
    struct Dev {
        PeakScan s;
        std::string boot;
        uint8_t saved = 0;
        int shaded = 0, covered = 0, wrong = 0;
    };
    std::map<std::string, Dev> devs;
    char line[256], dev[32], boot[32], tm[16];
    float voc, vm, pm, v, p;
    Dev *d = nullptr;
    while (fgets(line, sizeof line, stdin)) {
        if (sscanf(line, "S %31s %31s %15s %f %f %f", dev, boot, tm, &voc, &vm, &pm) == 6) {
            d = &devs[dev];
            if (d->boot != boot) {
                if (!crash && d->s.unsaved) d->saved = d->s.n;
                d->s = PeakScan{}, d->s.n = d->saved, d->boot = boot;
            }
            if (d->s.unsaved) d->saved = d->s.n, d->s.unsaved = false; // sweep start: tracker.conf write
            d->s.onSweep(voc, 0);
        } else if (line[0] == 'E' && d) {
            auto &s = d->s;
            const uint8_t n0 = s.n, nc = s.nc;
            s.onSweepMpp(vm, pm, pMin);
            const bool bright = s.pending;
            s.learn();
            int q = 0;
            for (int i = 0; i < std::min<int>(nc, 6); ++i) q += s.pc[i] >= 0.3f * pm;
            const bool shaded = bright && q >= 2;
            d->shaded += shaded, d->covered += shaded && s.n >= 2, d->wrong += s.n && s.n != 6;
            printf("%s %s %s Voc %.1f MPP %.0f W %.1f V maxima %u:", dev, boot, tm, voc, pm, vm, nc);
            for (int i = 0; i < std::min<int>(nc, 6); ++i)
                printf(" %.1fV/%.0fW=%.2f%s", s.vc[i], s.pc[i], s.vc[i] / (0.82f * voc) * 6, s.pc[i] < 0.3f * pm ? "(small)" : "");
            printf(" |%s%s n %u->%u%s\n", bright ? "" : nc > 6 ? " (overflow)" : " (dim)", shaded ? " shaded" : "", n0, s.n,
                   s.n != n0 ? " LEARNED" : "");
            d = nullptr;
        } else if (d && sscanf(line, "%f %f", &v, &p) == 2) {
            d->s.sweepSample(v, p);
        }
    }
    for (auto &[k, x]: devs)
        printf("== %s: shaded sweeps %d, covered (n known after the sweep) %d, sweeps with n other than 6: %d, saved n %u\n",
               k.c_str(), x.shaded, x.covered, x.wrong, x.saved);
}
