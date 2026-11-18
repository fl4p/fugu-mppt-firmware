// Replays field sweeps through PeakScan's substring learner (sweepSample + onSweepMpp), one PeakScan per device.
// stdin: "S <dev> <date> <time> <Voc> <Vmpp> <Pmpp>", then "<Vin> <P>" per sample, Vin falling, then "E".
// arg "day": a fresh PeakScan per device and date (a reboot every night). Build:
//   clang++ -std=gnu++20 -O2 -include test/host-stub/esp/sim_prelude.h -I test/host-stub/esp -I src \
//       test/host-stub/scan-learn-replay.cpp -o /tmp/scan-learn-replay
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <cassert>
#include "esp_log.h"
#include "peak_scan.h"

int main(int argc, char **argv) {
    const bool perDay = argc > 1 && !strcmp(argv[1], "day");
    const float pMin = 25 * 0.002f * 1000; // 25 sweepMinPower() at p_max 1000 W
    std::map<std::string, PeakScan> scans;
    std::map<std::string, std::string> days;
    char line[256], dev[32], date[16], tm[16];
    float voc, vm, pm, v, p;
    PeakScan *s = nullptr;
    while (fgets(line, sizeof line, stdin)) {
        if (sscanf(line, "S %31s %15s %15s %f %f %f", dev, date, tm, &voc, &vm, &pm) == 6) {
            if (perDay && days[dev] != date) scans[dev] = PeakScan{}, days[dev] = date;
            s = &scans[dev];
            s->onSweep(voc, 0);
        } else if (line[0] == 'E' && s) {
            const uint8_t cand0 = s->cand, n0 = s->n, nc = s->nc;
            float pk[6], vk[6];
            for (int i = 0; i < std::min<int>(nc, 6); ++i) pk[i] = s->pc[i], vk[i] = s->vc[i];
            s->onSweepMpp(vm, pm, pMin);
            printf("%s %s %s Voc %.1f MPP %.0f W %.1f V peaks %u:", dev, date, tm, voc, pm, vm, nc);
            for (int i = 0; i < std::min<int>(nc, 6); ++i)
                printf(" %.1fV/%.0fW=%.2f%s", vk[i], pk[i], vk[i] / (0.82f * voc) * 6, pk[i] < 0.3f * pm ? "(small)" : "");
            int q = 0;
            for (int i = 0; i < std::min<int>(nc, 6); ++i) q += pk[i] >= 0.3f * pm;
            printf(" | maxima %d%s cand %u->%u n %u->%u%s\n", q, pm < pMin ? " (dim)" : nc > 6 ? " (overflow)" : "",
                   cand0, s->cand, n0, s->n, s->n != n0 ? " LEARNED" : "");
            s = nullptr;
        } else if (s && sscanf(line, "%f %f", &v, &p) == 2) {
            s->sweepSample(v, p);
        }
    }
}
