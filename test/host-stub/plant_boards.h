#pragma once
// Host-sim plants of the field boards, fitted to the LTspice half-bridge bench
// (doc/research/sim-plant-spice-revalidation.md, reference rows in doc/research/vconv-spice-data/). Each field maps
// to a vconv.conf key.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

struct PlantBoard {
    const char *name;
    float l0Conf, lPlant; // coil.conf L0 (what the firmware believes), plant L
    float ringF, ringV, ringCexp, ringQ, ringCt; // free ring frequency at Vin = ringV, C ~ Vin^-ringCexp, Q, edge C ratio
    float qHigh; // Q of the "-hq" variant: ripple of the undamped bench (no core loss)
    float vf, tDead, tHsExt, rHs, rLs, tSw; // diode_vf, t_dead, t_hs_ext, r_hs, r_ls, t_sw
    float cIn; // plant_c_in: 2 x 470 uF +20 % and the panel's own; fry's dusk sweeps fit 1.0-1.5 mF output-referred
    float rectOffsetNs; // coil.conf rect_offset_ns (firmware side; mppt-sim applies it to the MCPWM preset, not in rcal*)

    [[nodiscard]] float ringC() const { return ringF > 0 ? 1.f / (39.478418f * ringF * ringF * lPlant) : 0.f; }

    // vconv.conf pairs; ring = false: ring_c 0, the rest stays
    [[nodiscard]] std::vector<std::pair<std::string, std::string>> conf(bool ring = true) const {
        auto s = [](float v) { char b[24]; snprintf(b, sizeof b, "%g", v); return std::string(b); };
        return {{"ring_c", s(ring ? ringC() : 0.f)}, {"ring_q", s(ringQ)}, {"ring_v", s(ringV)}, {"ring_cexp", s(ringCexp)}, {"ring_ct", s(ringCt)},
                {"diode_vf", s(vf)}, {"t_dead", s(tDead)}, {"t_hs_ext", s(tHsExt)}, {"r_hs", s(rHs)}, {"r_ls", s(rLs)},
                {"t_sw", s(tSw)}, {"plant_c_in", s(cIn)}};
    }
};

// fry: 76 uH plant, ring 233 kHz at 75 V (6.1 nF), ~195 kHz at 45 V; flat: 51 uH, 320 kHz at 60 V, ~295 kHz at 45 V.
// C ~ Vin^-0.6 and the edge C ratio 0.75 from the bench's free ring vs rise transit; Q from the field DCM sweeps;
// dead time = UCC21330B with RDT 20k; r_* and t_sw fitted to the bench's CCM efficiency.
// Firmware side as live: MCPWM 160 MHz, pwm_freq 39000 (4103 counts), no MCU dead time, boot_refresh_ns default 2000,
// coil.conf L0 80 / 56 uH. rect_offset_ns 0: the plant has no LS turn-off delay, so fry's live -1312 ns (rect-cal
// 2026-11-15) would turn the LS off ~1.5 us before this plant's zero crossing; opt in with SIM_PLANT=fry:rect_offset_ns=-1312.
inline PlantBoard kPlantBoards[] = {
    {"fry", 80e-6f, 76e-6f, 233e3f, 75.f, 0.6f, 25.f, 0.75f, 150.f, 0.7f, 185e-9f, 0.f, 0.008f, 0.014f, 100e-9f, 1.2e-3f, 0.f},
    {"flat", 56e-6f, 51e-6f, 320e3f, 60.f, 0.6f, 15.f, 0.75f, 100.f, 0.7f, 185e-9f, 0.f, 0.012f, 0.018f, 100e-9f, 1.2e-3f, 0.f},
};

inline PlantBoard *plantBoard(const char *name) {
    for (auto &b: kPlantBoards) if (!strcmp(b.name, name)) return &b;
    return nullptr;
}

inline PlantBoard highQ(PlantBoard b) {
    b.ringQ = b.qHigh;
    return b;
}

// SIM_PLANT="fry:t_sw=30e-9,flat:r_hs=0.02" overrides fields (tuning); "fry:ideal=1" drops ring, diode, timing and losses
inline void plantBoardsFromEnv() {
    const char *e = getenv("SIM_PLANT");
    if (!e) return;
    std::string all = e;
    for (size_t i = 0, j; i < all.size(); i = j + 1) {
        j = all.find(',', i);
        if (j == std::string::npos) j = all.size();
        const std::string kv = all.substr(i, j - i);
        const size_t c = kv.find(':'), q = kv.find('=');
        if (c == std::string::npos || q == std::string::npos) continue;
        PlantBoard *b = plantBoard(kv.substr(0, c).c_str());
        const std::string k = kv.substr(c + 1, q - c - 1);
        const float v = (float) atof(kv.c_str() + q + 1);
        if (!b) continue;
        if (k == "ideal") b->ringF = b->vf = b->tDead = b->tHsExt = b->rHs = b->rLs = b->tSw = 0;
        else if (k == "L") b->lPlant = v;
        else if (k == "ring_f") b->ringF = v;
        else if (k == "ring_v") b->ringV = v;
        else if (k == "ring_cexp") b->ringCexp = v;
        else if (k == "ring_q") b->ringQ = v;
        else if (k == "ring_ct") b->ringCt = v;
        else if (k == "q_high") b->qHigh = v;
        else if (k == "diode_vf") b->vf = v;
        else if (k == "t_dead") b->tDead = v;
        else if (k == "t_hs_ext") b->tHsExt = v;
        else if (k == "r_hs") b->rHs = v;
        else if (k == "r_ls") b->rLs = v;
        else if (k == "t_sw") b->tSw = v;
        else if (k == "c_in") b->cIn = v;
        else if (k == "rect_offset_ns") b->rectOffsetNs = v;
        else fprintf(stderr, "SIM_PLANT: unknown key %s\n", k.c_str());
    }
}
