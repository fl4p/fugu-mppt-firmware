// Minimal host example: build a virtual FAT12 volume in memory, let a "host" overwrite one data
// sector, then collect() the files back and diff() against the baseline.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "vfat/vfat.h"

// policy: a tracked name is any name ending in ".conf"
static bool validConfName(const std::string &n) {
    if (n.size() < 6) return false;
    for (int i = 0; i < 5; ++i) {
        char c = n[n.size() - 5 + i];
        if (c >= 'A' && c <= 'Z') c += 32;
        if (c != ".conf"[i]) return false;
    }
    return true;
}

// first non-LFN, non-volume root entry's start cluster
static uint16_t firstFileCluster(vfat::Volume &vol) {
    uint8_t buf[vfat::SECTOR];
    for (uint32_t l = vfat::Volume::ROOT; l < vfat::Volume::DATA; ++l) {
        vol.read(l, buf);
        for (uint32_t o = 0; o < vfat::SECTOR; o += 32) {
            if (!buf[o]) return 0;
            if ((buf[o + 11] & 0x3F) == 0x0F || (buf[o + 11] & 0x08)) continue;
            return buf[o + 26] | (buf[o + 27] << 8);
        }
    }
    return 0;
}

int main() {
    vfat::Volume vol(128, vfat::Naming{".conf", validConfName});
    std::vector<vfat::File> files = {
            {"board.conf", "vin_max=60\n"},
            {"charger.conf", "cv_float=3.4\n"},
            {"notes.txt", "not tracked\n"},
    };
    if (!vol.build(files, 0x12345678)) printf("warning: some files were skipped\n");

    // read the whole volume back, sector by sector, as a host driver would
    std::vector<uint8_t> img(vol.TOTAL_SECTORS * vfat::SECTOR);
    for (uint32_t l = 0; l < vol.TOTAL_SECTORS; ++l) vol.read(l, &img[l * vfat::SECTOR]);

    // overwrite board.conf's first data sector in place (same size)
    uint16_t c = firstFileCluster(vol);
    uint32_t lba = vfat::Volume::DATA + c - 2;
    uint8_t buf[vfat::SECTOR];
    vol.read(lba, buf);
    memcpy(buf, "vin_max=61\n", 11);
    if (!vol.write(lba, buf)) printf("write failed\n");

    std::vector<vfat::File> host;
    std::string err;
    if (!vol.collect(host, err)) {
        printf("collect failed: %s\n", err.c_str());
        return 1;
    }
    auto d = vfat::diff(vol.baseline(), host);
    for (auto &u: d.upsert) printf("upsert %s (%zu bytes)\n", u.name.c_str(), u.data.size());
    for (auto &r: d.removed) printf("removed %s\n", r.c_str());
    if (d.empty()) printf("no changes\n");
    return 0;
}
