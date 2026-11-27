// Host-side tests for the firmware ".conf" policy (src/usb/conf_drive.h) and its use by the
// config drive (src/usb/usb_dev.cpp). The FAT12 core mechanics live in the package suite
// components/virtual-fat12/test/test_vfat.cpp, which test/host-stub/host-tests.sh also runs
// (vfat-core-test) so firmware's standard host run keeps that coverage.
//
// Build & run:
//   clang++ -std=gnu++20 -I components/virtual-fat12/include -I src -o /tmp/vfat-test \
//       test/host-stub/vfat-test.cpp components/virtual-fat12/src/vfat.cpp && /tmp/vfat-test
//   test/host-stub/host-tests.sh vfat-test vfat-core-test
//
// Image round trip with a real FAT driver (macOS, see vfat-hdiutil.sh):
//   /tmp/vfat-test gen <confdir> <img>      write the volume built from confdir
//   /tmp/vfat-test apply <confdir> <img> [outdir]  rebuild from confdir, feed every img sector through write(),
//                                                  print the diff, write upserted files to outdir

#ifdef NDEBUG
#undef NDEBUG // the tests call build()/collect() inside assert(), so assert() must stay active
#endif
#include <cassert>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sstream>

#include "vfat/vfat.h"
#include "usb/conf_drive.h"

using namespace vfat;

static std::vector<File> loadDir(const char *dir) {
    std::vector<File> v;
    DIR *d = opendir(dir);
    assert(d);
    while (auto *de = readdir(d)) {
        if (de->d_name[0] == '.') continue;
        std::ifstream in(std::string(dir) + "/" + de->d_name, std::ios::binary);
        std::stringstream ss;
        ss << in.rdbuf();
        v.push_back({de->d_name, ss.str()});
    }
    closedir(d);
    return v;
}

static int gen(const char *dir, const char *img) {
    Volume vol(128, confdrive::naming());
    if (!vol.build(loadDir(dir), 0x12345678)) fprintf(stderr, "warning: some files skipped\n");
    FILE *f = fopen(img, "wb");
    uint8_t buf[SECTOR];
    for (uint32_t l = 0; l < Volume::TOTAL_SECTORS; ++l) {
        assert(vol.read(l, buf));
        fwrite(buf, 1, SECTOR, f);
    }
    fclose(f);
    return 0;
}

static int apply(const char *dir, const char *img, const char *outDir) {
    Volume vol(4096, confdrive::naming());
    vol.build(loadDir(dir), 0x12345678);
    FILE *f = fopen(img, "rb");
    uint8_t buf[SECTOR];
    for (uint32_t l = 0; l < Volume::TOTAL_SECTORS; ++l) {
        assert(fread(buf, 1, SECTOR, f) == SECTOR);
        assert(vol.write(l, buf));
    }
    fclose(f);
    printf("overlay sectors: %zu\n", vol.overlaySectors());
    std::vector<File> host;
    std::string err;
    if (!vol.collect(host, err)) {
        printf("collect failed: %s\n", err.c_str());
        return 1;
    }
    auto d = diff(vol.baseline(), host);
    for (auto &u: d.upsert) {
        printf("upsert %s %zu\n", u.name.c_str(), u.data.size());
        if (outDir) std::ofstream(std::string(outDir) + "/" + u.name, std::ios::binary) << u.data;
    }
    for (auto &r: d.removed) printf("removed %s\n", r.c_str());
    return 0;
}

static std::vector<File> sample() {
    std::string big(1300, 'x');
    for (size_t i = 0; i < big.size(); i += 40) big[i] = '\n';
    return {
        {"board.conf", "a=1\nb=2\n"},
        {"charger.conf", big},
        {"empty.conf", ""},
        {"a-very_long-name-that-needs-three-lfn-entries.conf", "k=v\n"},
        {"exact512.conf", std::string(512, 'z')},
    };
}

// first free root slot (the 0x00 terminator); the following entry stays 0x00
static void freeRootSlot(Volume &vol, uint32_t &lba, uint32_t &off) {
    uint8_t buf[SECTOR];
    for (lba = Volume::ROOT; lba < Volume::DATA; ++lba) {
        vol.read(lba, buf);
        for (off = 0; off < SECTOR; off += 32)
            if (buf[off] == 0x00) return;
    }
    assert(false);
}

// write a bare 8.3 entry (no LFN) into the first free root slot
static void writeSfn(Volume &vol, const char *base, const char *ext, uint16_t cluster, uint32_t size) {
    uint32_t lba, off;
    freeRootSlot(vol, lba, off);
    uint8_t buf[SECTOR];
    vol.read(lba, buf);
    memset(buf + off, ' ', 11);
    memcpy(buf + off, base, strlen(base));
    memcpy(buf + off + 8, ext, strlen(ext));
    buf[off + 11] = 0x20;
    buf[off + 26] = cluster;
    buf[off + 27] = cluster >> 8;
    buf[off + 28] = size & 0xFF;
    buf[off + 29] = (size >> 8) & 0xFF;
    vol.write(lba, buf);
}

static void setFat(Volume &vol, uint32_t c, uint16_t v) {
    uint8_t buf[SECTOR];
    uint32_t sec = Volume::FAT1 + (c + c / 2) / SECTOR;
    uint32_t o = (c + c / 2) % SECTOR;
    vol.read(sec, buf);
    if (c & 1) {
        buf[o] = (buf[o] & 0x0F) | (v << 4);
        buf[o + 1] = v >> 4;
    } else {
        buf[o] = v;
        buf[o + 1] = (buf[o + 1] & 0xF0) | ((v >> 8) & 0x0F);
    }
    vol.write(sec, buf);
}

// The real confdrive::naming() must build and collect the same files the policy accepts; the core
// suite covers the layout with a local copy of the policy, this one pins the header the firmware uses.
static void testConfDriveRoundTrip() {
    Volume vol(128, confdrive::naming());
    assert(vol.build(sample(), 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err));
    assert(host.size() == 5);
    for (size_t i = 0; i < host.size(); ++i) {
        assert(host[i].name == sample()[i].name);
        assert(host[i].data == sample()[i].data);
    }
    assert(diff(vol.baseline(), host).empty());
}

static void testValidName() {
    assert(confdrive::validName("board.conf"));
    assert(confdrive::validName("a_b-c.conf"));
    assert(confdrive::validName(std::string(59, 'a') + ".conf")); // 64 chars
    assert(!confdrive::validName("notes.txt"));
    assert(!confdrive::validName(".hidden.conf"));
    assert(!confdrive::validName("bad name.conf"));
    assert(!confdrive::validName("a.con")); // shorter than 6
    assert(!confdrive::validName(std::string(60, 'a') + ".conf")); // 65 chars
}

// A malformed 8.3-only CON entry is tracked through the policy's SFN extension branch, so the real
// validName() (not the core suite's local copy) must reject it and collect() must block.
static void testCollectInvalidNameBlocks() {
    Volume vol(128, confdrive::naming());
    assert(vol.build({{"ok.conf", "x"}}, 1));
    writeSfn(vol, "AB#CD", "CON", 2000, 8); // tracked via the SFN branch, fails the policy check
    setFat(vol, 2000, 0xFFF);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("invalid name") != std::string::npos);
}

static void testSfnOnlyConfExtensionBlocks() {
    Volume vol(128, confdrive::naming());
    assert(vol.build({{"ok.conf", "x"}}, 1));
    writeSfn(vol, "A", "CON", 2000, 8); // 8.3-only CON entry: tracked, but "A.CON" fails the policy
    setFat(vol, 2000, 0xFFF);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("invalid name") != std::string::npos);
}

static void testConfText() {
    std::string err;
    assert(confdrive::validConfText("", err));
    assert(confdrive::validConfText("# c\n\nkey=1\r\nk2 = v w # x\n  a.b-c_d=\n", err));
    assert(!confdrive::validConfText("key 1\n", err) && err.find("line 1") == 0);
    assert(!confdrive::validConfText("a=1\n=2\n", err) && err.find("line 2") == 0);
    assert(!confdrive::validConfText("a b=1\n", err));
    assert(!confdrive::validConfText("a=\xC3\xA4\n", err));
    assert(!confdrive::validConfText(std::string(20000, '#'), err));
    // ConfFile reads 255-byte chunks: a comment longer than that would hide a live key in its tail
    assert(!confdrive::validConfText("#" + std::string(254, ' ') + "pwm_freq=999\n", err));
    assert(confdrive::validConfText(std::string(254, '#') + "\n", err));
}

// --- case-insensitive alias preflight (src/usb/usb_dev.cpp buildVolume) ---

static void testCaseAliasConflict() {
    std::string msg;
    assert(confdrive::caseAliasConflict({{"board.conf", ""}, {"BOARD.conf", ""}}, msg));
    assert(msg.find("board.conf") != std::string::npos && msg.find("BOARD.conf") != std::string::npos);
    assert(msg.find("rename or remove") != std::string::npos);
    // first conflicting pair wins, both names reported
    assert(confdrive::caseAliasConflict({{"a.conf", ""}, {"b.conf", ""}, {"A.CONF", ""}}, msg));
    assert(msg.find("a.conf") != std::string::npos && msg.find("A.CONF") != std::string::npos);

    assert(!confdrive::caseAliasConflict({}, msg));
    assert(!confdrive::caseAliasConflict({{"only.conf", ""}}, msg));
    assert(!confdrive::caseAliasConflict({{"board.conf", ""}, {"limits.conf", ""}}, msg));
    assert(!confdrive::caseAliasConflict({{"board.conf", ""}, {"board.old.conf", ""}}, msg));
}

// The alias pair the preflight reports is the one Volume::build() returns false for; the real
// confdrive::naming() must build and collect a distinct set normally.
static void testCaseAliasBuildIntegration() {
    Volume vol(96, confdrive::naming());
    assert(!vol.build({{"board.conf", "x"}, {"Board.conf", "y"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1 && host[0].name == "board.conf"); // first wins, alias dropped

    Volume ok(96, confdrive::naming());
    assert(ok.build({{"board.conf", "x"}, {"limits.conf", "y"}}, 1));
    assert(ok.collect(host, err) && host.size() == 2);
}

int main(int argc, char **argv) {
    if (argc == 4 && !strcmp(argv[1], "gen")) return gen(argv[2], argv[3]);
    if (argc >= 4 && !strcmp(argv[1], "apply")) return apply(argv[2], argv[3], argc > 4 ? argv[4] : nullptr);
    testConfDriveRoundTrip();
    testValidName();
    testCollectInvalidNameBlocks();
    testSfnOnlyConfExtensionBlocks();
    testConfText();
    testCaseAliasConflict();
    testCaseAliasBuildIntegration();
    printf("vfat-test: all passed\n");
    return 0;
}
