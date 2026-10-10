// Host-side tests for src/usb/vfat.{h,cpp} (emulated FAT12 config drive).
//
// Build & run:
//   clang++ -std=gnu++20 -I src -o /tmp/vfat-test test/host-stub/vfat-test.cpp src/usb/vfat.cpp && /tmp/vfat-test
//
// Image round trip with a real FAT driver (macOS, see vfat-hdiutil.sh):
//   /tmp/vfat-test gen <confdir> <img>      write the volume built from confdir
//   /tmp/vfat-test apply <confdir> <img> [outdir]  rebuild from confdir, feed every img sector through write(),
//                                                  print the diff, write upserted files to outdir

#include <cassert>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sstream>

#include "usb/vfat.h"

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
    Volume vol;
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
    Volume vol(4096);
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

static uint16_t rootEntryFor(Volume &vol, const char *shortPrefix, uint32_t &lba, uint32_t &off) {
    uint8_t buf[SECTOR];
    for (lba = Volume::ROOT; lba < Volume::DATA; ++lba) {
        vol.read(lba, buf);
        for (off = 0; off < SECTOR; off += 32)
            if (buf[off + 11] != 0x0F && !memcmp(buf + off, shortPrefix, strlen(shortPrefix)))
                return buf[off + 26] | (buf[off + 27] << 8);
    }
    assert(false);
    return 0;
}

static void testRoundTrip() {
    Volume vol;
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

static void testInvalidNamesSkipped() {
    Volume vol;
    assert(!vol.build({{"ok.conf", "x"}, {"bad name.conf", "x"}, {"notes.txt", "x"}, {".hidden.conf", "x"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1);
}

static void testEditAndSizeMismatch() {
    Volume vol;
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t c = rootEntryFor(vol, "BOARD~", lba, off);
    uint8_t buf[SECTOR];

    // edit content in place (same size)
    vol.read(Volume::DATA + c - 2, buf);
    buf[2] = '9';
    assert(vol.write(Volume::DATA + c - 2, buf));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err));
    auto d = diff(vol.baseline(), host);
    assert(d.upsert.size() == 1 && d.upsert[0].name == "board.conf" && d.upsert[0].data == "a=9\nb=2\n");
    assert(d.removed.empty());

    // size larger than chain
    vol.read(lba, buf);
    buf[off + 28] = 0x00;
    buf[off + 29] = 0x04; // 1024 bytes, chain has 1 cluster
    vol.write(lba, buf);
    assert(!vol.collect(host, err));
    assert(err.find("chain") != std::string::npos);
}

static void testCrossLinkAndBadCluster() {
    Volume vol;
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t cBoard = rootEntryFor(vol, "BOARD~", lba, off);
    uint32_t lba2, off2;
    rootEntryFor(vol, "EXACT~", lba2, off2);
    uint8_t buf[SECTOR];
    vol.read(lba2, buf);
    buf[off2 + 26] = cBoard; // exact512 now shares board's cluster
    buf[off2 + 27] = cBoard >> 8;
    vol.write(lba2, buf);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));

    vol.build(sample(), 1);
    vol.read(lba, buf);
    buf[off + 26] = 0xFF;
    buf[off + 27] = 0x0F; // out of range
    vol.write(lba, buf);
    assert(!vol.collect(host, err));
}

static void testDeleteAndCreate() {
    Volume vol;
    vol.build(sample(), 1);
    uint32_t lba, off;
    rootEntryFor(vol, "EMPTY~", lba, off);
    uint8_t buf[SECTOR];
    vol.read(lba, buf);
    buf[off] = 0xE5; // 8.3 entry only: its LFN entries become orphans
    vol.write(lba, buf);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err) && err == "orphan long name");
    for (uint32_t o = off; o >= 32 && buf[o - 32 + 11] == 0x0F; o -= 32) buf[o - 32] = 0xE5;
    vol.write(lba, buf);
    assert(vol.collect(host, err));
    auto d = diff(vol.baseline(), host);
    assert(d.upsert.empty() && d.removed.size() == 1 && d.removed[0] == "empty.conf");

    // case-only rename is no change
    auto h2 = host;
    h2[0].name = "BOARD.conf";
    d = diff(vol.baseline(), h2);
    assert(d.upsert.empty() && d.removed.size() == 1);

    d = diff({}, {{"new.conf", "x"}});
    assert(d.upsert.size() == 1 && d.removed.empty());
}

static void testBrokenLfnBlocks() {
    Volume vol;
    vol.build(sample(), 1);
    uint32_t lba, off;
    rootEntryFor(vol, "CHARG~", lba, off);
    uint8_t buf[SECTOR];
    vol.read(lba, buf);
    assert(off >= 32 && buf[off - 32 + 11] == 0x0F);
    buf[off - 32 + 13] ^= 0xFF; // LFN checksum mismatch -> only the SFN CHARG~2.CON remains
    vol.write(lba, buf);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err) && err == "long name checksum");
}

// Every way a damaged entry of an existing conf could read as "file gone" must block instead.
static void testMalformedEntriesBlock() {
    auto mutate = [](auto &&fn) {
        Volume vol;
        vol.build(sample(), 1);
        uint32_t lba, off;
        rootEntryFor(vol, "BOARD~", lba, off);
        uint8_t buf[SECTOR];
        vol.read(lba, buf);
        fn(buf, off);
        vol.write(lba, buf);
        std::vector<File> host;
        std::string err;
        bool ok = vol.collect(host, err);
        return ok ? diff(vol.baseline(), host).removed.size() : (size_t) 100;
    };
    // control: untouched volume has no removals
    assert(mutate([](uint8_t *, uint32_t) {}) == 0);
    // empty long name (terminator in the first char)
    assert(mutate([](uint8_t *b, uint32_t o) { b[o - 32 + 1] = 0; b[o - 32 + 2] = 0; }) == 100);
    // reserved ordinal bit, LFN type, LFN cluster field
    assert(mutate([](uint8_t *b, uint32_t o) { b[o - 32] |= 0x20; }) == 100);
    assert(mutate([](uint8_t *b, uint32_t o) { b[o - 32 + 12] = 1; }) == 100);
    assert(mutate([](uint8_t *b, uint32_t o) { b[o - 32 + 26] = 1; }) == 100);
    // a char after the terminator that isn't 0xFFFF padding
    assert(mutate([](uint8_t *b, uint32_t o) { b[o - 32 + 30] = 'x'; b[o - 32 + 31] = 0; }) == 100);
    // directory bit on the conf entry
    assert(mutate([](uint8_t *b, uint32_t o) { b[o + 11] |= 0x10; }) == 100);
}

static void testJunkCrossLinkBlocks() {
    Volume vol;
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t c = rootEntryFor(vol, "BOARD~", lba, off);
    // append a junk 8.3 file "JUNK.TXT" sharing board.conf's cluster
    uint8_t buf[SECTOR];
    uint32_t jl = 0, jo = 0;
    for (uint32_t l = Volume::ROOT; l < Volume::DATA && !jl; ++l) {
        vol.read(l, buf);
        for (uint32_t o = 0; o < SECTOR; o += 32) if (!buf[o]) { jl = l; jo = o; break; }
    }
    vol.read(jl, buf);
    memcpy(buf + jo, "JUNK    TXT", 11);
    buf[jo + 11] = 0x20;
    buf[jo + 26] = c;
    buf[jo + 27] = c >> 8;
    buf[jo + 28] = 8;
    vol.write(jl, buf);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
}

static void testOverlayCap() {
    Volume vol(2);
    vol.build(sample(), 1);
    uint8_t buf[SECTOR];
    memset(buf, 0xAB, SECTOR);
    uint32_t free0 = Volume::DATA + 100;
    assert(vol.write(free0, buf));
    assert(vol.write(free0 + 1, buf));
    assert(!vol.write(free0 + 2, buf)); // full: error, not silently dropped
    assert(vol.write(free0, buf)); // rewrite of a held sector still works
    uint8_t z[SECTOR] = {};
    assert(vol.write(free0 + 3, z)); // unchanged content needs no slot
    assert(vol.overlaySectors() == 2);
    assert(!vol.write(Volume::TOTAL_SECTORS, buf));
}

static void testFat2Alias() {
    Volume vol;
    vol.build(sample(), 1);
    uint8_t a[SECTOR], b[SECTOR];
    for (uint32_t i = 0; i < Volume::FAT_SECTORS; ++i) {
        vol.read(Volume::FAT1 + i, a);
        vol.read(Volume::FAT2 + i, b);
        assert(!memcmp(a, b, SECTOR));
    }
}

static void testConfText() {
    std::string err;
    assert(validConfText("", err));
    assert(validConfText("# c\n\nkey=1\r\nk2 = v w # x\n  a.b-c_d=\n", err));
    assert(!validConfText("key 1\n", err) && err.find("line 1") == 0);
    assert(!validConfText("a=1\n=2\n", err) && err.find("line 2") == 0);
    assert(!validConfText("a b=1\n", err));
    assert(!validConfText("a=\xC3\xA4\n", err));
    assert(!validConfText(std::string(20000, '#'), err));
    // ConfFile reads 255-byte chunks: a comment longer than that would hide a live key in its tail
    assert(!validConfText("#" + std::string(254, ' ') + "pwm_freq=999\n", err));
    assert(validConfText(std::string(254, '#') + "\n", err));
}

int main(int argc, char **argv) {
    if (argc == 4 && !strcmp(argv[1], "gen")) return gen(argv[2], argv[3]);
    if (argc >= 4 && !strcmp(argv[1], "apply")) return apply(argv[2], argv[3], argc > 4 ? argv[4] : nullptr);
    testRoundTrip();
    testInvalidNamesSkipped();
    testEditAndSizeMismatch();
    testCrossLinkAndBadCluster();
    testDeleteAndCreate();
    testBrokenLfnBlocks();
    testMalformedEntriesBlock();
    testJunkCrossLinkBlocks();
    testOverlayCap();
    testFat2Alias();
    testConfText();
    printf("vfat-test: all passed\n");
    return 0;
}
