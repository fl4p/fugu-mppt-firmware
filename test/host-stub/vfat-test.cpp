// Host-side tests for the virtual-fat12 config drive (components/virtual-fat12) and its firmware
// ".conf" policy (src/usb/conf_drive.h).
//
// Build & run:
//   clang++ -std=gnu++20 -I components/virtual-fat12/include -I src -o /tmp/vfat-test \
//       test/host-stub/vfat-test.cpp components/virtual-fat12/src/vfat.cpp && /tmp/vfat-test
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

// the 8.3 name bytes of the first non-LFN, non-volume entry
static void firstSfn(Volume &vol, uint8_t out[11]) {
    uint8_t buf[SECTOR];
    for (uint32_t l = Volume::ROOT; l < Volume::DATA; ++l) {
        vol.read(l, buf);
        for (uint32_t o = 0; o < SECTOR; o += 32) {
            if (!buf[o]) break;
            if ((buf[o + 11] & 0x3F) == 0x0F || (buf[o + 11] & 0x08)) continue;
            memcpy(out, buf + o, 11);
            return;
        }
    }
    assert(false);
}

static void testRoundTrip() {
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

static void testInvalidNamesSkipped() {
    Volume vol(128, confdrive::naming());
    assert(!vol.build({{"ok.conf", "x"}, {"bad name.conf", "x"}, {"notes.txt", "x"}, {".hidden.conf", "x"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1);
}

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

static void testCollectBadCharLfnBlocks() {
    Volume vol(128, confdrive::naming());
    assert(vol.build({{"ok.conf", "x"}}, 1));
    uint32_t lba, off;
    rootEntryFor(vol, "OK~", lba, off);
    assert(off >= 32);
    uint8_t buf[SECTOR];
    vol.read(lba, buf);
    assert(buf[off - 32 + 11] == 0x0F);
    buf[off - 32 + 1] = '#'; // LFN char 0: "ok.conf" -> "#k.conf"
    buf[off - 32 + 2] = 0;
    vol.write(lba, buf);
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

static void testNonTrackedIgnored() {
    Volume vol(128, confdrive::naming());
    assert(vol.build({{"ok.conf", "x"}}, 1));
    writeSfn(vol, "NOTES", "TXT", 2000, 8);
    setFat(vol, 2000, 0xFFF);
    writeSfn(vol, ".HIDDEN", "", 2001, 8);
    setFat(vol, 2001, 0xFFF);
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err));
    assert(host.size() == 1 && host[0].name == "ok.conf");
}

static void testNonTrackedBadChainBlocks() {
    Volume vol(128, confdrive::naming());
    assert(vol.build({{"ok.conf", "x"}}, 1));
    writeSfn(vol, "NOTES", "TXT", 0xFF0F, 8); // out of range: an ignored entry still has its chain checked
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("bad cluster chain") != std::string::npos);
}

static void testHiddenCrossLinkBlocks() {
    Volume vol(128, confdrive::naming());
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t c = rootEntryFor(vol, "BOARD~", lba, off);
    writeSfn(vol, ".HIDDEN", "", c, 8); // hidden entry shares board.conf's cluster
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("bad cluster chain") != std::string::npos);
}

static void testEditAndSizeMismatch() {
    Volume vol(128, confdrive::naming());
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
    Volume vol(128, confdrive::naming());
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

static void testLoopingChain() {
    Volume vol(128, confdrive::naming());
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t c = rootEntryFor(vol, "CHARG~", lba, off); // charger.conf spans 3 clusters
    setFat(vol, c, c); // self-loop
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("bad cluster chain") != std::string::npos);
}

static void testBuildSkipsDuplicate() {
    // a case-insensitive duplicate is skipped, so build() still collects
    Volume vol(128, confdrive::naming());
    assert(!vol.build({{"dup.conf", "a"}, {"DUP.conf", "b"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1 && host[0].name == "dup.conf" && host[0].data == "a");
}

static void testBuildNameLengthBoundary() {
    // FAT long-name maximum is 255 bytes (20 entries cover 255 bytes plus the terminator)
    Volume vol; // default Naming tracks every non-hidden entry
    std::string max = std::string(255, 'a');
    assert(vol.build({{max, "x"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1 && host[0].name == max);

    Volume vol2;
    assert(!vol2.build({{std::string(256, 'a'), "x"}}, 1));
    std::vector<File> h2;
    std::string e2;
    assert(vol2.collect(h2, e2) && h2.empty());
}

static void testBuildRejectsBadNameBytes() {
    // a byte collect() would truncate or change, or that FAT forbids, must be skipped, not built
    const std::string bad[] = {
        std::string("a\0b.conf", 8),     // embedded NUL
        std::string("caf\xC3\xA9.conf"), // bytes > 0x7E
        std::string("del\x7f.conf"),     // DEL
        "star*.conf", "q?.conf", "c:olon.conf", "pipe|.conf", "sl/ash.conf",
        "back\\slash.conf", "les<s.conf", "gre>ater.conf", "quo\"te.conf",
        "trail.conf ", "trail.conf.",
    };
    for (auto &n: bad) {
        Volume vol; // default Naming
        assert(!vol.build({{n, "x"}}, 1));
        std::vector<File> host;
        std::string err;
        assert(vol.collect(host, err) && host.empty());
    }
}

static void testBuildSanitizesSfn() {
    // an LFN-legal name with a space still gets a legal 8.3 alias
    Volume vol; // default Naming
    assert(vol.build({{"my file.txt", "x"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.size() == 1 && host[0].name == "my file.txt");
    uint8_t sn[11];
    firstSfn(vol, sn);
    assert(!memcmp(sn, "MY_FI~1", 7));
    assert(!memcmp(sn + 8, "TXT", 3));
    for (int i = 0; i < 11; ++i) {
        char c = (char) sn[i];
        assert(c == ' ' || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || strchr("$%'-_@~!(){}^#&", c));
    }
}

static void testPrintableAsciiNames() {
    // every printable ASCII byte either round-trips exactly or is rejected; none is silently changed
    for (int c = 0x20; c <= 0x7E; ++c) {
        std::string name = std::string("a") + (char) c + "b.txt";
        Volume vol; // default Naming
        bool built = vol.build({{name, "x"}}, 1);
        std::vector<File> host;
        std::string err;
        assert(vol.collect(host, err));
        if (built) assert(host.size() == 1 && host[0].name == name);
        else assert(host.empty());
    }
}

static void testBuildSkipsOversizedFile() {
    Volume vol(128, Naming{".conf", nullptr});
    assert(!vol.build({{"big.conf", std::string(Volume::MAX_FILE_BYTES + 1, 'x')}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err) && host.empty());
}

static void testCollectDuplicateBlocks() {
    // two root entries with the same name (as a host could write) must not read as two files
    Volume vol;
    assert(vol.build({{"ok.txt", "x"}}, 1));
    writeSfn(vol, "DUP", "TXT", 2000, 4);
    setFat(vol, 2000, 0xFFF);
    writeSfn(vol, "DUP", "TXT", 2001, 4);
    setFat(vol, 2001, 0xFFF);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
    assert(err.find("duplicate") != std::string::npos);
}

static void testDeleteAndCreate() {
    Volume vol(128, confdrive::naming());
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
    Volume vol(128, confdrive::naming());
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
        Volume vol(128, confdrive::naming());
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
    Volume vol(128, confdrive::naming());
    vol.build(sample(), 1);
    uint32_t lba, off;
    uint16_t c = rootEntryFor(vol, "BOARD~", lba, off);
    // append a junk 8.3 file "JUNK.TXT" sharing board.conf's cluster
    writeSfn(vol, "JUNK", "TXT", c, 8);
    std::vector<File> host;
    std::string err;
    assert(!vol.collect(host, err));
}

static void testOverlayCap() {
    Volume vol(2, confdrive::naming());
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
    Volume vol(128, confdrive::naming());
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

static void testDefaultNamingTracksAll() {
    Volume vol; // Naming{} default: every non-hidden entry is tracked
    assert(vol.build({{"notes.txt", "hello"}, {"readme", "x"}}, 1));
    std::vector<File> host;
    std::string err;
    assert(vol.collect(host, err));
    assert(host.size() == 2);
    assert(host[0].name == "notes.txt" && host[1].name == "readme");
}

int main(int argc, char **argv) {
    if (argc == 4 && !strcmp(argv[1], "gen")) return gen(argv[2], argv[3]);
    if (argc >= 4 && !strcmp(argv[1], "apply")) return apply(argv[2], argv[3], argc > 4 ? argv[4] : nullptr);
    testRoundTrip();
    testValidName();
    testInvalidNamesSkipped();
    testCollectInvalidNameBlocks();
    testCollectBadCharLfnBlocks();
    testSfnOnlyConfExtensionBlocks();
    testNonTrackedIgnored();
    testNonTrackedBadChainBlocks();
    testHiddenCrossLinkBlocks();
    testEditAndSizeMismatch();
    testCrossLinkAndBadCluster();
    testLoopingChain();
    testBuildSkipsDuplicate();
    testBuildNameLengthBoundary();
    testBuildRejectsBadNameBytes();
    testBuildSanitizesSfn();
    testPrintableAsciiNames();
    testBuildSkipsOversizedFile();
    testCollectDuplicateBlocks();
    testDeleteAndCreate();
    testBrokenLfnBlocks();
    testMalformedEntriesBlock();
    testJunkCrossLinkBlocks();
    testOverlayCap();
    testFat2Alias();
    testConfText();
    testDefaultNamingTracksAll();
    printf("vfat-test: all passed\n");
    return 0;
}
