// Host test for ConfFile writes (src/conf.h): staged <path>.cft + rename, with faults injected at each
// file-API boundary. Error mode: the call fails (EIO). Reset mode: execution stops there (throws), as
// a reset would; the boot cleanup then runs. Either way the old file and its other keys must survive.
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src -o /tmp/conf-write-test \
//       test/host-stub/conf-write-test.cpp && /tmp/conf-write-test

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>
#include <esp_log.h>
#include "console.h"

enum Fault { None, ReadOpen, ReadErr, TmpOpen, Write, Flush, Sync, Close, Rename, NFaults };
static const char *const kNames[] = {"none", "read-open", "read-error", "tmp-open", "write", "flush", "fsync",
                                     "close", "rename"};
static Fault g_fault = None;
static bool g_reset = false;
static FILE *g_wf = nullptr;
struct Reset {};

static bool hit(Fault f) {
    if (g_fault != f) return false;
    if (g_reset) throw Reset{};
    errno = EIO;
    return true;
}
static FILE *f_fopen(const char *p, const char *m) {
    if (m[0] == 'r' && hit(ReadOpen)) return nullptr;
    if (m[0] == 'w' && hit(TmpOpen)) return nullptr;
    FILE *f = fopen(p, m);
    if (m[0] == 'w') g_wf = f;
    return f;
}
static int f_ferror(FILE *f) { return (f != g_wf && g_fault == ReadErr && !g_reset) ? 1 : ferror(f); }
static int f_fputs(const char *s, FILE *f) { return hit(Write) ? EOF : fputs(s, f); }
static int f_fflush(FILE *f) { return hit(Flush) ? EOF : fflush(f); }
static int f_fsync(int fd) { return hit(Sync) ? -1 : fsync(fd); }
static int f_fclose(FILE *f) {
    if (f == g_wf && g_fault == Close) {
        g_wf = nullptr;
        if (g_reset) throw Reset{}; // leaves the staged file open/partial, like a reset
        fclose(f);
        errno = EIO;
        return EOF;
    }
    if (f == g_wf) g_wf = nullptr;
    return fclose(f);
}
static int f_rename(const char *a, const char *b) { return hit(Rename) ? -1 : rename(a, b); }

#undef ferror
#define fopen f_fopen
#define ferror f_ferror
#define fputs f_fputs
#define fflush f_fflush
#define fsync f_fsync
#define fclose f_fclose
#define rename f_rename
#include "conf.h"
#undef fopen
#undef ferror
#undef fputs
#undef fflush
#undef fsync
#undef fclose
#undef rename

static int g_fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++g_fails; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const char *kOrig = "# coil\nL0=50e-6\nrect_cal_auto=0\nrect_offset=12\nrect_offset_ns=-200  # old\n";

static std::string slurp(const std::string &p) {
    std::string s;
    if (FILE *f = fopen(p.c_str(), "r")) {
        char b[256];
        size_t n;
        while ((n = fread(b, 1, sizeof(b), f)) > 0) s.append(b, n);
        fclose(f);
    }
    return s;
}
static void put(const std::string &p, const char *s) {
    FILE *f = fopen(p.c_str(), "w");
    fputs(s, f);
    fclose(f);
}
static bool exists(const std::string &p) { return access(p.c_str(), F_OK) == 0; }

int main() {
    char tmpl[] = "/tmp/conf-write-test-XXXXXX";
    const std::string dir = mkdtemp(tmpl);
    const std::string path = dir + "/coil.conf", staged = path + ".cft", usbStage = path + ".new";

    for (int r = 0; r < 2; ++r)
        for (int fi = None; fi < NFaults; ++fi) {
            const auto fault = (Fault) fi;
            if (r && fault == ReadErr) continue; // a read error has no reset point of its own
            put(path, kOrig);
            ConfFile c{path.c_str()};
            g_fault = fault;
            g_reset = r;
            bool ok = false, reset = false;
            try {
                ok = c.update({{"rect_offset_ns", std::string("-113").c_str()}, {"rect_offset_drv", "vconv"},
                               {"rect_offset_cal", "1"}}, "rect_offset");
            } catch (Reset &) { reset = true; }
            g_fault = None;
            const char *what = kNames[fi];
            if (fault == None) {
                ConfFile n{path.c_str()};
                CHECK(ok && n.getString("rect_offset_ns", "") == "-113" && n.getString("rect_offset", "x") == "x"
                      && n.getByte("rect_cal_auto", 1) == 0 && n.getString("L0", "") == "50e-6" && !exists(staged)
                      && c.getString("rect_offset_ns", "") == "-113", "%s: write", what);
                continue;
            }
            if (r) {
                CHECK(reset, "%s: no reset point", what);
                confDropStaging(dir.c_str());
            } else {
                CHECK(!ok && c.getString("rect_offset_ns", "") == "-200" && c.getString("rect_offset", "") == "12",
                      "%s: returned %d, in-memory view changed", what, ok);
            }
            ConfFile n{path.c_str()};
            CHECK(slurp(path) == kOrig && !exists(staged) && n.getByte("rect_cal_auto", 1) == 0
                  && n.getString("L0", "") == "50e-6", "%s %s: original not intact (staged left %d)", what,
                  r ? "reset" : "error", exists(staged));
        }

    g_reset = false;

    // a ConfFile write leaves the USB drive's stage alone; the boot cleanup drops it
    put(path, kOrig);
    put(usbStage, "usb=1\n");
    {
        ConfFile c{path.c_str()};
        g_fault = Rename;
        CHECK(!c.update({{"a", "1"}}) && slurp(usbStage) == "usb=1\n", "failed write touched the USB stage");
        g_fault = None;
        CHECK(c.update({{"a", "1"}}) && slurp(usbStage) == "usb=1\n", "write touched the USB stage");
    }
    CHECK(confDropStaging(dir.c_str()) && !exists(usbStage), "boot cleanup kept the USB stage");

    // a CR-only last line without LF, CRLF and a last line without LF
    for (const char *in: {"a=1\n\r", "a=1\r\n\r\r", "a=1\r\nb=2"}) {
        put(path, in);
        ConfFile c{path.c_str()};
        CHECK(c.update({{"x", "1"}}) && ConfFile{path.c_str()}.getByte("a", 0) == 1
              && ConfFile{path.c_str()}.getByte("x", 0) == 1, "line ends %zu", strlen(in));
    }
    CHECK(slurp(path) == "a=1\nb=2\nx=1\n", "CRLF rewrite: %s", slurp(path).c_str());
    // an embedded NUL: rejected, file unchanged
    for (size_t at: {0, 3, 7}) {
        std::string bin = "a=1\nb=2\n";
        bin.insert(at, 1, '\0');
        FILE *f = fopen(path.c_str(), "wb");
        fwrite(bin.data(), 1, bin.size(), f);
        fclose(f);
        ConfFile c{path.c_str()};
        CHECK(!c.update({{"x", "1"}}) && slurp(path) == bin && !exists(staged), "NUL at %zu accepted", at);
    }

    // missing file: created
    unlink(path.c_str());
    {
        ConfFile c{path.c_str(), true};
        CHECK(c.update({{"a", "1"}}) && ConfFile{path.c_str()}.getByte("a", 0) == 1, "create");
    }
    // a stale staged file is never read as config and goes at boot
    put(path, kOrig);
    put(staged, "rect_cal_auto=1\nL0=1\n");
    CHECK(ConfFile{path.c_str()}.getByte("rect_cal_auto", 1) == 0, "stale staged file read");
    confDropStaging(dir.c_str());
    CHECK(!exists(staged) && slurp(path) == kOrig, "stale staged file not removed");
    // remove(): a failed rename keeps the key on disk and in memory; absent key: false, no write
    {
        ConfFile c{path.c_str()};
        g_fault = Rename;
        CHECK(!c.remove("rect_offset") && c.getString("rect_offset", "") == "12" && slurp(path) == kOrig,
              "remove with failed rename");
        g_fault = None;
        CHECK(c.remove("rect_offset") && c.getString("rect_offset", "x") == "x"
              && ConfFile{path.c_str()}.getString("rect_offset", "x") == "x", "remove");
        CHECK(!c.remove("nope"), "remove of an absent key");
    }
    // a line longer than the read buffer stays one line
    {
        std::string longv(600, 'v');
        put(path, ("k=" + longv + "\nrect_cal_auto=0\n").c_str());
        ConfFile c{path.c_str()};
        CHECK(c.update({{"x", "1"}}) && slurp(path) == "k=" + longv + "\nrect_cal_auto=0\nx=1\n", "long line");
    }
    unlink(path.c_str());
    rmdir(dir.c_str());
    printf("conf-write-test: %s (fails=%d)\n", g_fails ? "FAIL" : "PASS", g_fails);
    return g_fails != 0;
}
