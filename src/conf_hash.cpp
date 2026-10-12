// Configuration fingerprint (`conf-hash`). Definition: conf_hash.h; pure parts: conf_hash_core.h;
// host reference: etc/conf_hash.py. Strictly read-only. Builds on the host (without ESP_PLATFORM)
// for test/host-stub/conf-hash-test.cpp, which supplies a software Sha256 (sha256_host.h).

#include "conf_hash.h"

#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <new>
#include <sys/stat.h>

#ifdef ESP_PLATFORM
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_littlefs.h>
#include <nvs.h>
#include <nvs_flash.h>
// IDF 5.x ships mbedTLS 3 with the public mbedtls/sha256.h. IDF 6 (mbedTLS 4 / TF-PSA-Crypto) made
// that header private; the PSA hash API is the public interface there.
#if ESP_IDF_VERSION_MAJOR < 6
#include <mbedtls/sha256.h>
#else
#include <psa/crypto.h>
#endif
#else
#include "sha256_host.h" // test/host-stub
#endif

namespace {

#ifdef ESP_PLATFORM
const char *TAG = "confhash";

class Sha256 {
#if ESP_IDF_VERSION_MAJOR < 6
    mbedtls_sha256_context ctx;
    bool ok;
public:
    Sha256() {
        mbedtls_sha256_init(&ctx);
        ok = mbedtls_sha256_starts(&ctx, 0) == 0;
    }
    ~Sha256() { mbedtls_sha256_free(&ctx); }
    void update(const void *p, size_t n) {
        if (ok && n) ok = mbedtls_sha256_update(&ctx, (const unsigned char *) p, n) == 0;
    }
    bool finish(uint8_t out[32]) {
        if (ok) ok = mbedtls_sha256_finish(&ctx, out) == 0;
        return ok;
    }
#else
    psa_hash_operation_t op = PSA_HASH_OPERATION_INIT;
    bool ok;
public:
    Sha256() {
        ok = psa_crypto_init() == PSA_SUCCESS && psa_hash_setup(&op, PSA_ALG_SHA_256) == PSA_SUCCESS;
    }
    ~Sha256() { psa_hash_abort(&op); }
    void update(const void *p, size_t n) {
        if (ok && n) ok = psa_hash_update(&op, (const uint8_t *) p, n) == PSA_SUCCESS;
    }
    bool finish(uint8_t out[32]) {
        size_t len = 0;
        if (ok) ok = psa_hash_finish(&op, out, 32, &len) == PSA_SUCCESS && len == 32;
        return ok;
    }
#endif
    Sha256(const Sha256 &) = delete;
    Sha256 &operator=(const Sha256 &) = delete;
};
#endif

// littlefs has no symlinks, and the IDF VFS has no lstat(); on a host copy, lstat() makes a symlink
// an error instead of following it (the Python reference does the same).
int statNoFollow(const char *p, struct stat *st) {
#ifdef ESP_PLATFORM
    return stat(p, st);
#else
    return lstat(p, st);
#endif
}

// Collect regular files below root/rel, recursively, as root-relative '/' paths.
bool collect(const std::string &root, const std::string &rel, int depth,
             std::vector<std::string> &out, std::string &err) {
    const std::string dir = rel.empty() ? root : root + "/" + rel;
    DIR *d = opendir(dir.c_str());
    if (!d) {
        err = "cannot open dir " + dir + ": " + strerror(errno);
        return false;
    }
    bool ok = true;
    for (;;) {
        errno = 0;
        dirent *e = readdir(d);
        if (!e) {
            if (errno) { err = "readdir " + dir + ": " + strerror(errno); ok = false; }
            break;
        }
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        const std::string r = rel.empty() ? std::string(e->d_name) : rel + "/" + e->d_name;
        const std::string full = root + "/" + r;
        struct stat st;
        if (statNoFollow(full.c_str(), &st) != 0) {
            err = "stat " + full + ": " + strerror(errno);
            ok = false;
            break;
        }
        if (S_ISDIR(st.st_mode)) {
            if (depth >= CONF_HASH_MAX_DEPTH) { err = "too deep: " + full; ok = false; break; }
            if (!collect(root, r, depth + 1, out, err)) { ok = false; break; }
        } else if (S_ISREG(st.st_mode)) {
            if (confHashIsStaging(r)) { err = "staging file present: " + r; ok = false; break; }
            if (r != CONF_HASH_EXCLUDED) out.push_back(r);
        } else {
            err = "not a regular file: " + full;
            ok = false;
            break;
        }
    }
    closedir(d);
    return ok;
}

constexpr size_t READ_BUF = 512;

struct FileStats {
    uint64_t rawBytes = 0;
    uint64_t outBytes = 0;
    uint32_t redacted = 0;
};

// Stream a file's redacted content into a and b (either may be null).
bool redactFile(const std::string &full, uint8_t *buf, Sha256 *a, Sha256 *b, FileStats &st,
                std::string &err) {
    st = FileStats{};
    FILE *f = fopen(full.c_str(), "rb");
    if (!f) { err = "open " + full + ": " + strerror(errno); return false; }
    struct stat fs;
    if (fstat(fileno(f), &fs) != 0) {
        err = "fstat " + full + ": " + strerror(errno);
        fclose(f);
        return false;
    }
    ConfHashRedactor red;
    auto emit = [&](const std::string &s) {
        if (a) a->update(s.data(), s.size());
        if (b) b->update(s.data(), s.size());
        st.outBytes += s.size();
    };
    for (;;) {
        size_t n = fread(buf, 1, READ_BUF, f);
        st.rawBytes += n;
        red.feed(buf, n, emit);
        if (n < READ_BUF || red.tooLong) break;
    }
    const bool rdErr = ferror(f) != 0;
    fclose(f);
    if (rdErr) { err = "read error " + full; return false; }
    if (red.tooLong) { err = "line too long in " + full; return false; }
    red.finish(emit);
    st.redacted = red.redacted;
    if (st.rawBytes != (uint64_t) fs.st_size) { err = "size changed while reading " + full; return false; }
    return true;
}

#ifdef ESP_PLATFORM
ConfHashResult s_boot = [] {
    ConfHashResult r;
    r.err = "not computed";
    return r;
}();
#endif

} // namespace

bool confHashComputeAt(ConfHashResult &out, bool perFile, const std::string &root) {
    out = ConfHashResult{};
    auto fail = [&](std::string m) { out.ok = false; out.err = std::move(m); out.perFile.clear(); return false; };

    struct stat st;
    if (stat(root.c_str(), &st) != 0)
        return fail("missing " + root);
    if (!S_ISDIR(st.st_mode))
        return fail(root + " is not a directory");

    std::vector<std::string> files;
    std::string err;
    if (!collect(root, "", 0, files, err))
        return fail(err);
    confHashSortPaths(files);

    std::unique_ptr<uint8_t[]> buf(new(std::nothrow) uint8_t[READ_BUF]);
    if (!buf) return fail("out of memory");

    Sha256 all;
    for (const auto &rel: files) {
        const std::string full = root + "/" + rel;
        // Two passes: the redacted length is hashed before the redacted content, and the content is
        // streamed line by line rather than buffered. Both passes must see the same file.
        FileStats p1, p2;
        if (!redactFile(full, buf.get(), nullptr, nullptr, p1, err)) return fail(err);
        confHashFrame(all, rel, p1.outBytes);
        Sha256 one;
        if (!redactFile(full, buf.get(), &all, &one, p2, err)) return fail(err);
        if (p1.rawBytes != p2.rawBytes || p1.outBytes != p2.outBytes || p1.redacted != p2.redacted)
            return fail("file changed while reading " + full);
        ConfHashFile cf;
        cf.relpath = rel;
        cf.bytes = (uint32_t) p2.outBytes;
        cf.redacted = p2.redacted;
        if (!one.finish(cf.sha)) return fail("sha256 failed");
        if (perFile) out.perFile.push_back(std::move(cf));
        out.files++;
        out.bytes += p2.outBytes;
        out.redacted += p2.redacted;
    }
    if (!all.finish(out.sha)) return fail("sha256 failed");
    out.ok = true;
    return true;
}

#ifdef ESP_PLATFORM

bool confHashCompute(ConfHashResult &out, bool perFile) {
    if (!esp_littlefs_mounted("littlefs")) {
        out = ConfHashResult{};
        out.err = "littlefs not mounted";
        return false;
    }
    return confHashComputeAt(out, perFile, CONF_HASH_ROOT);
}

static const char *nvsTypeName(nvs_type_t t) {
    switch (t) {
        case NVS_TYPE_U8: return "u8";
        case NVS_TYPE_I8: return "i8";
        case NVS_TYPE_U16: return "u16";
        case NVS_TYPE_I16: return "i16";
        case NVS_TYPE_U32: return "u32";
        case NVS_TYPE_I32: return "i32";
        case NVS_TYPE_U64: return "u64";
        case NVS_TYPE_I64: return "i64";
        case NVS_TYPE_STR: return "str";
        case NVS_TYPE_BLOB: return "blob";
        default: return nullptr;
    }
}

// Read one entry's value bytes (little-endian ints at native width, string without NUL, raw blob).
static esp_err_t nvsReadValue(nvs_handle_t h, const char *key, nvs_type_t t, std::string &v) {
    constexpr size_t MAX_VAR = 4096; // the app stores short strings; refuse to buffer anything huge
    v.clear();
    esp_err_t e;
    switch (t) {
#define NVS_INT(T, CT, FN) case T: { CT x{}; e = FN(h, key, &x); if (e == ESP_OK) v.assign((const char *) &x, sizeof(x)); return e; }
        NVS_INT(NVS_TYPE_U8, uint8_t, nvs_get_u8)
        NVS_INT(NVS_TYPE_I8, int8_t, nvs_get_i8)
        NVS_INT(NVS_TYPE_U16, uint16_t, nvs_get_u16)
        NVS_INT(NVS_TYPE_I16, int16_t, nvs_get_i16)
        NVS_INT(NVS_TYPE_U32, uint32_t, nvs_get_u32)
        NVS_INT(NVS_TYPE_I32, int32_t, nvs_get_i32)
        NVS_INT(NVS_TYPE_U64, uint64_t, nvs_get_u64)
        NVS_INT(NVS_TYPE_I64, int64_t, nvs_get_i64)
#undef NVS_INT
        case NVS_TYPE_STR: {
            size_t len = 0;
            if ((e = nvs_get_str(h, key, nullptr, &len)) != ESP_OK) return e;
            if (len == 0 || len > MAX_VAR) return ESP_ERR_INVALID_SIZE;
            std::string s(len, '\0');
            if ((e = nvs_get_str(h, key, s.data(), &len)) != ESP_OK) return e;
            v.assign(s.data(), strnlen(s.data(), len));
            return ESP_OK;
        }
        case NVS_TYPE_BLOB: {
            size_t len = 0;
            if ((e = nvs_get_blob(h, key, nullptr, &len)) != ESP_OK) return e;
            if (len > MAX_VAR) return ESP_ERR_INVALID_SIZE;
            v.assign(len, '\0');
            if (len && (e = nvs_get_blob(h, key, v.data(), &len)) != ESP_OK) return e;
            v.resize(len);
            return ESP_OK;
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

bool nvsHashCompute(NvsHashResult &out, const char *ns) {
    out = NvsHashResult{};
    struct Entry { std::string key; nvs_type_t type; };
    std::vector<Entry> found;

    nvs_iterator_t it = nullptr;
    esp_err_t e = nvs_entry_find(NVS_DEFAULT_PART_NAME, ns, NVS_TYPE_ANY, &it);
    while (e == ESP_OK) {
        nvs_entry_info_t info;
        e = nvs_entry_info(it, &info);
        if (e != ESP_OK) break;
        found.push_back({std::string(info.key, strnlen(info.key, sizeof(info.key))), info.type});
        e = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    if (e != ESP_ERR_NVS_NOT_FOUND) { // NOT_FOUND = iteration ended (or the namespace is empty)
        out.err = std::string("nvs iterate: ") + esp_err_to_name(e);
        return false;
    }
    out.keys = (uint32_t) found.size();

    nvs_handle_t h = 0;
    bool open = false;
    if (!found.empty()) {
        e = nvs_open(ns, NVS_READONLY, &h);
        if (e != ESP_OK) {
            out.err = std::string("nvs_open: ") + esp_err_to_name(e);
            return false;
        }
        open = true;
    }
    std::vector<ConfHashNvsEntry> entries;
    for (const auto &en: found) {
        ConfHashNvsEntry ce;
        ce.key = en.key;
        if (!confHashIsSecretKey(en.key)) { // a secret is never read
            const char *tn = nvsTypeName(en.type);
            e = tn ? nvsReadValue(h, en.key.c_str(), en.type, ce.value) : ESP_ERR_NOT_SUPPORTED;
            if (e != ESP_OK) {
                char b[96];
                snprintf(b, sizeof(b), "%s:0x%02x:%s", en.key.c_str(), (unsigned) en.type, esp_err_to_name(e));
                out.unreadableKeys.emplace_back(b);
                out.unreadable++;
                continue;
            }
            ce.type = tn;
        }
        entries.push_back(std::move(ce));
    }
    if (open) nvs_close(h);
    Sha256 sha;
    out.redacted = confHashNvsEntries(sha, std::move(entries));
    if (!sha.finish(out.sha)) { out.err = "sha256 failed"; return false; }
    if (out.unreadable) { out.err = "unreadable entries"; return false; }
    out.ok = true;
    return true;
}

void confHashCaptureBoot() {
    confHashCompute(s_boot, false);
    if (s_boot.ok)
        ESP_LOGI(TAG, "conf sha256=%s files=%" PRIu32 " (boot)", confHashHex(s_boot.sha, 32).c_str(), s_boot.files);
    else
        ESP_LOGW(TAG, "boot conf hash unavailable: %s", s_boot.err.c_str());
}

const ConfHashResult &confHashBoot() { return s_boot; }

#endif // ESP_PLATFORM
