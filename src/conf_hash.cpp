// Configuration fingerprint (`conf-hash`). Definition and rationale: conf_hash.h.
// Host reference implementation: etc/conf_hash.py. Strictly read-only.

#include "conf_hash.h"

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <new>
#include <sys/stat.h>

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

static const char *TAG = "confhash";

namespace {

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

    void updateStr(const std::string &s) { update(s.data(), s.size()); }
    void updateNul() { update("", 1); }
    void updateDec(uint64_t v) {
        char b[24];
        int n = snprintf(b, sizeof(b), "%" PRIu64, v);
        update(b, (size_t) n);
    }
};

constexpr int MAX_DEPTH = 8;

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
        if (stat(full.c_str(), &st) != 0) {
            err = "stat " + full + ": " + strerror(errno);
            ok = false;
            break;
        }
        if (S_ISDIR(st.st_mode)) {
            if (depth >= MAX_DEPTH) { err = "too deep: " + full; ok = false; break; }
            if (!collect(root, r, depth + 1, out, err)) { ok = false; break; }
        } else if (S_ISREG(st.st_mode)) {
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

ConfHashResult s_boot = [] {
    ConfHashResult r;
    r.err = "not computed";
    return r;
}();

} // namespace

std::string confHashHex(const uint8_t *b, size_t n) {
    static const char *hx = "0123456789abcdef";
    std::string s(n * 2, '0');
    for (size_t i = 0; i < n; ++i) {
        s[2 * i] = hx[b[i] >> 4];
        s[2 * i + 1] = hx[b[i] & 0xf];
    }
    return s;
}

bool confHashCompute(ConfHashResult &out, bool perFile) {
    out = ConfHashResult{};
    auto fail = [&](std::string m) { out.ok = false; out.err = std::move(m); out.perFile.clear(); return false; };

    if (!esp_littlefs_mounted("littlefs"))
        return fail("littlefs not mounted");
    const std::string root = CONF_HASH_ROOT;
    struct stat st;
    if (stat(root.c_str(), &st) != 0)
        return fail("missing " + root);
    if (!S_ISDIR(st.st_mode))
        return fail(root + " is not a directory");

    std::vector<std::string> files;
    std::string err;
    if (!collect(root, "", 0, files, err))
        return fail(err);
    // std::string's operator< compares via char_traits<char>, i.e. as unsigned char: bytewise.
    std::sort(files.begin(), files.end());

    constexpr size_t BUF = 1024;
    std::unique_ptr<uint8_t[]> buf(new(std::nothrow) uint8_t[BUF]);
    if (!buf) return fail("out of memory");

    Sha256 all;
    for (const auto &rel: files) {
        const std::string full = root + "/" + rel;
        FILE *f = fopen(full.c_str(), "rb");
        if (!f) return fail("open " + full + ": " + strerror(errno));
        struct stat fs;
        if (fstat(fileno(f), &fs) != 0) {
            int e = errno;
            fclose(f);
            return fail("fstat " + full + ": " + strerror(e));
        }
        const uint64_t size = (uint64_t) fs.st_size;
        all.updateStr(rel);
        all.updateNul();
        all.updateDec(size);
        all.updateNul();
        Sha256 one;
        uint64_t total = 0;
        for (;;) {
            size_t n = fread(buf.get(), 1, BUF, f);
            if (n) { all.update(buf.get(), n); one.update(buf.get(), n); total += n; }
            if (n < BUF) break;
        }
        bool rdErr = ferror(f) != 0;
        fclose(f);
        if (rdErr) return fail("read error " + full);
        // The length was hashed before the content; a file that changed size under us would make
        // the hash describe neither the old nor the new file.
        if (total != size) return fail("size changed while reading " + full);
        ConfHashFile cf;
        cf.relpath = rel;
        cf.bytes = (uint32_t) size;
        if (!one.finish(cf.sha)) return fail("sha256 failed");
        if (perFile) out.perFile.push_back(std::move(cf));
        out.files++;
        out.bytes += size;
    }
    if (!all.finish(out.sha)) return fail("sha256 failed");
    out.ok = true;
    return true;
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
    std::vector<Entry> entries;

    nvs_iterator_t it = nullptr;
    esp_err_t e = nvs_entry_find(NVS_DEFAULT_PART_NAME, ns, NVS_TYPE_ANY, &it);
    while (e == ESP_OK) {
        nvs_entry_info_t info;
        e = nvs_entry_info(it, &info);
        if (e != ESP_OK) break;
        entries.push_back({std::string(info.key, strnlen(info.key, sizeof(info.key))), info.type});
        e = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    if (e != ESP_ERR_NVS_NOT_FOUND) { // NOT_FOUND = iteration ended (or the namespace is empty)
        out.err = std::string("nvs iterate: ") + esp_err_to_name(e);
        return false;
    }
    std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b) { return a.key < b.key; });
    out.keys = (uint32_t) entries.size();

    nvs_handle_t h = 0;
    bool open = false;
    if (!entries.empty()) {
        e = nvs_open(ns, NVS_READONLY, &h);
        if (e != ESP_OK) {
            out.err = std::string("nvs_open: ") + esp_err_to_name(e);
            return false;
        }
        open = true;
    }
    Sha256 sha;
    std::string v;
    for (const auto &en: entries) {
        const char *tn = nvsTypeName(en.type);
        e = tn ? nvsReadValue(h, en.key.c_str(), en.type, v) : ESP_ERR_NOT_SUPPORTED;
        if (e != ESP_OK) {
            char b[96];
            snprintf(b, sizeof(b), "%s:0x%02x:%s", en.key.c_str(), (unsigned) en.type, esp_err_to_name(e));
            out.unreadableKeys.emplace_back(b);
            out.unreadable++;
            continue;
        }
        sha.updateStr(en.key);
        sha.updateNul();
        sha.update(tn, strlen(tn));
        sha.updateNul();
        sha.updateDec(v.size());
        sha.updateNul();
        sha.updateStr(v);
    }
    if (open) nvs_close(h);
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
