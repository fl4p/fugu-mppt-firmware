#include "vfat.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace vfat {

namespace {

constexpr uint16_t FAT_DATE = ((2026 - 1980) << 9) | (1 << 5) | 1;
constexpr uint8_t ATTR_VOL = 0x08, ATTR_DIR = 0x10, ATTR_ARCH = 0x20, ATTR_LFN = 0x0F;

void put16(uint8_t *p, uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}

void put32(uint8_t *p, uint32_t v) {
    put16(p, v);
    put16(p + 2, v >> 16);
}

uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }

uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t) get16(p + 2) << 16); }

uint8_t lfnChecksum(const uint8_t *sn) {
    uint8_t s = 0;
    for (int i = 0; i < 11; ++i) s = ((s & 1) << 7) + (s >> 1) + sn[i];
    return s;
}

// LFN entry character offsets
constexpr uint8_t LFN_POS[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};

char lower(char c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; }

bool iequals(const std::string &a, const std::string &b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) if (lower(a[i]) != lower(b[i])) return false;
    return true;
}

uint32_t clustersFor(size_t bytes) { return (bytes + SECTOR - 1) / SECTOR; }

}

bool validName(const std::string &n) {
    if (n.size() < 6 || n.size() > 64 || !iequals(n.substr(n.size() - 5), ".conf")) return false;
    for (size_t i = 0; i < n.size() - 5; ++i) {
        char c = n[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-'))
            return false;
    }
    return true;
}

bool validConfText(const std::string &t, std::string &err) {
    if (t.size() > 16384) {
        err = "too large";
        return false;
    }
    unsigned ln = 0;
    for (size_t i = 0; i < t.size();) {
        size_t e = t.find('\n', i);
        if (e == std::string::npos) e = t.size();
        ++ln;
        std::string line = t.substr(i, e - i);
        i = e + 1;
        if (line.size() > 254) { // ConfFile parses 255-byte fgets() chunks
            err = "line " + std::to_string(ln) + ": too long";
            return false;
        }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        for (char c: line)
            if (c != '\t' && (c < 0x20 || c > 0x7E)) {
                err = "line " + std::to_string(ln) + ": non-ASCII or control char";
                return false;
            }
        line = line.substr(0, line.find('#'));
        size_t b = line.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        size_t eq = line.find('=');
        size_t ke = eq == std::string::npos ? 0 : line.find_last_not_of(" \t", eq ? eq - 1 : 0);
        if (eq == std::string::npos || eq <= b || ke == std::string::npos || ke < b) {
            err = "line " + std::to_string(ln) + ": expected key=value";
            return false;
        }
        for (size_t k = b; k <= ke; ++k) {
            char c = line[k];
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-')) {
                err = "line " + std::to_string(ln) + ": bad key";
                return false;
            }
        }
    }
    return true;
}

void Volume::clear() {
    _img.clear();
    _imgCluster.clear();
    _fat.clear();
    _root.clear();
    _overlay.clear();
}

uint16_t Volume::fatGet(uint32_t c) const {
    uint32_t o = c + c / 2;
    uint16_t v = _fat[o] | (_fat[o + 1] << 8);
    return (c & 1) ? v >> 4 : v & 0xFFF;
}

void Volume::fatSet(uint32_t c, uint16_t v) {
    uint32_t o = c + c / 2;
    if (c & 1) {
        _fat[o] = (_fat[o] & 0x0F) | (v << 4);
        _fat[o + 1] = v >> 4;
    } else {
        _fat[o] = v;
        _fat[o + 1] = (_fat[o + 1] & 0xF0) | ((v >> 8) & 0x0F);
    }
}

bool Volume::build(std::vector<File> files, uint32_t volumeId) {
    clear();
    _volId = volumeId;
    _fat.assign(FAT_SECTORS * SECTOR, 0);
    _root.assign(ROOT_SECTORS * SECTOR, 0);
    fatSet(0, 0xFF8);
    fatSet(1, 0xFFF);

    uint8_t *e = _root.data(), *end = e + _root.size();
    memcpy(e, "FUGU_CONF  ", 11);
    e[11] = ATTR_VOL;
    put16(e + 24, FAT_DATE);
    e += 32;

    bool all = true;
    uint32_t next = 2;
    for (auto &f: files) {
        uint32_t nc = clustersFor(f.data.size());
        size_t nlfn = (f.name.size() + 12) / 13;
        if (!validName(f.name) || next + nc > CLUSTERS + 2 || e + 32 * (nlfn + 1) > end) {
            all = false;
            continue;
        }

        // short name BASE~N.CON, unique by index
        uint8_t sn[11];
        memset(sn, ' ', 11);
        size_t dot = f.name.size() - 5, k = 0;
        for (size_t i = 0; i < dot && k < 5; ++i) {
            char c = f.name[i];
            sn[k++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
        }
        unsigned idx = _img.size() + 1;
        char tail[6];
        int tl = snprintf(tail, sizeof tail, "~%u", idx);
        memcpy(sn + std::min<size_t>(k, 8 - tl), tail, tl);
        memcpy(sn + 8, "CON", 3);
        uint8_t cs = lfnChecksum(sn);

        for (size_t n = nlfn; n >= 1; --n, e += 32) {
            e[0] = n | (n == nlfn ? 0x40 : 0);
            e[11] = ATTR_LFN;
            e[13] = cs;
            for (int j = 0; j < 13; ++j) {
                size_t ci = (n - 1) * 13 + j;
                uint16_t ch = ci < f.name.size() ? (uint8_t) f.name[ci] : (ci == f.name.size() ? 0 : 0xFFFF);
                put16(e + LFN_POS[j], ch);
            }
        }
        memcpy(e, sn, 11);
        e[11] = ATTR_ARCH;
        put16(e + 16, FAT_DATE);
        put16(e + 18, FAT_DATE);
        put16(e + 24, FAT_DATE);
        put16(e + 26, nc ? next : 0);
        put32(e + 28, f.data.size());
        e += 32;

        for (uint32_t i = 0; i < nc; ++i) fatSet(next + i, i + 1 < nc ? next + i + 1 : 0xFFF);
        _imgCluster.push_back(nc ? next : 0);
        next += nc;
        _img.push_back(std::move(f));
    }
    _overlay.reserve(_maxOverlay);
    return all;
}

uint8_t *Volume::overlayFind(uint32_t lba) const {
    for (auto &s: _overlay) if (s->lba == lba) return s->d;
    return nullptr;
}

void Volume::readData(uint32_t lba, uint8_t *buf) const {
    memset(buf, 0, SECTOR);
    uint32_t c = lba - DATA + 2;
    for (size_t i = 0; i < _img.size(); ++i) {
        uint32_t c0 = _imgCluster[i], nc = clustersFor(_img[i].data.size());
        if (nc && c >= c0 && c < c0 + nc) {
            size_t off = (c - c0) * SECTOR;
            memcpy(buf, _img[i].data.data() + off, std::min<size_t>(SECTOR, _img[i].data.size() - off));
            return;
        }
    }
}

bool Volume::read(uint32_t lba, uint8_t *buf) const {
    if (lba >= TOTAL_SECTORS || _fat.empty()) return false;
    if (lba == 0) {
        memset(buf, 0, SECTOR);
        memcpy(buf, "\xEB\x3C\x90MSDOS5.0", 11);
        put16(buf + 11, SECTOR);
        buf[13] = 1; // sectors per cluster
        put16(buf + 14, 1); // reserved
        buf[16] = 2; // FATs
        put16(buf + 17, ROOT_ENTRIES);
        put16(buf + 19, TOTAL_SECTORS);
        buf[21] = 0xF8;
        put16(buf + 22, FAT_SECTORS);
        put16(buf + 24, 63);
        put16(buf + 26, 255);
        buf[36] = 0x80;
        buf[38] = 0x29;
        put32(buf + 39, _volId);
        memcpy(buf + 43, "FUGU_CONF  FAT12   ", 19);
        buf[510] = 0x55;
        buf[511] = 0xAA;
    } else if (lba < ROOT) {
        memcpy(buf, &_fat[((lba - FAT1) % FAT_SECTORS) * SECTOR], SECTOR);
    } else if (lba < DATA) {
        memcpy(buf, &_root[(lba - ROOT) * SECTOR], SECTOR);
    } else if (auto *o = overlayFind(lba)) {
        memcpy(buf, o, SECTOR);
    } else {
        readData(lba, buf);
    }
    return true;
}

bool Volume::write(uint32_t lba, const uint8_t *buf) {
    if (lba >= TOTAL_SECTORS || _fat.empty()) return false;
    if (lba == 0) return true; // boot sector is fixed
    if (lba < FAT2) memcpy(&_fat[(lba - FAT1) * SECTOR], buf, SECTOR);
    else if (lba < ROOT) return true; // FAT copy 2 aliases FAT 1
    else if (lba < DATA) memcpy(&_root[(lba - ROOT) * SECTOR], buf, SECTOR);
    else if (auto *o = overlayFind(lba)) memcpy(o, buf, SECTOR);
    else {
        uint8_t cur[SECTOR];
        readData(lba, cur);
        if (!memcmp(cur, buf, SECTOR)) return true;
        if (_overlay.size() >= _maxOverlay) return false;
        auto s = std::unique_ptr<Sec>(new(std::nothrow) Sec);
        if (!s) return false;
        s->lba = lba;
        memcpy(s->d, buf, SECTOR);
        _overlay.push_back(std::move(s));
    }
    return true;
}

bool Volume::collect(std::vector<File> &out, std::string &err) const {
    out.clear();
    auto fail = [&](const std::string &what) {
        err = what;
        return false;
    };
    if (_fat.empty()) return fail("not built");
    std::string lfn;
    int lfnNext = -1; // next expected LFN ordinal, 0 = complete, -1 = none
    uint8_t lfnCs = 0;
    uint8_t buf[SECTOR];
    std::vector<bool> used(CLUSTERS + 2);

    for (const uint8_t *e = _root.data(); e < _root.data() + _root.size(); e += 32) {
        if (e[0] == 0x00 || e[0] == 0xE5) {
            if (lfnNext >= 0) return fail("orphan long name");
            if (e[0] == 0x00) break;
            continue;
        }
        if ((e[11] & 0x3F) == ATTR_LFN) {
            int seq = e[0] & 0x1F;
            if ((e[0] & 0xA0) || seq == 0 || seq > 20 || e[12] || get16(e + 26)) return fail("bad long name entry");
            if (e[0] & 0x40) {
                if (lfnNext >= 0) return fail("orphan long name");
                lfn.assign(seq * 13, '\0');
                lfnCs = e[13];
            } else if (seq != lfnNext || e[13] != lfnCs) {
                return fail("broken long name");
            }
            for (int j = 0; j < 13; ++j) {
                uint16_t ch = get16(e + LFN_POS[j]);
                lfn[(seq - 1) * 13 + j] = ch == 0xFFFF ? '\xFF' : (ch > 0x7E ? '?' : (char) ch);
            }
            lfnNext = seq - 1;
            continue;
        }
        if (lfnNext > 0) return fail("broken long name");
        bool haveLfn = lfnNext == 0;
        lfnNext = -1;
        std::string name;
        if (haveLfn) {
            if (lfnCs != lfnChecksum(e)) return fail("long name checksum");
            size_t end = std::min(lfn.find('\0'), lfn.size());
            for (size_t k = 0; k < lfn.size(); ++k)
                if ((k < end && (lfn[k] == '\xFF' || !lfn[k])) || (k > end && lfn[k] != '\xFF'))
                    return fail("bad long name padding");
            name = lfn.substr(0, end);
            if (name.empty()) return fail("empty long name");
        } else {
            // 8.3 with NT case bits (0x08 base, 0x10 ext lowercase)
            for (int i = 0; i < 8 && e[i] != ' '; ++i) {
                char c = i == 0 && e[0] == 0x05 ? (char) 0xE5 : e[i];
                name += (e[12] & 0x08) ? lower(c) : c;
            }
            if (e[8] != ' ') name += '.';
            for (int i = 8; i < 11 && e[i] != ' '; ++i) name += (e[12] & 0x10) ? lower(e[i]) : e[i];
        }
        if (e[11] & ATTR_VOL) continue;

        // every entry claims its clusters, so a conf file cross-linked with junk is caught too
        bool dir = e[11] & ATTR_DIR;
        bool conf = name[0] != '.' && ((name.size() > 5 && iequals(name.substr(name.size() - 5), ".conf")) ||
                                       (!haveLfn && !memcmp(e + 8, "CON", 3)));
        if (conf && (dir || !validName(name))) return fail(name + ": invalid conf entry");
        uint32_t size = get32(e + 28), c = get16(e + 26), need = dir ? CLUSTERS : clustersFor(size);
        if (conf && size > 64 * 1024) return fail(name + ": too large");
        File f{name, {}};
        for (uint32_t n = 0; n < need && !(dir && n && c >= 0xFF8); ++n) {
            if (c < 2 || c >= CLUSTERS + 2 || used[c]) return fail(name + ": bad cluster chain");
            used[c] = true;
            if (conf) {
                read(DATA + c - 2, buf);
                f.data.append((const char *) buf, std::min<uint32_t>(SECTOR, size - n * SECTOR));
            }
            c = fatGet(c);
        }
        if (dir ? c < 0xFF8 : (need ? c < 0xFF8 : c != 0)) return fail(name + ": chain length != size");
        if (!conf) continue;
        for (auto &o: out) if (iequals(o.name, name)) return fail(name + ": duplicate");
        out.push_back(std::move(f));
    }
    if (lfnNext >= 0) return fail("orphan long name");
    return true;
}

Diff diff(const std::vector<File> &base, const std::vector<File> &host) {
    Diff d;
    for (auto &h: host) {
        const File *b = nullptr;
        for (auto &x: base) if (iequals(x.name, h.name)) b = &x;
        if (!b) d.upsert.push_back(h);
        else if (b->data != h.data) d.upsert.push_back({b->name, h.data});
    }
    for (auto &b: base) {
        bool found = false;
        for (auto &h: host) found |= iequals(b.name, h.name);
        if (!found) d.removed.push_back(b.name);
    }
    return d;
}

}
