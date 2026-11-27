#pragma once

// Firmware naming policy for the virtual FAT12 config drive: the core in components/virtual-fat12
// stays generic, these are the ".conf" rules it is configured with. Pure C++ (no IDF), so host
// tests can include it.

#include <string>
#include <vector>

#include "vfat/vfat.h"

namespace confdrive {

namespace detail {

inline char lower(char c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; }

inline bool iequals(const std::string &a, const std::string &b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) if (lower(a[i]) != lower(b[i])) return false;
    return true;
}

}

// [A-Za-z0-9_-]+\.conf, 6..64 chars
inline bool validName(const std::string &n) {
    if (n.size() < 6 || n.size() > 64 || !detail::iequals(n.substr(n.size() - 5), ".conf")) return false;
    for (size_t i = 0; i < n.size() - 5; ++i) {
        char c = n[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-'))
            return false;
    }
    return true;
}

// Strict key=value check (printable ASCII, every non-comment line has a key and '=').
inline bool validConfText(const std::string &t, std::string &err) {
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

// Volume::build() returns false when it skips a name that differs only in case from one already
// laid out, and the firmware caller must not mount a drive that drops a .conf the user can see.
// buildVolume() calls this over the same validName() candidates it feeds build() and refuses the
// whole drive instead. Returns true and names both files (with the fix) when such a pair exists;
// the caller must already have filtered `files` with validName(), exactly as buildVolume() does.
inline bool caseAliasConflict(const std::vector<vfat::File> &files, std::string &err) {
    for (size_t i = 0; i < files.size(); ++i)
        for (size_t j = i + 1; j < files.size(); ++j)
            if (detail::iequals(files[i].name, files[j].name)) {
                err = files[i].name + " and " + files[j].name + " differ only in case; rename or remove one";
                return true;
            }
    return false;
}

inline vfat::Naming naming() { return {".conf", validName}; }

}
