#pragma once

// Emulated FAT12 volume over a set of in-memory files. Pure C++ (no IDF), so it builds on the host.
// build() lays the files out contiguously and keeps them immutable; host writes land in RAM (FAT, root dir, overlay of data
// sectors). collect() parses the host's view back into files, diff() compares it with the baseline.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vfat {

constexpr uint16_t SECTOR = 512;

struct File {
    std::string name;
    std::string data;
};

struct Diff {
    std::vector<File> upsert;
    std::vector<std::string> removed;

    bool empty() const { return upsert.empty() && removed.empty(); }
};

// [A-Za-z0-9_-]+\.conf
bool validName(const std::string &name);

// Strict key=value check (printable ASCII, every non-comment line has a key and '=').
bool validConfText(const std::string &text, std::string &err);

class Volume {
public:
    static constexpr uint32_t TOTAL_SECTORS = 4096;
    static constexpr uint32_t FAT_SECTORS = 12;
    static constexpr uint32_t ROOT_ENTRIES = 64;
    static constexpr uint32_t ROOT_SECTORS = ROOT_ENTRIES * 32 / SECTOR;
    static constexpr uint32_t FAT1 = 1, FAT2 = FAT1 + FAT_SECTORS, ROOT = FAT2 + FAT_SECTORS;
    static constexpr uint32_t DATA = ROOT + ROOT_SECTORS;
    static constexpr uint32_t CLUSTERS = TOTAL_SECTORS - DATA;
    static_assert(CLUSTERS < 4085 - 16, "must stay FAT12");
    static_assert((CLUSTERS + 2) * 3 / 2 <= FAT_SECTORS * SECTOR, "FAT too small");

    explicit Volume(uint16_t maxOverlaySectors = 128) : _maxOverlay(maxOverlaySectors) {}

    // Lays out files with valid names. false if some were skipped (invalid name or no space).
    bool build(std::vector<File> files, uint32_t volumeId);

    void clear();

    bool read(uint32_t lba, uint8_t *buf) const;

    // false: out of range or overlay full
    bool write(uint32_t lba, const uint8_t *buf);

    // Host view of all valid *.conf entries. false (with err) if any of them is inconsistent.
    bool collect(std::vector<File> &out, std::string &err) const;

    // the files the volume was built from
    const std::vector<File> &baseline() const { return _img; }

    size_t overlaySectors() const { return _overlay.size(); }

private:
    struct Sec {
        uint32_t lba;
        uint8_t d[SECTOR];
    };

    uint16_t fatGet(uint32_t c) const;
    void fatSet(uint32_t c, uint16_t v);
    uint8_t *overlayFind(uint32_t lba) const;
    void readData(uint32_t lba, uint8_t *buf) const;

    uint16_t _maxOverlay;
    uint32_t _volId = 0;
    std::vector<File> _img;
    std::vector<uint16_t> _imgCluster;
    std::vector<uint8_t> _fat, _root;
    std::vector<std::unique_ptr<Sec>> _overlay;
};

// host vs base; names compare case-insensitively, a changed file keeps its base name
Diff diff(const std::vector<File> &base, const std::vector<File> &host);

}
