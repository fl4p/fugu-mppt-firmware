#pragma once

//#include "LittleFS.h"
//LittleFS fs;

/*
 * To implement an energy counter that survives power loss, we can use the flash memory.
 * ESP32 WROOM flash chips contain flash chips from different manufacturers.
 * All their specs seem to be similar:
 * - 100k program-erase cycles
 * - 20y data retention
 * - 256 bytes programmable pages
 * - 4KB sectors (16 pages)
 * - 32/64K erasure blocks
 *
 * Since we can only write a flash page 100k times before it is worn out, we need to distribute writes accross multiple
 * sectors or blocks (wear leveling). SPIFFS implements wear leveling and one update consumes 2 pages (payload and meta data).
 * A problem with SPIFFS is that it erases whole 64k, which can take 0.25s (GD25Q32C). It is also out-dated. FAT has problems
 * with power loss. Choose LittleFS, which has the same latency issue as SPIFFS, since it clears whole blocks.
 * If we carefully inspect values and choose the right moments (impending power loss), we can reduce writes to a minimum
 * and achieve >30y estimated life span.
 * Below is a low-latency DIY Proposal, which I have not tested nor implemented.
 *
 * DIY Approach
 * Once any data bit changes, a whole page needs to be programmed. This means the ideal payload is 256 bytes (64 floats,ints).
 * For a flash wear estimate lets assume a write-interval of 1 minute and a payload of up to 255 bytes.
 * To prevent data loss due to power loss, we need at least 2 sectors. (COW)
 * Allocating two 4KB sectors means that every 32min ( 2*256b/4kb / 1/min ) a page need to be erased, which wear out the flash after 6 years.
 * Allocating 8 sectors increases life-time to 24 years.
 * Because ESP32-WROOM has plenty of flash storage, using the LittleFS approach.
 *
 *
 * https://www.esp32.com/viewtopic.php?t=386#p1776
 *
 * GD25Q32C (https://github.com/espressif/esp-idf/blob/d2471b11e78fb0af612dfa045255ac7fe497bea8/components/spi_flash/spi_flash_chip_gd.c#L30)
 * - Minimum 100,000 Program/Erase Cycles
 * - 20-year data retention typical
* - 256 Bytes per programmable page
 * - Uniform Sector of 4K-Byte
 * - Uniform Block of 32/64K-Byte
 * - Page Program time: 0.6ms typical
 * - Sector Erase time: 50ms typical
 * - Block Erase time: 0.15/0.25s typical
 *
 *
 * W25Q128JV
 * - 256bytes page
 * - 4KB sector erase
 * - Uniform Sector/Block Erase (4K/32K/64K-Byte)
 *  Min. 100K Program-Erase cycles per sector – More than 20-year data retention
 *
 *
 *
 * little fs
 * https://github.com/littlefs-project/littlefs
 *
 */

#include <unistd.h> // fsync
#include <sys/stat.h>
#include <cstdio>
#include <functional>
#include <esp32-hal.h>
#include "esp_littlefs.h"
#include "LittleFS.h"

#define TAG "store" // see undef below

static bool mountLFS(const char *part_label = "littlefs", bool format = false) {

    if (format) {
        ESP_LOGI("store", "Formatting partition %s", part_label);
        if (strstr(part_label, "test") == nullptr) {
            ESP_LOGI("store", "wait 5s for confirmation");
            delay(5000);
        }
        if (esp_littlefs_format(part_label) != ESP_OK) {
            ESP_LOGE("store", "formatting %s failed!", part_label);
            return false;
        } else {
            ESP_LOGI("store", "Formatting done!");
        }
    }


    assert(LittleFS.begin(true, "/littlefs", 10, "littlefs"));

    /*
    esp_vfs_littlefs_conf_t conf = {
            .base_path = "/littlefs",
            .partition_label = part_label,
            .format_if_mount_failed = true,
            .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition '%s'", conf.partition_label);
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }
     */

    return true;
}

/**
 * Power-loss proof value storage
 *
 * @tparam T
 */
template<typename T>
class FlashValueFile {
public:
    const char *const fn;

    FlashValueFile(const char *fn) : fn(fn) {}


    bool load(T &value, bool checkFileSize = false) {
        if (checkFileSize) {
            struct stat st;
            if (stat(fn, &st) != 0) {
                ESP_LOGI("store", "File %s doesn't exists (unable to stat)", fn);
                return false;
            }
            if (st.st_size != sizeof(T)) {
                ESP_LOGW("store", "File-size %d != sizeof(T) %d", (int) st.st_size, (int) sizeof(T));
                return false;
            }
        }
        FILE *f = fopen(fn, "r");
        if (!f)
            return false;
        size_t r = fread(&value, sizeof(T), 1, f);
        fclose(f);
        if (r != 1) {
            ESP_LOGW("store", "Unexpected read size %u", r);
        }
        return r == 1;
    }


    bool update(T val) {

        // a power-loss safe way of updating a file
        // can't use fopen(..."a"), see https://stackoverflow.com/questions/5532371/does-fseek-move-the-file-pointer-to-the-beginning-of-the-file-if-it-was-opened/5532426#5532426
        FILE *f = fopen(fn, "r+");

        if (f == nullptr) {
            f = fopen(fn, "w");
            if (f == nullptr)
                return false;
        }
        if (fwrite(&val, sizeof(T), 1, f) != 1) {
            fclose(f);
            return false;
        }
        if (ftruncate(fileno(f), sizeof(T)) != 0) {
            fclose(f);
            return false;
        }

#ifndef CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE
        if (fsync(fileno(f)) != 0) {
            fclose(f);
            return false;
        }
#endif

        fclose(f);

        ESP_LOGI("store", "Wrote %s (size %i)", fn, (int) sizeof(T));

        return true;
    }

    /*
    void ensure_file() {
        //struct stat st;
        if (stat(fn, &st) != 0) {
            FILE *f = fopen(fn, "w");
            assert(f != NULL);
            fclose(f);
        }
    }
     */
};


/**
 * Flash value file with similarity check to reduce number of writes
 * @tparam T
 */
template<typename T>
class FlashValueStore {
    typedef bool SimilarFunc(const T &a, const T &b);

    T valueFlash;

    FlashValueFile<T> file;

    std::function<bool(const T &a, const T &b)> similar;

    unsigned long _lastWrite = 0;
    unsigned long _minWriteIntervalMs;

    //static std::function<SimilarFunc> neverSimilar = [](const T &a, const T &b) { return false; };

public:
    FlashValueStore(const char *fn,
                    std::function<bool(const T &a, const T &b)> similar = [](const T &a, const T &b) { return false; },
                    unsigned long minWriteIntervalMs = 0)
            : file(fn), similar(similar), _minWriteIntervalMs(minWriteIntervalMs) {}


    bool load(bool checkFileSize = false) {
        return file.load(valueFlash, checkFileSize);
    }

    const T &getFlashValue() { return valueFlash; }

    bool update(T val) {
        auto now = millis();
        if (_lastWrite && (now - _lastWrite < _minWriteIntervalMs)) {
            return false;
        }

        if (similar(val, valueFlash))
            return false;

        if (file.update(val)) {
            ESP_LOGI("flash", "Wrote flash value %s", file.fn);
            valueFlash = val;
            _lastWrite = now;
        } else {
            ESP_LOGW("flash", "Error storing %s", file.fn);
            return false;
        }

        return true;
    }
};


#include <fstream>
#include<unordered_map>
#include <numeric>

static std::string trim(const std::string &s) { // removes whitespace characters from beginnig and end of string s
    const int l = (int) s.length();
    int a = 0, b = l - 1;
    char c;
    while (a < l && ((c = s[a]) == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r' || c == '\0'))
        a++;
    while (b > a && ((c = s[b]) == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r' || c == '\0'))
        b--;
    return s.substr(a, 1 + b - a);
}

class ConfFile {

    std::unordered_map<std::string, std::string> _map;
    const char *path;
public:
    explicit ConfFile(const char *path, bool no_warn_if_not_open = false) : path(path) {
        std::ifstream file(path);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                auto ic = line.find_first_of('#');
                if (ic != std::string::npos)
                    line = line.substr(0, ic);
                line = trim(line);
                if (line.length() == 0)
                    continue;
                auto ie = line.find_first_of('=');
                if (ie == std::string::npos) {
                    ESP_LOGE(TAG, "error reading %s: '=' not found in line '%s'", path, line.c_str());
                    assert(false);
                }
                auto k = trim(line.substr(0, ie));
                if (_map.find(k) != _map.end()) {
                    ESP_LOGE(TAG, "duplicate key %s in file '%s'", k.c_str(), path);
                    assert(false);
                }
                _map[k] = trim(line.substr(ie + 1));
            }
            file.close();
        } else {
            if (!no_warn_if_not_open) {
                ESP_LOGW(TAG, "cannot read ConfFile %s", path);
            }
        }
    }


    void add(const std::unordered_map<std::string, std::string> &values) {
        FILE *f = fopen(path, "a");
        if (f == nullptr) {
            f = fopen(path, "w");
            if (f == nullptr) {
                ESP_LOGE("store", "Cannot write %s", path);
            }
            assert (f != nullptr);
        }


        for (auto [key, val]: values) {
            if (_map.find(key) != _map.end()) {
                ESP_LOGE(TAG, "cannot add duplicate key %s", key.c_str());
                //assert(false);
                throw std::runtime_error("duplicate key: " + key);
            }
            fputc('\n', f);
            assert(fwrite(key.c_str(), key.length(), 1, f) == 1);
            assert(fwrite(" = ", 3, 1, f) == 1);
            assert(fwrite(val.c_str(), val.length(), 1, f) == 1);
        }

#ifndef CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE
        if (fsync(fileno(f)) != 0) {
            fclose(f);
            assert(false);
        }
#endif

        fclose(f);


/*
        std::ofstream file(path, std::ios_base::app);
        if(!file.is_open()) file = std::ofstream (path, )
        assert(file.is_open());
        for (auto [key, val]: values) {
            if (_map.find(key) != _map.end()) {
                ESP_LOGE(TAG, "cannot add duplicate key %s", key.c_str());
                assert(false);
            }
            file << std::endl << key << " = " << val;
        }
         */
    }

    template<typename T>
    // # (const char *, char **)
    T getX(const std::string &key, T def, const std::function<T(const char *, char **)> &strto_,
           bool noDef = false) const {
        // strto_ error handling https://stackoverflow.com/questions/26080829/detecting-strtol-failure
        auto i = _map.find(key);
        if (i != _map.end()) {
            char *endptr = nullptr;
            errno = 0; // reset
            T l = strto_(i->second.c_str(), &endptr);
            if (errno != 0) {
                ESP_LOGE(TAG, "%s:%s: strto_(\"%s\") failed: ret=%f, errno=%i", path, key.c_str(), i->second.c_str(),
                         (float) l, errno);
                throw std::runtime_error("strto_ error " + i->second);
            }
            if (*endptr != 0) {
                ESP_LOGE(TAG, "additional chars after strtol(%s): '%s'", i->second.c_str(), endptr);
                //assert(false);
                throw std::runtime_error("additional chars " + i->second);
            }
            return l;
        }

        if (!noDef && def == std::numeric_limits<T>::max()) {
            auto v = keys();
            std::string s = std::accumulate(v.begin(), v.end(), std::string{});
            ESP_LOGE(TAG, "key '%s' not found in %s (%s)", key.c_str(), s.c_str(), path);
            //assert(false);
            throw std::runtime_error("key not found: " + key);
        }

        return def;
    }

    std::vector<std::string> keys() const {
        std::vector<std::string> keys{_map.size()};
        std::transform(_map.begin(), _map.end(), keys.begin(),
                       [](const std::pair<std::string, std::string> &p) { return p.first; });
        return keys;
    }

    //inline static long strtol_10(const char *s, char **endptr) { return strtol(s, endptr, 10); }

    inline static long strtol_2_8_10_16(const char *s, char **endptr) {
        int off = 0, base = 10;
        auto len = strlen(s);

        if (len > 2 and strncmp(s, "0b", 2) == 0) {
            base = 2;
            off = 2;
        } else if (len > 2 and strncmp(s, "0x", 2) == 0) {
            base = 16;
            off = 2;
        } else if (len > 1 && s[0] == '0' && strchr(s, '.') == nullptr && strchr(s, 'e') == nullptr) {
            // valid floats (not octal): 0.1, 01e1
            off = 1;
            base = 8;
        }
        return strtol(s + off, endptr, base);
    }

    long getLong(const std::string &key, long def = std::numeric_limits<long>::max()) const {
        return getX<long>(key, def, strtol_2_8_10_16);
    }

    uint8_t getByte(const std::string &key) const {
        return getX<long>(key, 0, strtol_2_8_10_16, true);
    }

    uint8_t getByte(const std::string &key, uint8_t def) const {
        return getX<long>(key, def, strtol_2_8_10_16);
    }

    long getLong(const std::string &key, int base, long def) {
        auto fn = [base](const char *s, char **endptr) { return std::strtol(s, endptr, base); };
        return getX<long>(key, def, fn);
    }

    float getFloat(const std::string &key, float def = std::numeric_limits<float>::max()) const {
        return getX<float>(key, def, std::strtof);
    }

    float f(const std::string &key, float def = std::numeric_limits<float>::max()) { return getFloat(key, def); }

    const std::string &getString(const std::string &key) const {
        auto i = _map.find(key);
        if (i != _map.end())
            return i->second;
        throw std::runtime_error("key not found: " + key);
    }

    const std::string &getString(const std::string &key, const std::string &def) const {
        auto i = _map.find(key);
        if (i != _map.end()) {
            return i->second;
        }
        return def;
        //ESP_LOGE(TAG, "key '%s' not found", key.c_str());
        //assert(false);
    }

    const char *c(const std::string &key, const char *def = nullptr) {
        auto i = _map.find(key);
        if (i != _map.end()) {
            return i->second.c_str();
        }
        return def;
    }

    explicit operator bool() const { return !_map.empty(); }
};

#undef TAG