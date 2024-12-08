#pragma once


#include <fstream>
#include<unordered_map>
#include <numeric>
#include <functional>
#include <vector>
#include <limits>
#include <cstring>
#include <esp_log.h>


#define TAG "conf"

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


        for (auto &[key, val]: values) {
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
        return getX<long>(key, 255, strtol_2_8_10_16, true);
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
