#pragma once

// A small service architecture (systemd-lite). A Service wraps one building block (MQTT, FTP,
// telnet, telemetry, LCD, scope, ...) and exposes start()/stop()/restart() plus a status
// (Running/Stopped/Failed). Each service has its own ESP_LOG tag (== name()) and a runtime
// log level, and an optional periodic tick() driven from the network loop (core 0).
// Per-service state (enabled flag + log level) persists in the service's own conf file.

#include <esp_log.h>

#include <vector>
#include <string>
#include <cstring>
#include <exception>

#include "conf.h"

enum class ServiceState { Stopped, Running, Failed };

// --- level <-> string helpers (only ERROR/WARN/INFO are usable; DEBUG/VERBOSE are compiled out) ---

inline const char *levelToStr(esp_log_level_t lvl) {
    switch (lvl) {
        case ESP_LOG_ERROR: return "error";
        case ESP_LOG_WARN: return "warn";
        case ESP_LOG_INFO: return "info";
        default: return "info";
    }
}

inline esp_log_level_t strToLevel(const std::string &s) {
    if (s == "error" || s == "err" || s == "e") return ESP_LOG_ERROR;
    if (s == "warn" || s == "warning" || s == "w") return ESP_LOG_WARN;
    if (s == "info" || s == "i") return ESP_LOG_INFO;
    ESP_LOGW("service", "unknown log level '%s', defaulting to info", s.c_str());
    return ESP_LOG_INFO;
}

inline const char *stateStr(ServiceState st) {
    switch (st) {
        case ServiceState::Running: return "Running";
        case ServiceState::Stopped: return "Stopped";
        case ServiceState::Failed: return "Failed";
        default: return "?";
    }
}

class Service {
public:
    // confPath e.g. "/littlefs/conf/mqtt.conf". `name` is also the ESP_LOG tag.
    Service(const char *name, const char *confPath, bool requiresNetwork = false, bool enabledDefault = true)
        : _name(name), _confPath(confPath), _requiresNetwork(requiresNetwork), _enabled(enabledDefault) {}

    virtual ~Service() = default;

    const char *name() const { return _name; }

    ServiceState state() const { return liveState(); }

    esp_log_level_t logLevel() const { return _logLevel; }

    bool enabled() const { return _enabled; }

    bool requiresNetwork() const { return _requiresNetwork; }

    // Start the service. No-op (returns true) if already Running. On exception or onStart()==false
    // the service transitions to Failed. Exceptions never escape (the network loop must not throw).
    bool start() {
        if (_state == ServiceState::Running) return true;
        esp_log_level_set(_name, _logLevel); // apply level before onStart logs
        try {
            if (onStart()) {
                _state = ServiceState::Running;
                ESP_LOGI("service", "started '%s'", _name);
                return true;
            }
            fail("onStart returned false");
        } catch (const std::exception &e) {
            fail(e.what());
        } catch (...) {
            fail("unknown exception");
        }
        return false;
    }

    // Stop the service. No-op if already Stopped. Stopping a Failed service clears it to Stopped.
    void stop() {
        if (_state == ServiceState::Stopped) return;
        try {
            onStop();
        } catch (const std::exception &e) {
            ESP_LOGE("service", "'%s' onStop error: %s", _name, e.what());
        } catch (...) {
            ESP_LOGE("service", "'%s' onStop unknown error", _name);
        }
        _state = ServiceState::Stopped;
        ESP_LOGI("service", "stopped '%s'", _name);
    }

    bool restart() {
        stop();
        return start();
    }

    // Periodic tick from the network loop. Only ticks while Running; failures move to Failed.
    void tick() {
        if (_state != ServiceState::Running) return;
        try {
            onTick();
        } catch (const std::exception &e) {
            fail(e.what());
        } catch (...) {
            fail("unknown exception in tick");
        }
    }

    // Set the runtime log level (applies via esp_log_level_set, optionally persists to conf first).
    // False, level unchanged, if it cannot be persisted.
    bool setLogLevel(esp_log_level_t lvl, bool persist = true) {
        if (persist) {
            ConfFile c{_confPath, true};
            if (!c.add({{"log_level", levelToStr(lvl)}}, true)) return false;
        }
        _logLevel = lvl;
        esp_log_level_set(_name, lvl);
        ESP_LOGI("service", "'%s' log level -> %s", _name, levelToStr(lvl));
        return true;
    }

    // Persist the desired enabled state to the conf file (does not start/stop).
    // False, state unchanged, if it cannot be persisted.
    bool setEnabledPersist(bool en) {
        ConfFile c{_confPath, true};
        if (!c.add({{"enabled", en ? "1" : "0"}}, true)) return false;
        _enabled = en;
        return true;
    }

    // Read enabled + log_level from the conf file. Called once at registration (in setup(), outside
    // any try/catch), so a malformed value must not throw out -> reboot loop; fall back to defaults.
    void loadConf() {
        ConfFile c{_confPath, /*no_warn_if_not_open*/ true};
        try {
            _enabled = c.getByte("enabled", _enabled ? 1 : 0) != 0;
            _logLevel = strToLevel(c.getString("log_level", "info"));
        } catch (const std::exception &e) {
            ESP_LOGW("service", "'%s' conf parse error (%s); using defaults", _name, e.what());
        }
        esp_log_level_set(_name, _logLevel);
    }

protected:
    // Hooks implemented by concrete services. Return false (or throw) => Failed.
    virtual bool onStart() = 0;

    virtual void onStop() = 0;

    virtual void onTick() {}

    // Override for services whose "Running" depends on a live condition (e.g. MQTT connected).
    virtual ServiceState liveState() const { return _state; }

public:
    // Optional one-line, human-readable live detail shown after the service in `svc list`
    // (e.g. "connected", client count). Empty by default.
    virtual std::string statusDetail() const { return {}; }

protected:

    void fail(const char *why) {
        _state = ServiceState::Failed;
        ESP_LOGE("service", "'%s' failed: %s", _name, why ? why : "?");
    }

    const char *_name;
    const char *_confPath;
    bool _requiresNetwork;
    ServiceState _state = ServiceState::Stopped;
    esp_log_level_t _logLevel = ESP_LOG_INFO;
    bool _enabled;
};

class ServiceManager {
    std::vector<Service *> _services;

public:
    void registerService(Service *s) {
        _services.push_back(s);
        s->loadConf();
    }

    Service *findByName(const char *n) const {
        for (auto *s: _services)
            if (strcasecmp(s->name(), n) == 0) return s;
        return nullptr;
    }

    const std::vector<Service *> &all() const { return _services; }

    // Start every service whose persisted enabled flag is set. Network services are skipped when
    // networkUp=false (they'd just transition to Failed with no recourse); they self-heal via
    // startEnabledNetworkServices() on the WiFi-up edge and stay Stopped if WiFi never comes up.
    void startEnabledAtBoot(bool networkUp) {
        for (auto *s: _services)
            if (s->enabled() && (networkUp || !s->requiresNetwork())) s->start();
    }

    // Self-heal: (re)start enabled network services that aren't Running. Called when WiFi connects.
    void startEnabledNetworkServices() {
        for (auto *s: _services)
            if (s->requiresNetwork() && s->enabled() && s->state() != ServiceState::Running)
                s->start();
    }

    // Start enabled network services that are Failed or Stopped again; returns how many were tried.
    int retryFailedNetworkServices() {
        int n = 0;
        for (auto *s: _services)
            if (s->requiresNetwork() && s->enabled() && s->state() != ServiceState::Running) {
                s->start();
                ++n;
            }
        return n;
    }

    // Stop network-requiring services while the netif is still valid — symmetric to
    // startEnabledNetworkServices(), called on the WiFi-down edge before WiFi.disconnect(). Releases
    // each service's sockets + log sink before lwip is deinited (tearing down after deinit was a UAF).
    void stopNetworkServices() {
        for (auto *s: _services)
            if (s->requiresNetwork() && s->state() != ServiceState::Stopped) s->stop();
    }

    void tickAll() {
        for (auto *s: _services)
            s->tick();
    }

    // Cleanly stop every running service (e.g. before a reboot) so their tasks tear down while the
    // network is still up. Stopping MQTT here ends mqtt_task before WiFi drops, avoiding the
    // transport-error storm that otherwise overflows its stack on the way down.
    void stopAll() {
        for (auto *s: _services)
            if (s->state() != ServiceState::Stopped) s->stop();
    }
};

inline ServiceManager g_services;
