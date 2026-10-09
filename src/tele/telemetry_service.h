#pragma once

// TelemetryService: governs the core-0 InfluxDB point flush over UDP (connectionless, so onStart
// only needs Wi-Fi up). Touches the `mppt` global (defined in main.cpp).

#include <WiFi.h>

#include "../service.h"
#include "../mppt.h"
#include "telemetry.h"
#include "tele_core.h"

void teleUdpLoadCompressor();  // telemetry.cpp: cache the UDP wire's compressor

extern MpptController mppt;

class TelemetryService : public Service {
public:
    TelemetryService() : Service("tele", "/littlefs/conf/tele.conf", /*requiresNetwork*/ true, false) {}

    // The UDP target it flushes points to, once a host is configured/resolved.
    std::string statusDetail() const override {
        const auto &host = mppt.tele.influxdbHost;
        return host ? std::string("→ ") + host.toString().c_str() : std::string();
    }

protected:
    bool onStart() override {
        if (!WiFi.isConnected()) return false;          // UDP is connectionless, nothing to open
        onStop();                                        // a failed tick leaves flushTask alive
        mppt.tele = TeleConf{ConfFile{"/littlefs/conf/tele.conf"}};
        teleLoadWireConf();                              // pick text|binary from tele.conf (binary wire = tamp)
        teleUdpLoadCompressor();
        // A restarted service sends from a new UDP source port -> the proxy starts a fresh
        // symbol table; burst the full table so it isn't blind until the 120 s heartbeat.
        // (g_symtab is shared: the burst also lands on a concurrent BLE stream — harmless,
        // the decoder tolerates repeated tables; costs it a few hundred bytes.)
        g_symtab.forceResend();
        xTaskCreatePinnedToCore(flushTask, "teleflush", 4096, this, 1, &_flushTask, 0);
        g_teleUdpActive = true;
        return true;
    }
    void onStop() override {                            // delete the task -> frees CPU + its stack
        g_teleUdpActive = false;
        if (_flushTask) { vTaskDelete(_flushTask); _flushTask = nullptr; }
    }
    void onTick() override {
        mppt.telemetry();   // produce only; flushTask does the compress+send
    }

private:
    // Transport: the core-0 task loop and the per-tick batch+send. Defined in telemetry.cpp,
    // where the point queue / UDP socket / compressor statics live (the header stays lean).
    [[noreturn]] static void flushTask(void *arg);
    void flushQueue(const IPAddress &addr);

    TaskHandle_t _flushTask = nullptr;
};

inline TelemetryService telemetryService;
