#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

// WiFi-free telemetry core: point queue, symbol table, wire conf, device id.
// Shared by the UDP path (telemetry.cpp, CONFIG_FUGU_WITH_NETW) and the BLE stream
// (tele_ble.cpp, CONFIG_FUGU_WITH_BLE_TELE); must not include any WiFi/lwip headers.

#include "telemetry.h"
#include "tele_core.h"
#ifdef CONFIG_FUGU_WITH_BLE_TELE
#include "tele_ble.h"
#endif
#include "tele_mqtt.h"

#include <Arduino.h>

#include "../conf.h"
#include "../storage/key-value.h"

#ifdef CONFIG_FUGU_WITH_TELE_BINARY
SymbolTable g_symtab;                    // shared interning table for the binary wire (telemetry.h)
bool g_teleBinary = false;               // set by teleLoadWireConf; default = text
#endif

bool timeSynced = false;

bool g_teleUdpActive = false;

extern KeyValueStorage nvs;

std::string getDeviceId() {
    return "fugu-" + std::string(getChipId());
}

const std::string &getHostname(bool reload) {
    static std::string hostname{};
    if (hostname.empty() or reload) {
        nvs.open();
        hostname = nvs.readString("hostname", getDeviceId());
        nvs.close();
    }
    return hostname;
}

const char *getChipId() {
    static char ssid[25]{0};
    if (!strlen(ssid)) {
        // newlib-nano printf has no 64-bit support; split MAC into hi/lo 32-bit
        // words so the result stays identical to "%llX" for any 48-bit MAC.
        uint64_t mac = ESP.getEfuseMac();
        snprintf(ssid, 25, "%s-%lX%08lX", CONFIG_IDF_TARGET,
                 (unsigned long) (mac >> 32), (unsigned long) (mac & 0xFFFFFFFF));
    }
    return ssid;
}

// Load tele.conf::binary; cache so the hot path doesn't read littlefs per point.
// The UDP path additionally caches its compressor (teleUdpLoadCompressor in telemetry.cpp).
void teleLoadWireConf() {
    ConfFile conf{"/littlefs/conf/tele.conf"};
#ifdef CONFIG_FUGU_WITH_TELE_BINARY
    g_teleBinary = conf.getLong("binary", 0) != 0;
#else
    if (conf.getLong("binary", 0))
        ESP_LOGW("tele", "binary=1 needs CONFIG_FUGU_WITH_TELE_BINARY, using text");
#endif
#ifdef CONFIG_FUGU_WITH_BLE_TELE
    if (teleBleStreaming()) g_teleBinary = true;   // BLE stream requires the binary wire
#endif
#ifdef CONFIG_FUGU_WITH_TELE_MQTT
    if (teleMqttActive()) g_teleBinary = true;
#endif
}

moodycamel::ReaderWriterQueue<QueuedTelePoint> pointsQ{};

void telemetryAddPoint(TelePoint &p, uint16_t maxQueue) {
    assert(p.hasTime());
    std::string wire = p.takeWire();
#ifdef CONFIG_FUGU_WITH_BLE_TELE
    teleBleEnqueue(wire);
#endif
    teleMqttEnqueue(wire);
    if (g_teleUdpActive && pointsQ.size_approx() < maxQueue)
        pointsQ.enqueue(QueuedTelePoint{std::move(wire), uint64_t(esp_timer_get_time()) / 1000});
}

bool telePminiConfigured() {
    ConfFile conf{"/littlefs/conf/tele.conf"};
    const bool requested = conf.getLong("pmini", 1) != 0;
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
    return requested;
#else
    if (requested && conf.has("pmini")) ESP_LOGW("tele", "pmini requested but not compiled; using legacy telemetry");
    return false;
#endif
}
