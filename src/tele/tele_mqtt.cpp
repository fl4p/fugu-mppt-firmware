#include "tele_mqtt.h"

#include <esp_heap_caps.h>
#include <esp_log.h>

#include "mqtt.h"
#include "telemetry.h"
#include "tele_core.h"

static const char *TAG = "telemqtt";

static constexpr size_t CAP = 1024;
static constexpr time_ms FLUSH_MS = 500;

// buf[0] is the compressor id (0 = uncompressed), kept so a publish needs no second buffer
static std::string buf;
static std::string topic;
static time_ms leaseUntilMs = 0;   // 0 = inactive
static time_ms lastFlushMs = 0;
static size_t dropped = 0;
static bool wasConnected = false;

bool teleMqttActive() { return leaseUntilMs != 0; }

size_t teleMqttDropped() { return dropped; }

uint32_t teleMqttLeaseLeftS(time_ms nowMs) {
    return leaseUntilMs > nowMs ? (uint32_t) ((leaseUntilMs - nowMs + 999) / 1000) : 0;
}

static void stop() {
    leaseUntilMs = 0;
    std::string().swap(buf);
    teleLoadWireConf();
}

const char *teleMqttLease(uint32_t seconds) {
    if (!seconds) {
        if (teleMqttActive()) stop();
        return nullptr;
    }
    if (!timeSynced)
        return "clock not set (send `set-time <epoch_ms>` or wait for SNTP)";
    if (!MQTT.isConnected())
        return "mqtt not connected";
    if (g_teleUdpActive && !g_teleBinary)
        return "UDP telemetry runs the text wire (set-config tele.conf binary 1, svc rs tele)";
    if (!teleMqttActive()) {
        if (heap_caps_get_free_size(MALLOC_CAP_DEFAULT) < 16 * 1024)
            return "low heap";
        try {
            buf.reserve(CAP + 64 + 1);
            buf.assign(1, '\0');
            topic = "pv/tele/" + getHostname();
        } catch (const std::bad_alloc &) {
            std::string().swap(buf);
            return "out of memory";
        }
        g_teleBinary = true;
        g_symtab.forceResend();
        dropped = 0;
        wasConnected = true;
    }
    if (seconds > 3600) seconds = 3600;
    leaseUntilMs = wallClockMs() + seconds * 1000;
    ESP_LOGI(TAG, "tele-mqtt: lease %u s", (unsigned) seconds);
    return nullptr;
}

void teleMqttEnqueue(const std::string &frame) {
    if (!teleMqttActive()) return;
    if (buf.size() - 1 + frame.size() > CAP) dropped += frame.size();
    else buf += frame;
}

void teleMqttTick(time_ms nowMs) {
    if (!teleMqttActive()) return;
    bool expired = nowMs >= leaseUntilMs;
    if (expired || MQTT.state() != ServiceState::Running) {
        ESP_LOGI(TAG, "tele-mqtt: %s, off", expired ? "lease expired" : "mqtt stopped");
        stop();
        return;
    }
    if (!MQTT.isConnected()) {
        buf.assign(1, '\0');
        wasConnected = false;
        return;
    }
    if (!wasConnected) {   // a subscriber may have joined meanwhile: restart with the table
        buf.assign(1, '\0');
        g_symtab.forceResend();
        wasConnected = true;
    }
    teleProducePoint();
    if (buf.size() > 1 && (buf.size() - 1 >= CAP - 128 || nowMs - lastFlushMs > FLUSH_MS)) {
        if (!MQTT.publish(topic.c_str(), buf.data(), buf.size(), 0, false))
            dropped += buf.size() - 1;
        buf.assign(1, '\0');
        lastFlushMs = nowMs;
    }
}
