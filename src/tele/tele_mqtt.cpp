#include "tele_mqtt.h"

#include <esp_heap_caps.h>
#include <esp_log.h>

#include "mqtt.h"
#include "telemetry.h"
#include "tele_core.h"
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
#include "tele_packet.h"
#include <memory>
static std::unique_ptr<TelePacketEncoder> packetEncoder;
static uint64_t firstPointMs = 0;
#endif

static const char *TAG = "telemqtt";

static constexpr size_t CAP = 1024;
static constexpr time_ms FLUSH_MS = 500;
static size_t batchCap() {
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
    if (packetEncoder) return TelePacketBatch::CAP;
#endif
    return CAP;
}

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
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
    packetEncoder.reset(); firstPointMs = 0;
#endif
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
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
            if (telePminiConfigured()) packetEncoder.reset(new TelePacketEncoder());
#endif
            buf.reserve(batchCap() + 64 + 1);
            buf.assign(1, '\0');
            topic = "pv/tele/" + getHostname();
        } catch (const std::bad_alloc &) {
            std::string().swap(buf);
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
            packetEncoder.reset();
#endif
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

static void flush(time_ms nowMs) {
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
    if (packetEncoder) {
        try {
            std::vector<std::string> packets;
            if (buf.size() > 1) {
                if (!packetEncoder->pack(buf.substr(1), 1440, packets)) dropped += buf.size() - 1;
                else for (const auto &p : packets)
                    if (!MQTT.publish(topic.c_str(), p.data(), p.size(), 0, false)) dropped += p.size();
            }
        } catch (const std::bad_alloc &) { dropped += buf.size() > 1 ? buf.size() - 1 : 0; }
        buf.assign(1, '\0'); firstPointMs = 0; lastFlushMs = nowMs;
        return;
    }
#endif
    if (buf.size() > 1 && !MQTT.publish(topic.c_str(), buf.data(), buf.size(), 0, false))
        dropped += buf.size() - 1;
    buf.assign(1, '\0');
    lastFlushMs = nowMs;
}

void teleMqttEnqueue(const std::string &frame) {
    if (!teleMqttActive()) return;
    // A frame that does not fit ships the batch first: drop-newest would preferentially lose the
    // largest frames, which are the symbol-table heartbeats a late subscriber needs.
    if (buf.size() - 1 + frame.size() > batchCap() && MQTT.isConnected()) flush(lastFlushMs);
    if (buf.size() - 1 + frame.size() > batchCap()) dropped += frame.size();
    else {
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
        if (buf.size() == 1) firstPointMs = wallClockMs();
#endif
        try { buf += frame; } catch (const std::bad_alloc &) { dropped += frame.size(); }
    }
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
    bool due = buf.size() - 1 >= CAP - 128 || nowMs - lastFlushMs > FLUSH_MS;
#ifdef CONFIG_FUGU_WITH_TELE_PMINI
    if (packetEncoder) due = buf.size() - 1 >= batchCap() || nowMs - firstPointMs >= TelePacketBatch::WAIT_MS;
#endif
    if (buf.size() > 1 && due) flush(nowMs);
}
