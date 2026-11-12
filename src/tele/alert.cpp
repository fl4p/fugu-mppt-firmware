#include "alert.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

#include "esp_log.h"
#ifdef WITH_NETW
#include "mqtt.h"
#include "telemetry.h"
#endif

namespace {
constexpr uint8_t SOURCES = 2;
const char *const LVL[]{"ok", "warn", "derate", "trip", "unverified"};

struct Entry {
    const char *src;
    uint32_t t;
    AlertLevel lvl;
    bool pending;
    char msg[64];
};

Entry entries[SOURCES]{};
bool wasConnected = false;

void publish(Entry &e) {
#ifdef WITH_NETW
    if (!MQTT.isConnected()) return;
    char buf[128];
    snprintf(buf, sizeof buf, "{\"lvl\":\"%s\",\"msg\":\"%s\",\"t\":%lu}", LVL[(int) e.lvl], e.msg,
             (unsigned long) e.t);
    const std::string topic = "pv/alert/" + getHostname() + "/" + e.src;
    if (MQTT.publish(topic.c_str(), buf, 1, true)) e.pending = false;
#else
    e.pending = false;
#endif
}
}

void alert(const char *source, AlertLevel lvl, const char *fmt, ...) {
    Entry *e = nullptr;
    for (auto &x: entries)
        if (x.src && !strcmp(x.src, source)) { e = &x; break; }
    if (e && e->lvl == lvl) return;
    if (!e) {
        for (auto &x: entries)
            if (!x.src) { e = &x; break; }
        if (!e) {
            ESP_LOGE("alert", "no slot for source %s", source);
            return;
        }
        e->src = source;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e->msg, sizeof e->msg, fmt, ap);
    va_end(ap);
    for (char *c = e->msg; *c; ++c)
        if (*c == '"' || *c == '\\' || (uint8_t) *c < 0x20) *c = '\'';
    e->lvl = lvl;
    e->t = (uint32_t) time(nullptr);
    e->pending = true;
    if (lvl == AlertLevel::Ok) ESP_LOGI("alert", "%s %s: %s", source, LVL[(int) lvl], e->msg);
    else ESP_LOGW("alert", "%s %s: %s", source, LVL[(int) lvl], e->msg);
    publish(*e);
}

void alertTick() {
#ifdef WITH_NETW
    const bool conn = MQTT.isConnected();
    const bool edge = conn && !wasConnected;
    wasConnected = conn;
    if (!conn) return;
    for (auto &e: entries)
        if (e.src && (e.pending || edge)) publish(e);
#endif
}
