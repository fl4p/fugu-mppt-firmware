#include "telemetry.h"
#include "telemetry_service.h"   // transport methods (flushTask/flushQueue) are defined here

#include "../adc/sampling.h"
#include "line_protocol.h"
#include "../store.h"

#include <WiFi.h>
//#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include <AsyncUDP.h>

//#include <Preferences.h>

#include <Arduino.h>

#include "../etc/readerwriterqueue.h"

#include "../storage/key-value.h"

#include "compress.h"
#include "../conf.h"
#include "tele_core.h"
#include "../etc/idle_profile.h"
static Compressor *s_teleCompressor = nullptr;   // cached compressor for the binary wire


// ssid, psk. Replaces WiFiMulti, whose fixed 100-300 ms/channel scan dwell starves a connected BLE
// link: the S3 controller then asserts in the connection-update procedure (INT_WDT reset).
static std::vector<std::pair<std::string, std::string>> s_aps;

static void addAp(const std::string &ssid, const std::string &psk) {
    for (auto &ap: s_aps)
        if (ap.first == ssid) {
            ap.second = psk;
            return;
        }
    s_aps.emplace_back(ssid, psk);
}

// Scan at the driver's default dwell (coexistence-safe), join the strongest configured AP.
static wl_status_t wifiRun() {
    if (WiFi.status() == WL_CONNECTED) return WL_CONNECTED;
    WiFi.setScanActiveMinTime(0);
    const int n = WiFi.scanNetworks(false, false, false, 0);
    int best = -1, bestAp = -1;
    for (int i = 0; i < n; ++i)
        for (size_t k = 0; k < s_aps.size(); ++k)
            if (s_aps[k].first == WiFi.SSID(i).c_str() && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best))) {
                best = i;
                bestAp = (int) k;
            }
    if (best < 0) {
        WiFi.scanDelete();
        return WL_NO_SSID_AVAIL;
    }
    const auto &ap = s_aps[bestAp];
    WiFi.begin(ap.first.c_str(), ap.second.empty() ? nullptr : ap.second.c_str(), WiFi.channel(best),
               WiFi.BSSID(best));
    WiFi.scanDelete();
    return (wl_status_t) WiFi.waitForConnectResult(5000);
}
//WiFiUDP udp;
AsyncUDP asyncUdp;

bool noSsid = true;
static bool sntpSynced = false;   // SNTP ran to completion; separate from timeSynced, which a
                                  // BLE `set-time` can also set (must not suppress NTP)

// Grace period to keep retrying the AP we lost before roaming to another configured network
// (the router might just be rebooting). 0 disables. Set via wifi.conf::switch_delay (seconds).
static uint32_t wifiSwitchDelayMs = 30 * 1000;

IPAddress ha_host{};

extern KeyValueStorage nvs;

void wifi_load_conf() {
    ConfFile wifiConf{"/littlefs/conf/wifi.conf"};

    auto starts_with = [](const std::string &s, const std::string &t) { return s.substr(0, t.length()) == t; };
    auto ends_with = [](const std::string &s, const std::string &t) { return s.substr(s.length() - t.length()) == t; };

    noSsid = true;

    wifiSwitchDelayMs = (uint32_t) wifiConf.getLong("switch_delay", 30) * 1000;

    nvs.open();
    auto ssid_def = nvs.readString("wifi_ssid", "");
    auto psk_def = nvs.readString("wifi_psk", "");
    nvs.close();
    if (!ssid_def.empty()) {
        addAp(ssid_def, psk_def);
        noSsid = false;
    }

    for (const auto &k: wifiConf.keys()) {
        if (starts_with(k, "ssid") && !ends_with(k, "_psk")) {
            auto ssid = wifiConf.getString(k); // keep the string alive; .c_str() on the temp dangled
            auto psk = wifiConf.c(k + "_psk", "");
            if (ssid == ssid_def and psk == psk_def) continue;
            addAp(ssid, psk ? psk : "");
            ESP_LOGI("tele", "Add WiFi SSID %s (psk %s)", ssid.c_str(), psk && *psk ? "***" : "<none>");
            noSsid = false;
        }
    }
}

// The key is a sanitized label — the loader above reads the SSID from the VALUE, so '='/spaces
// in an SSID can't corrupt the key=value format. '#' starts a comment in the conf parser and is
// unstorable (no escaping): reject instead of persisting a truncated credential. Overwrites an
// existing entry (psk update). Two single-pair add() calls: a 2-pair KVList literal with runtime
// values miscompiles on xtensa 14.2 (throws length_error).
bool add_ap(const std::string &ssid, const std::string &psk) {
    auto confPath = "/littlefs/conf/wifi.conf";
    if (ssid.empty() || ssid.find('#') != std::string::npos || psk.find('#') != std::string::npos ||
        trim(ssid) != ssid || trim(psk) != psk) {
        ESP_LOGE("tele", "ssid/psk with '#' or leading/trailing whitespace cannot be stored");
        return false;
    }
    std::string label = "ssid_";
    for (char c: ssid) label += (char) (isalnum((unsigned char) c) ? c : '_');
    ConfFile wifiConf{confPath, true};
    // different SSIDs can sanitize to the same label; suffix instead of silently clobbering
    auto base = label;
    for (int i = 2; !wifiConf.getString(label, ssid).empty() &&
                    wifiConf.getString(label, ssid) != ssid && i < 10; ++i)
        label = base + std::to_string(i);
    const bool known = wifiConf.getString(label, "") == ssid;
    // psk first: if the second write fails, an orphan _psk key is ignored by the loader,
    // while an orphan ssid key would be joined as an open network
    if (!wifiConf.add({{label + "_psk", psk}}, true) ||
        !wifiConf.add({{label, ssid}}, true)) {
        ESP_LOGE("tele", "failed to write %s", confPath);
        return false;
    }
    addAp(ssid, psk);
    ESP_LOGI("tele", "%s Wifi AP %s in %s", known ? "Updated" : "Added", ssid.c_str(), confPath);
    noSsid = false;
    return true;
}

void connect_wifi_async() {
    sntpSynced = false;   // force a fresh NTP sync after (re)connect; clock stays valid
    if (noSsid) wifi_load_conf();
    if (!noSsid) {
        NetworkManager::setHostname(getHostname().c_str());
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(WIFI_PS_MIN_MODEM); // modem-sleep between DTIM beacons; cuts RF idle power/heat, keeps console+MQTT latency low
    }
}

void disconnect_wifi(bool off) {
    MDNS.end(); // unregister netif hooks + leave the mcast group while the netif is still valid
    WiFi.disconnect(off);
}


static bool timeSyncAsync(const char *tzInfo, const char *ntpServer1, const char *ntpServer2 = nullptr,
                   const char *ntpServer3 = nullptr) {
    static unsigned long tSyncStarted = 0;

    if (tSyncStarted && time(nullptr) > 1000000000l) {
        // fall through to the success branch: a sync that completed just before a link
        // drop still counts, no need to restart it
    } else if (!WiFi.isConnected()) {
        tSyncStarted = 0; // don't let a link drop age into a bogus timeout
        return false;
    }

    if (!tSyncStarted) {
        ESP_LOGI("tele", "Starting time sync");
        tSyncStarted = millis() + 1;
        configTzTime(tzInfo, ntpServer1, ntpServer2, ntpServer3);
    } else if (time(nullptr) > 1000000000l) {
        struct tm timeinfo;
        time_t nowSecs = time(nullptr);
        gmtime_r(&nowSecs, &timeinfo);
        ESP_LOGI("tele", "Time synced! %s UTC", asctime(&timeinfo));
        tSyncStarted = 0;
        return true;
    } else if ((millis() - tSyncStarted) > (20 * 1000)) {
        ESP_LOGW("tele", "Timeout syncing time! (%s)", ntpServer1);
        tSyncStarted = 0;
    }
    return false;
}


static void _wifiConnected() {
    if (!WiFi.isConnected()) return;

    String hostname = String(getHostname().c_str());

    MDNS.end();
    if (!MDNS.begin(hostname.c_str())) {
        // abc.local
        ESP_LOGE("tele", "Error setting up MDNS responder!");
    } else {
        ESP_LOGI("tele", "Set hostname %s", hostname.c_str());
    }

    MDNS.setInstanceName(hostname);

    ha_host = MDNS.queryHost("homeassistant.local");
    ESP_LOGI("tele", "%s resolved to %s", "homeassistant.local", ha_host.toString().c_str());

    // telnet / ftp / scope are now started by the ServiceManager (self-heals on the WiFi-up edge,
    // see loopNetwork_task). Their onStart() relies on the MDNS setup above being done first.
}

void wifiLoop(bool connect) {
    //static bool initialized = false;
    if (noSsid) return;

    if (connect && !WiFi.isConnected()) {
        wait_for_wifi();
    }

    if (unlikely(!sntpSynced)) {
        if (timeSyncAsync("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nis.gov")) {
            sntpSynced = true;
            timeSynced = true;
        }
    }

    //if (unlikely(!initialized) && WiFi.isConnected()) {
    //    initialized = true;
    //    _wifiConnected();
    // }
}

bool wait_for_wifi() {
    static unsigned long lastTimeout = 0;
    static unsigned long disconnectedSince = 0;
    static std::string prevSsid;

    if (noSsid) return false;

    if (!disconnectedSince) disconnectedSince = millis();

    // While the AP we last held may just be rebooting, keep retrying that same network (via the
    // station's stored config) instead of roaming to another configured AP. wifiRun() rescans
    // and picks the strongest visible AP, so only fall back to it once switch_delay has elapsed.
    bool stick = wifiSwitchDelayMs and !prevSsid.empty()
                 and (millis() - disconnectedSince < wifiSwitchDelayMs);

    // 30 s backoff after a failed attempt before rescanning/roaming - but while sticking, skip it so
    // we keep retrying the lost AP and re-acquire it the moment it finishes rebooting.
    if (lastTimeout and !stick and (micros() - lastTimeout < 30 * 1000 * 1000)) return false;

    ESP_LOGI("tele", "Connecting WiFi...");
    auto t_start = millis();
    if (stick) WiFi.reconnect();
    while ((stick ? WiFi.status() : wifiRun()) != WL_CONNECTED) {
        delay(50);
        if (millis() - t_start > 6000) {
            ESP_LOGW("tele", "WiFi connection timeout");
            if (!stick) lastTimeout = micros();
            return false;
        }
    }

    disconnectedSince = 0;
    prevSsid = WiFi.SSID().c_str();
    ESP_LOGI("tele", "Connected to WiFi %s, RSSI %d IP %s", prevSsid.c_str(), (int) WiFi.RSSI(),
             WiFi.localIP().toString().c_str());

    _wifiConnected();

    return true;
}

static void udpFlushString(const IPAddress &host, uint16_t port, String &msg) {
    if (msg.length() > CONFIG_TCP_MSS) {
        ESP_LOGW("tele", "Payload len %d > TCP_MSS: %s", msg.length(), msg.substring(0, 200).c_str());
        msg.clear();
        return;
    }

    bytesSent += asyncUdp.writeTo((uint8_t *) msg.c_str(), msg.length(), host, port);

    //udp.beginPacket(host, port);
    //udp.print(msg);
    //udp.endPacket();

    msg.clear();
}

// Cache the UDP wire's compressor at service start (after teleLoadWireConf). The binary
// wire is always tamp-compressed; on text the compressor stays null.
void teleUdpLoadCompressor() {
#ifdef WITH_TELE_BINARY
    s_teleCompressor = g_teleBinary ? &compressorByName("tamp") : nullptr;
#endif
}

// Compress one batch of concatenated (already length-prefixed) wire frames, tag
// it with the compressor id, and send as one self-contained UDP datagram.
static void sendBinaryBatch(const IPAddress &dst, uint16_t port, Compressor &comp, std::string &raw) {
    if (raw.empty()) return;
    std::string out(1, (char) comp.id());    // 1-byte tag so the receiver picks the decompressor
    if (comp.packet((const uint8_t *) raw.data(), raw.size(), out)) {
        if (out.size() > CONFIG_TCP_MSS)
            ESP_LOGW("tele", "binary datagram %u B > MSS (raw %u)", (unsigned) out.size(), (unsigned) raw.size());
        bytesSent += asyncUdp.writeTo((const uint8_t *) out.data(), out.size(), dst, port);
    }
    raw.clear();
}

// TelemetryService transport lives here (where the queue/UDP/compressor statics are file-local);
// the header stays free of AsyncUDP/compress includes. Producers stay decoupled: they only call
// the free telemetryAddPoint() above.

// Drains/compresses/sends the point queue on its own core-0 task, so compression (tens of ms with
// tamp) never stalls production. SPSC: producer = onTick thread, consumer = this task.
void TelemetryService::flushTask(void *arg) {
    auto *self = static_cast<TelemetryService *>(arg);
    for (;;) {
        self->flushQueue(mppt.tele.influxdbHost);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Sends AT MOST ONE datagram per call, and ONLY when the batch is full (~MSS). flushTask calls
// this at a fixed cadence, so full datagrams leave one-at-a-time rather than bursting — back-to-
// back UDP sends drop more, especially across NAT. A partially-filled batch waits for more points,
// unless points are sparse (teleSparse): then it leaves at once.
void TelemetryService::flushQueue(const IPAddress &addr) {
    if (noSsid) return;
    constexpr auto port = 8086;
    if (g_teleBinary && s_teleCompressor) {
        Compressor &comp = *s_teleCompressor;
        const size_t cap = comp.maxBatchRaw(CONFIG_TCP_MSS);
        static std::string batch;
        std::string frame;
        try {
            if (batch.capacity() < cap) batch.reserve(cap); // once: growing by doubling held up to 2x cap
            while (pointsQ.try_dequeue(frame)) {
                if (!batch.empty() && batch.size() + frame.size() > cap) {  // batch full -> send, carry frame over
                    sendBinaryBatch(addr, port, comp, batch);                // one full datagram per call
                    batch += frame;                                         // keeps the reserved buffer
                    return;
                }
                batch += frame;
            }
            if (teleSparse()) sendBinaryBatch(addr, port, comp, batch);
        } catch (const std::bad_alloc &) {
            // Heap exhausted (fragmentation during a WiFi flap). Telemetry is non-critical: drop the
            // batch and free its buffer rather than let the throw escape the task and panic the device.
            std::string().swap(batch);
            ESP_LOGW("tele", "flushQueue OOM, dropped batch");
        }
    } else {
        constexpr size_t MTU = CONFIG_TCP_MSS;
        static String msg;
        std::string lp;
        while (pointsQ.try_dequeue(lp)) {
            if (msg.length() && msg.length() + lp.length() + 1 > MTU) {  // full -> send, carry line over
                udpFlushString(addr, port, msg);
                msg = String(lp.c_str()); msg += '\n';
                return;
            }
            msg += lp.c_str(); msg += '\n';
        }
        if (msg.length() && teleSparse()) udpFlushString(addr, port, msg);
    }
}


extern VIinVout<const Sensor *> sensors;

void dcdcDataChanged(const ADC_Sampler &dcdc, const Sensor &sensor) {
    if (timeSynced && sensor.params.rawTelemetry && !sensor.params.teleName.empty() && WiFi.isConnected()) {
        auto point = makeTelePoint("mppt");
        point.addTag("device", getHostname().c_str());
        point.addField(sensor.params.teleName.c_str(), sensor.last, 3);
        point.setTimeMs();
        telemetryAddPoint(point, 600);
    }
}


