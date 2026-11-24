#include "improv_ble.h"

#ifdef WITH_IMPROV

#include <algorithm>
#include <mutex>
#include <string>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <esp_app_desc.h>
#include <host/ble_gap.h>

#include "telemetry.h"
#include "tele_adv.h"
#include "console_ble.h"
#include <ota_ble.h>
#include "../app_state.h"
#include "../conf.h"

extern bool noSsid;

static const char *TAG = "improv";

#define IMPROV_UUID(n) "00467768-6228-2272-4663-27747826800" #n

enum : uint8_t { ST_AUTH_REQUIRED = 1, ST_AUTHORIZED, ST_PROVISIONING, ST_PROVISIONED };
enum : uint8_t { E_NONE = 0, E_INVALID_RPC, E_UNKNOWN_RPC, E_UNABLE_TO_CONNECT, E_NOT_AUTHORIZED,
                 E_UNKNOWN = 0xFF };
enum : uint8_t { CMD_WIFI = 1, CMD_DEVICE_INFO = 3, CMD_SCAN = 4 };
static constexpr uint8_t CAPS = 0x06; // device info + scan, no identify
static constexpr time_ms CONNECT_TIMEOUT_MS = 25000, PROVISIONED_ADV_MS = 60000, RPC_STALE_MS = 2000,
                         ADV_SWAP_MS = 500, WINDOW_OFFLINE_MS = 20000;

static BLECharacteristic *stateChr, *errChr, *resChr;
static uint8_t state = 0, err = E_NONE;
static bool accepting = false; // credentials (latched until online)
static uint32_t offlineOpenMs = 0;   // wifi.conf improv_offline_s, 0 = only with no network configured
static time_ms onlineMs = 0, attemptMs = 0, provisionedMs = 0;
static std::string attemptSsid;

// RPC assembly: written by the NimBLE host task, consumed by the network loop.
static std::mutex rxMutex;
static uint8_t rxBuf[260];
static uint16_t rxLen = 0;
static bool rxOverflow = false;
static uint32_t rxMs = 0; // millis()

static void notify(BLECharacteristic *c, const uint8_t *d, size_t n) {
    c->setValue((uint8_t *) d, n);
    // Same floor as the console drain: an MTU notify into an empty mbuf pool asserts in NimBLE.
    for (int i = 0; i < 50 && bleMsys1Free() < 8; ++i) vTaskDelay(pdMS_TO_TICKS(2));
    c->notify();
}

static void setErr(uint8_t e) {
    err = e;
    notify(errChr, &err, 1); // notified on every RPC, also unchanged: the client waits for it
}

static void setState(uint8_t s) {
    if (s == state) return;
    state = s;
    notify(stateChr, &state, 1);
    ESP_LOGI(TAG, "state %u", s);
}

static void sendResult(uint8_t cmd, const std::string &body) {
    uint8_t p[258];
    size_t n = std::min(body.size(), (size_t) 255);
    p[0] = cmd;
    p[1] = (uint8_t) n;
    memcpy(p + 2, body.data(), n);
    uint8_t cs = 0;
    for (size_t i = 0; i < n + 2; ++i) cs += p[i];
    p[n + 2] = cs;
    notify(resChr, p, n + 3);
}

static bool addStr(std::string &body, const char *s) {
    size_t n = strlen(s);
    if (n > 255 || body.size() + 1 + n > 255) return false;
    body += (char) n;
    body.append(s, n);
    return true;
}

static const char *authName(wifi_auth_mode_t a) {
    switch (a) {
        case WIFI_AUTH_OPEN: return "NO";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2 EAP";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK: return "WAPI";
        default: return "WPA2";
    }
}

static void scanResult() {
    disconnect_wifi(false); // offline here; a reconnecting station refuses scans
    const int n = WiFi.scanNetworks(false, false, false, 0);
    std::string body;
    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        bool dup = ssid.isEmpty();
        for (int j = 0; j < i && !dup; ++j) dup = WiFi.SSID(j) == ssid;
        if (dup) continue;
        char rssi[8];
        snprintf(rssi, sizeof(rssi), "%d", (int) WiFi.RSSI(i));
        const char *auth = authName(WiFi.encryptionType(i));
        if (body.size() + 3 + ssid.length() + strlen(rssi) + strlen(auth) > 255) break;
        addStr(body, ssid.c_str());
        addStr(body, rssi);
        addStr(body, auth);
    }
    WiFi.scanDelete();
    sendResult(CMD_SCAN, body);
}

static void wifiSettings(const uint8_t *d, uint8_t len) {
    if (state == ST_PROVISIONING) return setErr(E_UNKNOWN);
    if (state != ST_AUTHORIZED) return setErr(E_NOT_AUTHORIZED);
    uint8_t sl = len ? d[0] : 0;
    if (!sl || sl > 32 || 1 + sl >= len || 2 + sl + d[1 + sl] != len || d[1 + sl] > 64)
        return setErr(E_INVALID_RPC);
    std::string ssid((const char *) d + 1, sl), psk((const char *) d + 2 + sl, d[1 + sl]);
    ESP_LOGI(TAG, "credentials for '%s'", ssid.c_str());
    if (!add_ap(ssid, psk)) return setErr(E_UNKNOWN);
    attemptSsid = ssid;
    attemptMs = wallClockMs();
    wifi_retry_now();
    connect_wifi_async();
    // the station otherwise keeps auto-reconnecting with its old config (scans and begin fail meanwhile)
    disconnect_wifi(false);
    WiFi.begin(ssid.c_str(), psk.empty() ? nullptr : psk.c_str());
    setState(ST_PROVISIONING);
}

static void handleRpc(const uint8_t *p, size_t n) {
    uint8_t cs = 0;
    for (size_t i = 0; i + 1 < n; ++i) cs += p[i];
    if (cs != p[n - 1]) return setErr(E_INVALID_RPC);
    setErr(E_NONE);
    switch (p[0]) {
        case CMD_WIFI:
            if (otaBleActive()) return setErr(E_UNKNOWN); // Wi-Fi stays off until the push's reboot
            return wifiSettings(p + 2, p[1]);
        case CMD_DEVICE_INFO: {
            const esp_app_desc_t *app = esp_app_get_description();
            std::string body;
            addStr(body, "Fugu MPPT");
            addStr(body, app->version);
            addStr(body, CONFIG_IDF_TARGET);
            addStr(body, getHostname().c_str());
            return sendResult(CMD_DEVICE_INFO, body);
        }
        case CMD_SCAN:
            if (state != ST_AUTHORIZED) return setErr(E_NOT_AUTHORIZED);
            if (otaBleActive()) return setErr(E_UNKNOWN);
            return scanResult();
        default:
            return setErr(E_UNKNOWN_RPC);
    }
}

class RpcCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String v = c->getValue();
        std::lock_guard<std::mutex> lk(rxMutex);
        uint32_t now = millis();
        if (rxLen && now - rxMs > RPC_STALE_MS) rxLen = 0;
        rxMs = now;
        if (rxLen + v.length() > sizeof(rxBuf)) {
            rxOverflow = true;
            rxLen = 0;
            return;
        }
        memcpy(rxBuf + rxLen, v.c_str(), v.length());
        rxLen += v.length();
    }
};

static RpcCallbacks rpcCallbacks;

void improvCreateService(BLEServer *srv, uint32_t writeProps) {
    const long s = ConfFile{"/littlefs/conf/wifi.conf", true}.getLong("improv_offline_s", 600);
    if (s < 0) return; // Improv off: no service, stateChr stays null
    offlineOpenMs = (uint32_t) s * 1000;
    BLEService *svc = srv->createService(IMPROV_UUID(0));
    const uint32_t rn = BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY;
    stateChr = svc->createCharacteristic(IMPROV_UUID(1), rn);
    errChr = svc->createCharacteristic(IMPROV_UUID(2), rn);
    BLECharacteristic *rpc = svc->createCharacteristic(
        IMPROV_UUID(3), writeProps & ~(uint32_t) BLECharacteristic::PROPERTY_WRITE_NR);
    rpc->setCallbacks(&rpcCallbacks);
    resChr = svc->createCharacteristic(IMPROV_UUID(4), rn);
    static const uint8_t caps = CAPS;
    svc->createCharacteristic(IMPROV_UUID(5), BLECharacteristic::PROPERTY_READ)->setValue((uint8_t *) &caps, 1);
    errChr->setValue(&err, 1);
    svc->start();
}

static bool advertised(time_ms now) {
    return accepting || (state == ST_PROVISIONED && provisionedMs && now - provisionedMs < PROVISIONED_ADV_MS);
}

size_t improvAdvPayload(uint8_t *ad) {
    if (!stateChr || !advertised(wallClockMs())) return 0;
    static const uint8_t hdr[] = {
        2, 0x01, 0x06,
        17, 0x07, 0x00, 0x80, 0x26, 0x78, 0x74, 0x27, 0x63, 0x46, 0x72, 0x22, 0x28, 0x62, 0x68, 0x77, 0x46, 0x00,
        9, 0x16, 0x77, 0x46};
    memcpy(ad, hdr, sizeof(hdr));
    uint8_t *sd = ad + sizeof(hdr);
    sd[0] = state;
    sd[1] = CAPS;
    sd[2] = sd[3] = sd[4] = sd[5] = 0;
    return sizeof(hdr) + 6;
}

// Without the telemetry broadcast nothing else rewrites the connectable advert: alternate the
// Improv advert (UUID + service data, 31 B) with the console's flags + name (the NUS UUID stays in the
// scan response). The console advert carries the service data too when it fits: a host that merges
// adverts (and lists GATT-cached UUIDs for a bonded device) would otherwise see the Improv UUID without it.
static void advTick(time_ms now) {
    static time_ms lastMs = 0;
    static bool onAir = false, improvPhase = false;
    if (teleAdvEnabled() || now - lastMs < ADV_SWAP_MS) return;
    lastMs = now;
    uint8_t imp[31], ad[31];
    size_t n, ni = improvAdvPayload(imp);
    if (!ni && !onAir) return;
    if (ni) improvPhase = !improvPhase;
    if (ni && improvPhase) memcpy(ad, imp, n = ni);
    else if ((n = bleNameAdv(ad)) + 10 <= sizeof(ad) && ni) memcpy(ad + n, imp + 21, 10), n += 10;
    if (ble_gap_adv_set_data(ad, n) == 0) onAir = ni;
}

void improvTick(time_ms now) {
    if (!stateChr) return;
    const bool up = WiFi.isConnected();
    if (up || !onlineMs) onlineMs = now;
    // The BLE pairing window (boot, `ble pair`) is Improv's authorization; it also offers Improv
    // after a short outage, so a power cycle re-provisions without waiting out improv_offline_s.
    const bool window = blePairRemainingS() > 0;
    const time_ms openAfter = window ? std::min<time_ms>(offlineOpenMs, WINDOW_OFFLINE_MS) : offlineOpenMs;
    if (up || g_app.disableWifi) accepting = false;
    else if (!accepting && (noSsid || (offlineOpenMs && now - onlineMs >= openAfter))) {
        accepting = true;
        ESP_LOGI(TAG, "offline, accepting Wi-Fi credentials");
    }
    const uint8_t idle = up ? ST_PROVISIONED : accepting && (window || !blePairGated()) ? ST_AUTHORIZED : ST_AUTH_REQUIRED;

    if (state == ST_PROVISIONING) {
        if (up && attemptSsid == WiFi.SSID().c_str()) {
            ESP_LOGI(TAG, "provisioned, connected to '%s'", attemptSsid.c_str());
            sendResult(CMD_WIFI, {}); // result before state: clients finish on the result
            provisionedMs = now;
            setState(ST_PROVISIONED);
        } else if (now - attemptMs > CONNECT_TIMEOUT_MS) {
            ESP_LOGW(TAG, "could not connect to '%s'", attemptSsid.c_str());
            if (!up) disconnect_wifi(false); // stop the retry loop, it starves the BLE link
            setState(idle);
            setErr(E_UNABLE_TO_CONNECT);
        }
    } else {
        setState(idle);
    }

    uint8_t pkt[sizeof(rxBuf)];
    size_t n = 0;
    bool bad = false;
    {
        std::lock_guard<std::mutex> lk(rxMutex);
        if (rxOverflow) {
            rxOverflow = false;
            bad = true;
        } else if (rxLen >= 3 && rxLen >= rxBuf[1] + 3u) {
            bad = rxLen != rxBuf[1] + 3u; // bytes past the declared length
            n = rxLen;
            memcpy(pkt, rxBuf, n);
            rxLen = 0;
        }
    }
    if (bad) setErr(E_INVALID_RPC);
    else if (n) handleRpc(pkt, n);

    advTick(now);
}

#endif // WITH_IMPROV
