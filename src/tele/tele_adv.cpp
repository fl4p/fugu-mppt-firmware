#include "tele_adv.h"

#ifdef WITH_BLE_ADV

#include <BLEDevice.h>
#include <host/ble_gap.h>
#include <host/ble_hs_id.h>
#include <esp_log.h>

#include "console_ble.h"
#include "improv_ble.h"
#include "../conf.h"
#include "../mppt.h"
#include "../app_state.h"
#include "../math/float16.h"

extern MpptController mppt;

static const char *TAG = "teleadv";

static uint32_t intervalMs = 0;      // tele.conf adv_ms, 0 = off

// ble_gap_adv_active() can't tell a connectable adv from our non-connectable one, so the mode we
// last established is tracked here. Unknown means unverified (boot, svc restart) and always
// re-asserts — assuming "connectable" while a non-conn adv runs strands the device unreachable.
enum class AdvMode : uint8_t { Unknown, Connectable, NonConn };
static AdvMode advMode = AdvMode::Unknown;
static uint8_t seq = 0;
static time_ms lastRefreshMs = 0;
static time_ms lastWarnMs = 0;

struct __attribute__((packed)) Rec {
    uint8_t magic, seq;
    uint16_t Ui, Uo, I, P;           // float16 bits
    int8_t mcuTemp, ntcTemp;         // °C
    uint16_t duty, lagUs;
    uint8_t state;                   // mppt_state | cv_lim_idx<<4
};
static_assert(sizeof(Rec) == 17);

static constexpr uint8_t MAGIC = 0xF7;

void teleAdvInit() {
    intervalMs = ConfFile{"/littlefs/conf/tele.conf"}.getLong("adv_ms", 500);
    if (intervalMs && intervalMs < 100) intervalMs = 100;
    // bleConsoleBegin just (re)started the connectable adv on every path that calls this, so
    // Connectable is verified state — Unknown here would make the first tick stop+restart a
    // correct adv, racing a CONNECT_IND already in flight. If the wrapper's start failed, the
    // reconciler's !active branch resumes it.
    advMode = AdvMode::Connectable;
    if (intervalMs) ESP_LOGI(TAG, "broadcast every %lu ms", (unsigned long) intervalMs);
}

bool teleAdvEnabled() { return intervalMs != 0; }

bool teleAdvOwnsAdv() { return advMode == AdvMode::NonConn; }

static void warnThrottled(const char *what, int rc) {
    time_ms now = millis();
    if (now - lastWarnMs < 10000) return;
    lastWarnMs = now;
    ESP_LOGW(TAG, "%s rc=%d", what, rc);
}

static int nonConnGapEvent(ble_gap_event *, void *) { return 0; }

// The broadcast must keep the MAC of the connectable adv: observers cache the name per address, and
// a second identity also hides the console from anything scanning by address. BLEDevice picks its
// own address type at sync and exposes only the resulting address, so match on the address itself —
// re-deriving its rule would inherit its quirks. (BLE_ADDR_*/BLE_OWN_ADDR_* share values here.)
static uint8_t consoleOwnAddrType() {
    uint8_t pub[6];
    return ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, pub, nullptr) == 0 &&
           memcmp(pub, BLEDevice::getAddress().getNative(), 6) == 0
           ? BLE_OWN_ADDR_PUBLIC : BLE_OWN_ADDR_RANDOM;
}

static bool startNonConn() {
    ble_gap_adv_params p{};
    p.conn_mode = BLE_GAP_CONN_MODE_NON;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = 160;   // 100 ms (0.625 ms units)
    p.itvl_max = 400;   // 250 ms
    int rc = ble_gap_adv_start(consoleOwnAddrType(), nullptr, BLE_HS_FOREVER, &p,
                               nonConnGapEvent, nullptr);
    if (rc != 0) { warnThrottled("nonconn adv start", rc); return false; }
    ESP_LOGI(TAG, "nonconn adv as %s", BLEDevice::getAddress().toString().c_str());
    return true;
}

// Every 8th slot broadcasts flags + complete name instead of the record: scan responses
// don't reach every observer (rpi kernel scans effectively passively next to an active LE
// connection), so the name must ride the adv payload itself. Observers cache it per MAC.
static bool namePayload(uint8_t slot) {
    if (slot % 8 != 0) return false;
    const char *name = bleConsoleName();
    size_t n = std::min(strlen(name), (size_t) 26);
    if (!n) return false;
    uint8_t ad[31];
    ad[0] = 2; ad[1] = 0x01; ad[2] = 0x06;
    ad[3] = (uint8_t) (n + 1);
    ad[4] = strlen(name) > 26 ? 0x08 : 0x09;   // shortened / complete local name
    memcpy(&ad[5], name, n);
    int rc = ble_gap_adv_set_data(ad, 5 + n);
    if (rc != 0) warnThrottled("set name data", rc);
    return true;
}

static void refreshPayload() {
    static uint8_t slot = 0;
    ++slot;
    // Improv needs its UUID + service data in one advert (31 B): every odd slot while offered
    if (advMode == AdvMode::Connectable && (slot & 1)) {
        uint8_t ad[31];
        if (size_t n = improvAdvPayload(ad)) {
            int rc = ble_gap_adv_set_data(ad, n);
            if (rc != 0) warnThrottled("set improv data", rc);
            return;
        }
    }
    if (namePayload(slot)) return;
    // Sensor setup can fail (setupErr) with the BLE service still up — the console must
    // survive to debug exactly that (same guard as MQTT.tickHook).
    if (!mppt.sensorPhysicalI || !mppt.sensorPhysicalU) return;
    auto s = mppt.teleSnap();
    // med3 seeds NaN until the first sample; every other telemetry path isnan-guards.
    if (std::isnan(s.Ui) || std::isnan(s.Uo) || std::isnan(s.I)) return;
    // NaN temp (e.g. no NTC fitted) -> -128 sentinel, decoder omits the field.
    auto i8 = [](float f) { return std::isnan(f) ? (int8_t) -128
                                                 : (int8_t) std::max(-127.f, std::min(127.f, f)); };
    Rec r{.magic = MAGIC, .seq = seq++,
          .Ui = float16(s.Ui).getBinary(), .Uo = float16(s.Uo).getBinary(),
          .I = float16(s.I).getBinary(), .P = float16(s.P).getBinary(),
          .mcuTemp = i8(s.mcuTemp), .ntcTemp = i8(s.ntcTemp),
          .duty = s.duty,
          .lagUs = (uint16_t) std::min<uint32_t>(g_app.maxLoopLag, 65535),
          .state = (uint8_t) ((s.mode & 0xF) | (s.limIdx << 4))};
    uint8_t ad[3 + 4 + sizeof(Rec)];
    ad[0] = 2; ad[1] = 0x01; ad[2] = 0x06;              // flags: LE general disc, no BR/EDR
    ad[3] = 3 + sizeof(Rec); ad[4] = 0xFF;              // mfr data AD
    ad[5] = 0xFF; ad[6] = 0xFF;                         // company id 0xFFFF (internal)
    memcpy(&ad[7], &r, sizeof(Rec));
    int rc = ble_gap_adv_set_data(ad, sizeof(ad));      // allowed while advertising
    if (rc != 0) warnThrottled("set adv data", rc);
}

void teleAdvTick(time_ms nowMs) {
    if (!intervalMs) return;
    bool active = ble_gap_adv_active();
    // Connected: the controller stopped the connectable adv, and the broadcast must not pause.
    AdvMode want = bleConsoleConnected() ? AdvMode::NonConn : AdvMode::Connectable;

    if (advMode != want || !active) {
        // First attempt of a transition runs immediately; only RETRIES after a failed start
        // back off (a tick-rate ble_gap_adv_start loop hammers the HCI queue under pressure).
        static time_ms lastTryMs = 0;
        if (advMode != AdvMode::Unknown || nowMs - lastTryMs >= 1000) {
            lastTryMs = nowMs;
            if (active) ble_gap_adv_stop();
            advMode = AdvMode::Unknown;    // re-asserted below; a failed start retries with backoff
            if (want == AdvMode::NonConn) {
                if (startNonConn()) advMode = AdvMode::NonConn;
            } else {
                bleConsoleResumeAdv();
                advMode = AdvMode::Connectable;
            }
        }
    }

    if (nowMs - lastRefreshMs >= intervalMs) {
        lastRefreshMs = nowMs;
        refreshPayload();
    }
}

#endif // WITH_BLE_ADV
