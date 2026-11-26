#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#include "console_ble.h"

#ifdef CONFIG_FUGU_WITH_BLE

#include <algorithm>
#include <mutex>

#include <Arduino.h> // String
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#if defined(CONFIG_BT_NIMBLE_ENABLED)
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_store.h>
#endif
#include <BLESecurity.h>
#include <os/os_mbuf.h> // os_msys_num_free(): peek NimBLE's free mbuf pool before notifying
#if !defined(CONFIG_BLUEDROID_ENABLED)
#include <host/ble_gap.h> // ble_gap_conn_find(): connection state from the stack, not our callbacks
#endif

#include "console.h"   // loopConsole
#include "app_state.h"
#include "etc/ota.h"     // otaPendingVerify
#include "util.h"      // wallClockMs
#include "logging.h"   // addLogCallback / removeLogCallback, ESP_LOG*
#include "etc/readerwriterqueue.h"
#include <ota_ble.h>     // OTA-over-BLE firmware push (esp-ota-ble component; FW char data sink)
#include "adc/sampling.h" // ADC_Sampler -- halted while flash is busy
#include "tele_ble.h"    // BLE telemetry stream (TELE characteristic, CONFIG_FUGU_WITH_BLE_TELE)
#include "tele_adv.h"    // advertising telemetry broadcast (CONFIG_FUGU_WITH_BLE_ADV)
#include "improv_ble.h"

// esp-ota-ble host hooks. The status sink goes to ESP_LOG* at the level the module asked for, with
// the tag still "otab" -- the wire protocol IS these log lines (they are mirrored to the client), so
// the format and the severity both have to survive. Flattening FAIL lines to INFO would mean raising
// this tag's level silently swallows exactly what the host needs to tell failure from a dead link.
extern ADC_Sampler adcSampler;
void stopAndBackoff(uint32_t secondsDelay, const char *who = "stopAndBackoff");
void systemRestart();
void wifiRelease();

static volatile bool otaRebootDue = false;

static void otaStatusHook(OtaBleLevel level, const char *line) {
    switch (level) {
        case OtaBleLevel::Info:  ESP_LOGI("otab", "%s", line); break;
        case OtaBleLevel::Warn:  ESP_LOGW("otab", "%s", line); break;
        case OtaBleLevel::Error: ESP_LOGE("otab", "%s", line); break;
    }
}

static void otaQuiesceHook(bool halt) {
    // Flash erase/write disables the CPU cache and stalls the other core; at full power the RT
    // loop-latency watchdog has no margin and resets the device mid-download. The converter goes
    // down first, then the sampler.
    if (halt) stopAndBackoff(10);
    adcSampler.halted = halt;
#ifdef CONFIG_FUGU_WITH_NETW
    // The push keeps Wi-Fi off until the reboot that ends it; a failed push reboots too. Not without
    // a reboot to bring it back (cmdOtaBle refuses those cases), and Wi-Fi the user turned off stays
    // as it is, with no reboot on failure.
    static bool wifiReleased = false;
    if (halt && !wifiReleased && !g_app.disableWifi && !g_restartBlocked && !otaPendingVerify()) {
        const unsigned h = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        wifiReleased = true;
        g_app.wifiReenableMs = 0;
        wifiRelease();
        ESP_LOGI("ota", "WiFi released, free heap %u -> %u", h, (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    } else if (!halt && wifiReleased) {
        wifiReleased = false;
        otaRebootDue = true;
    }
#endif
}

bool bleOtaRebootDue() {
    if (!otaRebootDue) return false;
    otaRebootDue = false;
    return true;
}

static void otaBleInstallHooks() {
    OtaBleHooks h;
    h.status = &otaStatusHook;
    h.quiesce = &otaQuiesceHook;
    h.restart = &systemRestart;
    otaBleInit(h);
}

// Nordic UART Service. RX = client->device (write commands), TX = device->client (notify output).
// FW = client->device write-no-response firmware bytes (OTA push, bypasses the console line parser).
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_FW_CHAR_UUID "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

static const char *TAG = "ble";

static BLEServer *bleServer = nullptr;
static BLECharacteristic *txChar = nullptr;
static volatile bool deviceConnected = false;
// The Arduino-ESP32 BLE wrapper is not reinit-safe: BLEDevice::deinit() frees the controller but
// leaves the static BLEServer and its service map intact, so a second init+createService() hands
// back a stale BLEService over torn-down NimBLE handles and crashes. So we init the whole stack
// exactly once and, on stop/start, only toggle advertising. `bleStarted` tracks the advertising
// (logical service running) state; `bleInited` tracks the one-time stack bring-up.
static bool bleInited = false;
static bool bleStarted = false;
static bool bleGone = false; // bleConsoleRelease() freed the stack; no init until reboot
static uint16_t advMs = 0; // connectable adv interval, 0 = stack default

// Single-producer (NimBLE host task, onWrite) / single-consumer (network loop, bleRead) byte queue.
static moodycamel::ReaderWriterQueue<char> rxQueue{256};

// NimBLE keeps its msys pools sorted by block size; notify/ATT/L2CAP allocate from the first (msys_1),
// so os_msys_num_free(), which sums all pools, overstates the headroom. No public per-pool API.
struct MsysPoolList { os_mbuf_pool *first; os_mbuf_pool **last; };
extern "C" MsysPoolList g_msys_pool_list;

int bleMsys1Free() {
    const os_mbuf_pool *p = g_msys_pool_list.first;
    return p ? p->omp_pool->mp_num_free : os_msys_num_free();
}

// --- console read/write hooks fed to loopConsole() ---

static int bleRead(char *buf, size_t len) {
    size_t n = 0;
    char c;
    while (n < len && rxQueue.try_dequeue(c)) buf[n++] = c;
    return (int) n;
}

// Serializes access to the TX FIFO and notify() across tasks: console echo/responses run on the
// network loop while the log mirror (bleLogWrite) can fire from other core-0 tasks. Recursive so the
// rc=6 error log emitted inside notify() on the same task can't self-deadlock (it re-enters bleWrite).
static std::recursive_mutex txMutex;

// NimBLE can only emit roughly one notification per connection interval and has a small mbuf pool, so
// notifying a multi-line command response (e.g. `get-config`) in a tight loop exhausts it: then
// ble_gatts_notify_custom returns BLE_HS_ENOMEM (rc=6) and the packet is silently dropped, truncating
// the client's output. So bleWrite only appends to this FIFO; bleTxDrain() (pumped from the network
// loop) sends 20-byte chunks with backpressure — it stops the moment a notify fails and leaves the
// rest queued for the next tick, by when the pool has refilled.
static std::string txBuf;
static std::string advName;
static size_t txHead = 0;                    // consumed-prefix offset, amortizes pop-front
static constexpr size_t TX_BUF_CAP = 8192;   // bound the unsent backlog if a client stalls
static bool lastNotifyOk = true;             // set by TxCallbacks::onStatus, read right after notify()

// Right after connect the central is still on its default (slow) connection interval and the param
// update we request hasn't applied yet, so pushing the queued log backlog into the link overflows the
// controller's ACL buffers (the rc=6 burst). Hold the drain for a short settle window; the backlog
// then flushes cleanly. Re-armed on every connect.
// Command results (pwm-freq, sync, dt, status, ...) are emitted with UART_LOG, so they travel the
// LOG path and are subject to the half-FIFO cap above -- a client can lose a reply with no error
// and no gap, and cannot tell a command that failed from one whose answer was discarded. Until
// results get their own channel, at least make the loss visible: latch a drop and emit one marker
// once the backlog clears, through bleWrite (full FIFO) so the marker itself cannot be dropped.
static bool logDropped = false;
static volatile bool txArmSettle = false;
static time_ms txConnectMs = 0;

static constexpr time_ms TX_SETTLE_MS = 500;

// Post-connect link setup (2M PHY, then conn interval), one LL procedure at a time. The S3 controller
// asserts in lld_con.c:3275 (r_lld_con_param_update, INT_WDT) when the central's own connection
// update crosses a procedure we opened (fry 2026-11-20, weak link; same assert and fix in
// walkthru-earth/opensensor-space-firmware 27febc4). Starts LINK_SETTLE_MS after encryption (or
// LINK_ENC_WAIT_MS after connect if the link stays plain), waits for the PHY update to complete,
// and asks for an interval only if the central has not already chosen one in range.
#if defined(CONFIG_BT_NIMBLE_ENABLED)
enum : uint8_t { LINK_IDLE, LINK_PHY_WAIT, LINK_CONN, LINK_DONE };
static volatile uint8_t linkStep = LINK_DONE;
static volatile bool linkEnc = false, linkPhyDone = false;
static volatile uint32_t linkT = 0; // millis(): connect / encryption / last step
static bool linkEncRequired = false;
static constexpr uint32_t LINK_SETTLE_MS = 2000, LINK_ENC_WAIT_MS = 6000, LINK_PHY_WAIT_MS = 2000,
                          LINK_RETRY_MS = 1000;
static constexpr uint16_t ITVL_MIN = 6, ITVL_MAX = 12; // 7.5..15 ms

// Pairing window. Closed, sm_sec_lvl 1 makes the SM answer every Pairing Request with "command not
// supported" (ble_sm_pair_req_rx), before key exchange and before the repeat-pairing check; level 1
// has no other effect (ATT checks start at 2). Bonded peers re-encrypt with their LTK and never send one.
static volatile bool pairOpen = false, connEnc = false, bondEvicted = false;
static bool rejLogged = false;
static uint32_t pairEndMs = 0, pairDefaultS = 300; // millis()
static int connBonds = 0;
static const char *secName = "none";

static const char *fmtAddr(const ble_addr_t &a, char *s) {
    snprintf(s, 18, "%02x:%02x:%02x:%02x:%02x:%02x", a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0]);
    return s;
}

static int bondCount() {
    int n = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &n);
    return n;
}

static void pairSet(uint32_t s) {
    s = std::min(s, (uint32_t) 86400);
    pairEndMs = millis() + s * 1000;
    pairOpen = s > 0;
    if (linkEncRequired) ble_hs_cfg.sm_sec_lvl = pairOpen ? MYNEWT_VAL(BLE_SM_LVL) : 1;
}

static void logUnbonded(const ble_addr_t &a) {
    if (rejLogged || connEnc || !linkEncRequired || pairOpen) return;
    rejLogged = true;
    char s[18];
    ESP_LOGW(TAG, "%s: no bond, pairing window closed (`ble pair`)", fmtAddr(a, s));
}

// Backstop: a bond added while the window is closed is dropped with its link.
static void encAudit(uint16_t ch) {
    ble_gap_conn_desc d;
    if (ble_gap_conn_find(ch, &d) != 0) return;
    const bool fresh = bondEvicted || bondCount() > connBonds;
    bondEvicted = false;
    char s[18];
    fmtAddr(d.peer_id_addr, s);
    if (fresh && linkEncRequired && !pairOpen) {
        ESP_LOGE(TAG, "%s bonded outside the pairing window, dropped", s);
        ble_gap_unpair(&d.peer_id_addr);
        return;
    }
    ESP_LOGI(TAG, "%s %s", s, fresh ? "paired, new bond" : d.sec_state.bonded ? "bonded, encrypted" : "encrypted");
}

struct CccdPick {
    ble_addr_t self;     // peer being written
    ble_store_key_cccd drop;
    bool found;
};

static int cccdPick(int, ble_store_value *v, void *arg) {
    auto *p = (CccdPick *) arg;
    const bool other = ble_addr_cmp(&v->cccd.peer_addr, &p->self) != 0;
    if (!p->found || other) ble_store_key_from_value_cccd(&p->drop, &v->cccd);
    p->found = true;
    return other;
}

// Window closed: refuse a new bond with a full table. Open: NimBLE evicts the oldest on overflow.
// A full CCCD table drops a stored subscription (another peer's first) instead of a whole bond.
class StoreCallbacks : public BLEDeviceCallbacks {
    int onStoreStatus(ble_store_status_event *e, void *arg) override {
        if (e->event_code == BLE_STORE_EVENT_FULL && linkEncRequired && !pairOpen) {
            ESP_LOGW(TAG, "pairing refused: window closed, bond table full");
            return BLE_HS_ESTORE_CAP;
        }
        if (e->event_code == BLE_STORE_EVENT_OVERFLOW && e->overflow.obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
            CccdPick p{e->overflow.value->cccd.peer_addr, {}, false};
            ble_store_iterate(BLE_STORE_OBJ_TYPE_CCCD, cccdPick, &p);
            return p.found ? ble_store_delete_cccd(&p.drop) : BLE_HS_ESTORE_CAP;
        }
        if (e->event_code == BLE_STORE_EVENT_OVERFLOW && (e->overflow.obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC
                                                          || e->overflow.obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC)) {
            ble_addr_t a[CONFIG_BT_NIMBLE_MAX_BONDS];
            int n = 0;
            char s[18];
            if (ble_store_util_bonded_peers(a, &n, CONFIG_BT_NIMBLE_MAX_BONDS) == 0 && n)
                ESP_LOGW(TAG, "bond table full, evicting oldest %s", fmtAddr(a[0], s));
            bondEvicted = true;
        }
        return ble_store_util_status_rr(e, arg);
    }
};

static StoreCallbacks storeCallbacks;

static int linkGapEvent(ble_gap_event *e, void *) {
    if (e->type == BLE_GAP_EVENT_ENC_CHANGE && e->enc_change.status == 0) {
        connEnc = true;
        encAudit(e->enc_change.conn_handle);
        if (linkStep == LINK_IDLE) {
            linkEnc = true;
            linkT = millis();
        }
    } else if (e->type == BLE_GAP_EVENT_DISCONNECT) {
        logUnbonded(e->disconnect.conn.peer_id_addr);
    } else if (e->type == BLE_GAP_EVENT_PHY_UPDATE_COMPLETE) {
        linkPhyDone = true;
    } else if (e->type == BLE_GAP_EVENT_CONN_UPDATE && linkStep == LINK_CONN) {
        linkT = millis(); // central just updated: keep clear of its next procedure
    }
    return 0;
}

static void linkTick() {
    const uint32_t now = millis(), dt = now - linkT;
    const uint16_t ch = bleServer->getConnId();
    switch (linkStep) {
        case LINK_IDLE:
            if (dt < (linkEnc || !linkEncRequired ? LINK_SETTLE_MS : LINK_ENC_WAIT_MS)) return;
            if (!linkEnc) {
                ble_gap_conn_desc d;
                if (ble_gap_conn_find(ch, &d) == 0) logUnbonded(d.peer_id_addr);
            }
            linkStep = LINK_CONN;
            linkT = now - LINK_RETRY_MS;
#if defined(CONFIG_BT_NIMBLE_LL_CFG_FEAT_LE_2M_PHY)
            // Measured: 2M does not speed up an OTA push (flu 2026-09-08, ~37 s either way); kept for
            // console/telemetry latency. A REQUEST, not a default: set-default never starts a procedure.
            linkPhyDone = false;
            if (ble_gap_set_prefered_le_phy(ch, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0) == 0) {
                linkStep = LINK_PHY_WAIT;
                linkT = now;
            }
#endif
            return;
        case LINK_PHY_WAIT: {
            if (!linkPhyDone && dt < LINK_PHY_WAIT_MS) return;
            uint8_t tx = 0, rx = 0;
            ble_gap_read_le_phy(ch, &tx, &rx);
            ESP_LOGI(TAG, "phy: done=%d tx=%u rx=%u", (int) linkPhyDone, (unsigned) tx, (unsigned) rx);
            linkStep = LINK_CONN;
            linkT = now;
            return;
        }
        case LINK_CONN: {
            if (dt < LINK_RETRY_MS) return;
            ble_gap_conn_desc d;
            if (ble_gap_conn_find(ch, &d) != 0) return;
            if (d.conn_itvl >= ITVL_MIN && d.conn_itvl <= ITVL_MAX && d.conn_latency == 0) {
                ESP_LOGI(TAG, "conn: itvl=%u kept", (unsigned) d.conn_itvl);
                linkStep = LINK_DONE;
                return;
            }
            // supervision timeout 400 = 4 s; refused while a procedure runs -> retry
            if (bleServer->requestConnParams(ch, ITVL_MIN, ITVL_MAX, 0, 400)) {
                ESP_LOGI(TAG, "conn: itvl=%u, requested %u..%u", (unsigned) d.conn_itvl, ITVL_MIN, ITVL_MAX);
                linkStep = LINK_DONE;
            }
            linkT = now;
            return;
        }
        default:
            return;
    }
}
#endif

static int bleWrite(const char *buf, size_t len) {
    if (!deviceConnected || !txChar) return 0;
    std::lock_guard<std::recursive_mutex> lk(txMutex);
    if (txHead == txBuf.size()) { txBuf.clear(); txHead = 0; } // fully drained: reclaim
    // Compact the consumed prefix before growing: without this txBuf's *capacity* is unbounded
    // (backlog is capped but size = head + backlog isn't) and the doubling realloc bad_alloc'd
    // at 16 KB on a 27 KB-free no-PSRAM heap (fboost coredump 8-05, log backlog on connect).
    else if (txHead && txBuf.size() + len > TX_BUF_CAP) { txBuf.erase(0, txHead); txHead = 0; }
    size_t backlog = txBuf.size() - txHead;
    if (backlog + len > TX_BUF_CAP) len = backlog < TX_BUF_CAP ? TX_BUF_CAP - backlog : 0; // drop overflow
    try { txBuf.append(buf, len); } catch (const std::bad_alloc &) { return 0; } // OOM: drop, never
    return (int) len;                                    // throw into the log path (would terminate)
}

// Push queued output to the client, paced by NimBLE's buffer availability. Pump from the network loop.
static void bleTxDrain(time_ms nowMs) {
    if (!deviceConnected || !txChar || !bleServer) return;
    std::lock_guard<std::recursive_mutex> lk(txMutex);
    if (txArmSettle) { txConnectMs = nowMs; txArmSettle = false; }
    if (nowMs - txConnectMs < TX_SETTLE_MS) return; // let pairing settle before touching the link
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    linkTick();
    if (linkEncRequired && !connEnc) return; // no output to an unpaired peer
#endif

    // Chunk at the negotiated ATT MTU minus the 3-byte notify header (falls back to the 20-byte
    // default before MTU exchange). A larger MTU means a multi-line reply is a few notifications
    // instead of dozens of 20-byte ones — the dominant console-latency win alongside a fast interval.
    uint16_t mtu = bleServer->getPeerMTU(bleServer->getConnId());
    size_t chunk = mtu > 23 ? (size_t) (mtu - 3) : 20;
    while (txHead < txBuf.size()) {
        // Peek NimBLE's free mbuf count first. Right after connect the connection interval is still
        // slow and the pool drains faster than it refills; calling notify() into an empty pool returns
        // BLE_HS_ENOMEM (rc=6) and logs an [E] from the BLE wrapper. So skip the call entirely when the
        // pool is low and retry next tick — no failed call, no error spam, no dropped bytes.
        // Floor 8: an MTU-sized notify can consume ~4 blocks across ATT/L2CAP fragmentation, and
        // IDF's NimBLE hard-asserts (ble_att_cmd.c:91) if ble_l2cap_tx runs dry mid-send — an
        // exhausted pool there PANICS the device (fboost coredump 8-05), it doesn't just drop.
        if (bleMsys1Free() < 8) break;
        size_t n = std::min(chunk, txBuf.size() - txHead);
        // Don't split a multi-byte UTF-8 sequence across notifications: clients that decode each
        // notification independently would render the split halves as replacement chars. If the next
        // chunk would start on a continuation byte (10xxxxxx), trim back to the code-point boundary.
        if (txHead + n < txBuf.size())
            while (n > 1 && ((uint8_t) txBuf[txHead + n] & 0xC0) == 0x80) --n;
        txChar->setValue((uint8_t *) (txBuf.data() + txHead), n);
        lastNotifyOk = true;     // TxCallbacks::onStatus flips this to false on ENOMEM/error (backstop)
        txChar->notify();
        if (!lastNotifyOk) break; // pool exhausted anyway — retry this chunk on the next tick
        txHead += n;
    }
    if (txHead == txBuf.size()) { txBuf.clear(); txHead = 0; }
    if (logDropped && txBuf.size() - txHead < TX_BUF_CAP / 4) {
        logDropped = false; // clear first: bleWrite re-enters nothing, but keep it re-armable
        static const char marker[] = "\r\n[log output dropped - a command result may be missing]\r\n";
        bleWrite(marker, sizeof(marker) - 1);
    }
}

// Mirror logs to the connected client (registered as a log sink on connect, like telnet). Just queues;
// the error log a failed notify() emits re-enters here and is appended (bounded by TX_BUF_CAP), never
// re-notified, so there is no feedback storm — NimBLE's own INFO notify log is silenced to WARN below.
// The mirror only gets half the FIFO: with drop-newest a sustained log storm would otherwise fill
// the buffer and starve console replies — the client sees its commands time out (rpi bridge 8-05).
static void bleLogWrite(const char *str, uint16_t len) {
    std::lock_guard<std::recursive_mutex> lk(txMutex);
    if (txBuf.size() - txHead + len > TX_BUF_CAP / 2) { logDropped = true; return; }
    bleWrite(str, len);
}

// --- BLE callbacks ---

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String v = c->getValue();
        for (size_t i = 0; i < v.length(); ++i) rxQueue.enqueue(v[i]);
    }
};

// Firmware-data sink for OTA push. Runs on the NimBLE host task: only copies bytes into the OTA ring
// (otaBleStageBytes never touches flash); the network-loop tick drains them to the partition.
class FwRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String v = c->getValue();
        otaBleStageBytes((const uint8_t *) v.c_str(), v.length());
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *s) override {
        deviceConnected = true;
        addLogCallback(bleLogWrite, false); // no boot replay: it floods the FIFO and eats the next result
#if defined(CONFIG_BT_NIMBLE_ENABLED)
        linkEnc = false;
        connEnc = false;
        bondEvicted = false;
        rejLogged = false;
        connBonds = bondCount();
        linkT = millis();
        linkStep = LINK_IDLE; // linkTick() on the network loop, never from this host callback
#endif
        txArmSettle = true; // hold the TX drain until the link speeds up (see TX_SETTLE_MS)
        ESP_LOGI(TAG, "client connected");
    }

#if defined(CONFIG_BT_NIMBLE_ENABLED)
    // Security request only to bonded peers (silent re-encryption). A new peer pairs on its first
    // encrypted access (ATT insufficient authentication); asking on connect makes Android prompt early.
    void onConnect(BLEServer *, ble_gap_conn_desc *d) override {
        ble_store_key_sec k{};
        k.peer_addr = d->peer_id_addr;
        ble_store_value_sec v;
        if (linkEncRequired && ble_store_read_peer_sec(&k, &v) == 0) ble_gap_security_initiate(d->conn_handle);
    }
#endif

    void onDisconnect(BLEServer *s) override {
        deviceConnected = false;
        // UNCONDITIONAL. The otaBleActive() guard that used to be here made this
        // miss the case it exists for: otaBleSubmitCommand() only LATCHES a
        // begin, and the consumer tick executes it, so between those two points
        // a disconnect sees active == false and the guard swallowed the abort.
        // The tick then opened the OTA handle for a peer that was already gone -
        // with quiesce(true) fired, i.e. stopAndBackoff(10) and a halted sampler,
        // so the converter stayed down until someone rebooted it.
        // esp-ota-ble now cancels a latched begin, but only if it is TOLD, and
        // otaBleRequestAbort() has always been documented as safe to call with
        // nothing in flight. See esp-ota-ble/BUGS-2026-09-07.md defect 1.
        otaBleRequestAbort(); // net-loop tick aborts; never free OTA state on the host task
        teleBleRequestStop(); // ditto: net-loop tick frees the stream buffers, not the host task
        removeLogCallback(bleLogWrite);
        { std::lock_guard<std::recursive_mutex> lk(txMutex); txBuf.clear(); txHead = 0; }
        ESP_LOGI(TAG, "client disconnected, re-advertising");
        // With the broadcast enabled, the teleAdvTick reconciler is the ONLY caller of GAP
        // advertising APIs: a start here runs on the NimBLE host task and would race the
        // network-loop tick's stop/start (same cross-task class as the lld_con.c:3275 assert
        // that moved the link setup off the connect callback).
        if (!teleAdvEnabled()) s->startAdvertising();
    }
};

// Backpressure signal for bleTxDrain(): notify() invokes this synchronously on the same task with
// SUCCESS_NOTIFY on success or ERROR_GATT (rc=6 BLE_HS_ENOMEM) when the mbuf pool is exhausted.
class TxCallbacks : public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic *, Status s, uint32_t) override {
        if (s != SUCCESS_NOTIFY) lastNotifyOk = false;
    }
};

static RxCallbacks rxCallbacks;
static FwRxCallbacks fwRxCallbacks;
static ServerCallbacks serverCallbacks;
static TxCallbacks txCallbacks;

void bleConsoleBegin(const std::string &deviceName, const std::string &security, uint32_t passkey,
                     uint32_t pairWindowS) {
    if (bleStarted || bleGone) return;

    if (bleInited) {
        // Stack is already up from a previous start; just resume advertising (see bleInited note).
        BLEDevice::startAdvertising();
        bleStarted = true;
        teleAdvInit(); // `svc rs ble` re-reads tele.conf::adv_ms
        ESP_LOGI(TAG, "re-advertising as '%s' (NUS console)", deviceName.c_str());
        return;
    }

    advName = deviceName;
    BLEDevice::init(deviceName.c_str()); // const char* -> Arduino String
    BLEDevice::setMTU(247); // prefer a large ATT MTU so the client negotiates up from the 23-byte default

    // Reserve the TX FIFO once, at init, while the heap is still roomy. With size bounded to
    // TX_BUF_CAP (compaction in bleWrite) the append path then never reallocates — growing the
    // string mid-connect (log backlog burst) needed a ~16 KB doubling alloc that bad_alloc'd
    // and took the device down on a ~27 KB-free no-PSRAM heap (fboost coredumps 8-05).
    txBuf.reserve(TX_BUF_CAP + 64);

    // NimBLE logs "GAP procedure initiated: notify;" at INFO on *every* notification. Because we
    // mirror logs to the BLE link, that feeds back into more notifications — a self-sustaining storm
    // that corrupts the console. Keep the NimBLE tag at WARN.
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    // Per-characteristic encryption requirement enforces pairing before commands are accepted
    // (NimBLE backend; under Bluedroid the *_ENC/_AUTHEN flags are 0 and degrade to open).
    uint32_t writeProps = BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR;
    auto *sec = new BLESecurity();
    BLESecurity::setForceAuthentication(false); // see ServerCallbacks::onConnect
    if (security == "passkey") {
        sec->setPassKey(true, passkey);
        sec->setCapability(ESP_IO_CAP_OUT);            // device displays the passkey
        sec->setAuthenticationMode(true, true, true);  // bond + MITM + secure-connections
        writeProps |= BLECharacteristic::PROPERTY_WRITE_AUTHEN;
        ESP_LOGI(TAG, "security: passkey (bond+MITM)");
    } else if (security == "justworks") {
        sec->setCapability(ESP_IO_CAP_NONE);
        sec->setAuthenticationMode(true, false, true); // bond + no MITM + secure-connections
        writeProps |= BLECharacteristic::PROPERTY_WRITE_ENC;
        ESP_LOGI(TAG, "security: justworks (encrypted, no passkey)");
    } else {
        ESP_LOGW(TAG, "security: none (open console)");
    }

    otaBleInstallHooks();

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(&serverCallbacks);
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    linkEncRequired = security == "passkey" || security == "justworks";
    if (linkEncRequired) secName = security == "passkey" ? "passkey" : "justworks";
    BLEDevice::setDeviceCallbacks(&storeCallbacks);
    if (pairWindowS) pairDefaultS = pairWindowS;
    pairSet(pairWindowS);
    if (linkEncRequired) ESP_LOGI(TAG, "pairing window %lu s", (unsigned long) pairWindowS);
    static ble_gap_event_listener gapListener;
    ble_gap_event_listener_register(&gapListener, linkGapEvent, nullptr);
#endif

    BLEService *svc = bleServer->createService(NUS_SERVICE_UUID);

    txChar = svc->createCharacteristic(NUS_TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    txChar->setCallbacks(&txCallbacks); // onStatus drives bleTxDrain() backpressure
#if defined(CONFIG_BLUEDROID_ENABLED)
    txChar->addDescriptor(new BLE2902()); // NimBLE adds the CCCD automatically from PROPERTY_NOTIFY
#endif

    BLECharacteristic *rxChar = svc->createCharacteristic(NUS_RX_CHAR_UUID, writeProps);
    rxChar->setCallbacks(&rxCallbacks);

    // OTA firmware push: write-no-response only (high-throughput), same pairing requirement as RX.
    uint32_t fwProps = (writeProps & ~(uint32_t) BLECharacteristic::PROPERTY_WRITE)
                       | BLECharacteristic::PROPERTY_WRITE_NR;
    BLECharacteristic *fwChar = svc->createCharacteristic(NUS_FW_CHAR_UUID, fwProps);
    fwChar->setCallbacks(&fwRxCallbacks);

    teleBleCreateChar(svc); // telemetry notify char (no-op unless CONFIG_FUGU_WITH_BLE_TELE)

    svc->start();
    improvCreateService(bleServer, writeProps);

    // Passive scanners never see the scan response, so the name rides the advert (the wrapper builds
    // flags + name, shortened to fit, and rebuilds it after a host reset) and the NUS UUID the scan
    // response, which active scanners merge.
    BLEAdvertisementData rsp;
    rsp.setCompleteServices(BLEUUID(NUS_SERVICE_UUID));
    BLEDevice::getAdvertising()->setScanResponseData(rsp);
    BLEDevice::startAdvertising();

    bleInited = true;
    bleStarted = true;
    teleAdvInit();
    ESP_LOGI(TAG, "advertising as '%s' (NUS console)", deviceName.c_str());
}

void bleConsoleEnd() {
    if (!bleStarted) return;
    if (otaBleActive()) otaBleAbort(); // no tick runs once stopped
    // Stop advertising only — do NOT BLEDevice::deinit(): the Arduino BLE wrapper can't be
    // re-initialized cleanly (see bleInited note), so the stack stays up across stop/start.
    removeLogCallback(bleLogWrite);
    BLEDevice::stopAdvertising();
    if (deviceConnected && bleServer) bleServer->disconnect(bleServer->getConnId());
    deviceConnected = false;
    bleStarted = false;
    char c;
    while (rxQueue.try_dequeue(c)) {} // drop stale input
}

void bleConsoleLoop(time_ms nowMs) {
    if (!bleStarted) return;
    // Let a blocking command (e.g. `ota <url>`) push pending output while it runs — otherwise its
    // progress/result sits in txBuf until it returns, and on a successful OTA the reboot eats it.
    consoleFlushHook = []() { bleTxDrain(wallClockMs()); };
    static ConsoleLine line{};
    loopConsole(line, bleRead, bleWrite, nowMs, linkEncRequired ? nullptr : "ble.conf ble_security=none");
    consoleFlushHook = nullptr;
    bleTxDrain(nowMs); // flush queued console/log output, paced by NimBLE buffer availability
    otaBleTick(nowMs); // drain any staged OTA firmware bytes to flash (no-op when not updating)
    teleBleTick(nowMs); // batch + notify the telemetry stream (no-op unless streaming)
    teleAdvTick(nowMs); // broadcast payload refresh + adv-mode reconciler (no-op unless enabled)
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    if (pairOpen && (int32_t) (millis() - pairEndMs) >= 0) {
        pairSet(0);
        ESP_LOGI(TAG, "pairing window closed");
    }
#endif
    improvTick(nowMs);
}

bool bleConsoleRelease() {
    if (!bleInited) return false;
    bleConsoleAwaitTxDrain(0, 1000);
    bleServer->setCallbacks(nullptr); // no re-advertising from onDisconnect into the teardown
    BLEDevice::stopAdvertising();
    if (bleServer->getConnectedCount()) bleServer->disconnect(bleServer->getConnId());
    // deinit() deletes the server before the host stops, so no disconnect event may still be due
    for (int i = 0; i < 500 && bleServer->getConnectedCount(); ++i) vTaskDelay(pdMS_TO_TICKS(10));
    if (bleServer->getConnectedCount()) {
        bleServer->setCallbacks(&serverCallbacks);
        ESP_LOGW(TAG, "client did not disconnect, BLE kept");
        return false;
    }
    otaBleRequestAbort();
    teleBleSetStreaming(false);
    bleConsoleEnd();
    { std::lock_guard<std::recursive_mutex> lk(txMutex); txBuf.clear(); txHead = 0; }
    BLEDevice::deinit(true);
    bleServer = nullptr;
    txChar = nullptr;
    bleInited = false;
    bleGone = true;
    return true;
}

bool bleConsoleConnected() {
    if (!deviceConnected || !bleServer) return false;
#if !defined(CONFIG_BLUEDROID_ENABLED)
    // Ask the stack, not the callback flag: a missed onDisconnect would otherwise pin the
    // advertising reconciler to the connected branch, i.e. non-connectable adv forever.
    ble_gap_conn_desc d;
    if (ble_gap_conn_find(bleServer->getConnId(), &d) != 0) return false;
#endif
    return true;
}

void bleConsoleResumeAdv() {
    if (bleStarted) BLEDevice::startAdvertising();
}

// The params stick to the wrapper's advertising object, so every later start (onDisconnect, the
// tele_adv reconciler, `svc on ble`) uses them; tele_adv's non-connectable advert has its own.
bool bleConsoleSetAdvMs(uint16_t ms) {
    if (!bleInited || bleConsoleConnected() || teleAdvOwnsAdv()) return false;
    auto *adv = BLEDevice::getAdvertising();
    adv->setMinInterval(ms * 8 / 5); // 0.625 ms units
    adv->setMaxInterval(ms * 8 / 5);
    if (bleStarted && adv->isAdvertising()) {
        adv->stop();
        BLEDevice::startAdvertising();
    }
    advMs = ms;
    return true;
}

uint16_t bleConsoleAdvMs() { return advMs; }

const char *bleConsoleName() { return advName.c_str(); }

size_t bleNameAdv(uint8_t *ad) {
    const size_t len = advName.size(), n = std::min(len, (size_t) 26);
    ad[0] = 2; ad[1] = 0x01; ad[2] = 0x06;
    if (!n) return 3;
    ad[3] = (uint8_t) (n + 1);
    ad[4] = len > n ? 0x08 : 0x09;
    memcpy(&ad[5], advName.data(), n);
    return 5 + n;
}

size_t bleConsoleChunk() {
    if (!deviceConnected || !bleServer) return 20;
    uint16_t mtu = bleServer->getPeerMTU(bleServer->getConnId());
    return mtu > 23 ? (size_t) (mtu - 3) : 20;
}

bool bleConsoleLinkSettled() {
    return deviceConnected && !txArmSettle && wallClockMs() - txConnectMs >= TX_SETTLE_MS;
}

void bleConsoleAwaitTxDrain(unsigned lowWater, unsigned timeoutMs) {
    if (!bleStarted || !deviceConnected) return;
    time_ms start = wallClockMs();
    for (;;) {
        size_t backlog;
        { std::lock_guard<std::recursive_mutex> lk(txMutex); backlog = txBuf.size() - txHead; }
        if (backlog <= lowWater || !deviceConnected) return;
        if (wallClockMs() - start > timeoutMs) return; // client stalled; proceed rather than hang
        bleTxDrain(wallClockMs());
        vTaskDelay(1); // let the NimBLE host transmit and refill the mbuf pool
    }
}

#if defined(CONFIG_BT_NIMBLE_ENABLED)
bool blePairGated() { return linkEncRequired; }

uint32_t blePairRemainingS() {
    const int32_t ms = (int32_t) (pairEndMs - millis());
    return pairOpen && ms > 0 ? (ms + 999) / 1000 : 0;
}

static bool parseAddr(const char *s, uint8_t *v) {
    for (int i = 5; i >= 0; --i) {
        char *e;
        unsigned long b = strtoul(s, &e, 16);
        if (e == s || e - s > 2 || *e != (i ? ':' : 0)) return false;
        v[i] = (uint8_t) b;
        s = e + 1;
    }
    return true;
}

const char *bleCommand(const char *sub, const char *arg) {
    if (!bleInited) return "service not running";
    ble_addr_t a[CONFIG_BT_NIMBLE_MAX_BONDS];
    int n = 0;
    ble_store_util_bonded_peers(a, &n, CONFIG_BT_NIMBLE_MAX_BONDS);
    char s[18];
    if (!strcmp(sub, "pair")) {
        char *e;
        unsigned long t = *arg ? strtoul(arg, &e, 10) : pairDefaultS;
        if (*arg && *e) return "expected seconds";
        pairSet(t);
        ESP_LOGI(TAG, "pairing window %lu s", (unsigned long) blePairRemainingS());
    } else if (!strcmp(sub, "unpair")) {
        uint8_t v[6];
        const bool all = !strcmp(arg, "all");
        if (!all && !parseAddr(arg, v)) return "expected unpair <aa:bb:cc:dd:ee:ff|all>";
        int k = 0;
        for (int i = 0; i < n; ++i) {
            if (!all && memcmp(a[i].val, v, 6)) continue;
            ble_gap_unpair(&a[i]);
            ESP_LOGI(TAG, "unpaired %s", fmtAddr(a[i], s));
            ++k;
        }
        if (!k) return "no such bond";
        // removing a peer IRK preempts GAP, which ends advertising (tele_adv's reconciler restarts its own)
        if (!teleAdvEnabled() && !bleConsoleConnected() && !ble_gap_adv_active()) bleConsoleResumeAdv();
        n = 0;
        ble_store_util_bonded_peers(a, &n, CONFIG_BT_NIMBLE_MAX_BONDS);
    } else if (*sub && strcmp(sub, "bonds")) {
        return "expected pair [s] | bonds | unpair <addr|all>";
    }
    UART_LOG("ble: security %s, pairing %s %lu s, bonds %d/%d", secName, pairOpen ? "open" : "closed",
             (unsigned long) blePairRemainingS(), n, CONFIG_BT_NIMBLE_MAX_BONDS);
    ble_gap_conn_desc d;
    const bool conn = deviceConnected && bleServer && ble_gap_conn_find(bleServer->getConnId(), &d) == 0;
    for (int i = 0; i < n; ++i)
        UART_LOG("  %s %s%s", fmtAddr(a[i], s), a[i].type ? "random" : "public",
                 conn && !ble_addr_cmp(&a[i], &d.peer_id_addr) ? " (connected)" : "");
    return nullptr;
}
#else
bool blePairGated() { return false; }

uint32_t blePairRemainingS() { return 0; }

const char *bleCommand(const char *, const char *) { return "not supported"; }
#endif

#else // !CONFIG_FUGU_WITH_BLE — no-op stubs so callers (and the service wrapper) link without the BLE stack

void bleConsoleBegin(const std::string &, const std::string &, uint32_t, uint32_t) {}

void bleConsoleEnd() {}

void bleConsoleLoop(time_ms) {}

bool bleConsoleConnected() { return false; }

bool bleConsoleRelease() { return false; }

bool bleOtaRebootDue() { return false; }

int bleMsys1Free() { return 0; }

size_t bleConsoleChunk() { return 20; }

bool bleConsoleLinkSettled() { return false; }

void bleConsoleResumeAdv() {}

bool bleConsoleSetAdvMs(uint16_t) { return false; }

uint16_t bleConsoleAdvMs() { return 0; }

const char *bleConsoleName() { return ""; }

size_t bleNameAdv(uint8_t *ad) { ad[0] = 2; ad[1] = 0x01; ad[2] = 0x06; return 3; }

void bleConsoleAwaitTxDrain(unsigned, unsigned) {}

bool blePairGated() { return false; }

uint32_t blePairRemainingS() { return 0; }

const char *bleCommand(const char *, const char *) { return "not supported"; }

#endif
