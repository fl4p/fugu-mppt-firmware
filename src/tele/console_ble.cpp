#include "console_ble.h"

#ifdef WITH_BLE

#include <algorithm>
#include <mutex>

#include <Arduino.h> // String
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#if defined(CONFIG_BT_NIMBLE_ENABLED)
#include <host/ble_gap.h>
#endif
#include <BLESecurity.h>
#include <os/os_mbuf.h> // os_msys_num_free(): peek NimBLE's free mbuf pool before notifying
#if !defined(CONFIG_BLUEDROID_ENABLED)
#include <host/ble_gap.h> // ble_gap_conn_find(): connection state from the stack, not our callbacks
#endif

#include "console.h"   // loopConsole
#include "util.h"      // wallClockMs
#include "logging.h"   // addLogCallback / removeLogCallback, ESP_LOG*
#include "etc/readerwriterqueue.h"
#include <ota_ble.h>     // OTA-over-BLE firmware push (esp-ota-ble component; FW char data sink)
#include "adc/sampling.h" // ADC_Sampler -- halted while flash is busy
#include "tele_ble.h"    // BLE telemetry stream (TELE characteristic, WITH_BLE_TELE)
#include "tele_adv.h"    // advertising telemetry broadcast (WITH_BLE_ADV)

// esp-ota-ble host hooks. The status sink goes to ESP_LOG* at the level the module asked for, with
// the tag still "otab" -- the wire protocol IS these log lines (they are mirrored to the client), so
// the format and the severity both have to survive. Flattening FAIL lines to INFO would mean raising
// this tag's level silently swallows exactly what the host needs to tell failure from a dead link.
extern ADC_Sampler adcSampler;
void stopAndBackoff(uint32_t secondsDelay);
void systemRestart();

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

// Single-producer (NimBLE host task, onWrite) / single-consumer (network loop, bleRead) byte queue.
static moodycamel::ReaderWriterQueue<char> rxQueue{256};

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
// The settle window also defers our connection-parameter request: issuing a device-initiated LL
// connection update synchronously from the host connect callback — while justworks pairing/feature
// exchange is still in flight and Wi-Fi coex is active — races the peer's procedures and trips the
// controller assert lld_con.c:3275 (r_lld_con_param_update). We instead request it once from the
// network-loop drain after the window, off the host callback. Backpressure tolerates the brief
// slow-interval drain before the faster interval applies.
// Command results (pwm-freq, sync, dt, status, ...) are emitted with UART_LOG, so they travel the
// LOG path and are subject to the half-FIFO cap above -- a client can lose a reply with no error
// and no gap, and cannot tell a command that failed from one whose answer was discarded. Until
// results get their own channel, at least make the loss visible: latch a drop and emit one marker
// once the backlog clears, through bleWrite (full FIFO) so the marker itself cannot be dropped.
static bool logDropped = false;
static volatile bool txArmSettle = false;
static volatile bool connParamsPending = false;
static int phySetRc = 0;
static time_ms phyReportAt = 0;
static time_ms txConnectMs = 0;

static constexpr time_ms TX_SETTLE_MS = 500;

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
    if (phyReportAt && millis() >= phyReportAt) {
        phyReportAt = 0;
#if defined(CONFIG_BT_NIMBLE_ENABLED) && defined(CONFIG_BT_NIMBLE_LL_CFG_FEAT_LE_2M_PHY)
        uint8_t txPhy = 0, rxPhy = 0;
        const int rrc = ble_gap_read_le_phy(bleServer->getConnId(), &txPhy, &rxPhy);
        // set_rc==0 means only that the host ACCEPTED the request, not that the link moved.
        // tx/rx: 1=1M, 2=2M, 3=coded.
        ESP_LOGI(TAG, "phy: set_rc=%d read_rc=%d tx=%u rx=%u",
                 phySetRc, rrc, (unsigned) txPhy, (unsigned) rxPhy);
#endif
    }

    if (connParamsPending) {
        // Deferred off the host connect callback — see the txArmSettle note for why.
        // 6..12 = 7.5..15ms interval, latency 0, supervision timeout 400 = 4s.
        connParamsPending = false;
        bleServer->requestConnParams(bleServer->getConnId(), 6, 12, 0, 400);
#if defined(CONFIG_BT_NIMBLE_ENABLED) && defined(CONFIG_BT_NIMBLE_LL_CFG_FEAT_LE_2M_PHY)
        // Ask for the 2M PHY, which doubles the on-air symbol rate.
        //
        // MEASURED, and it does NOT speed up an OTA push: 36.5/38.4/38.2 s on 2M against
        // 38.3/35.9/35.6/39.2 s on 1M (flu, 1.76 MB, same host, 2026-09-08) -- indistinguishable.
        // The link is not symbol-rate-bound: at the 15 ms connection interval the transfer moves
        // ~830 B per connection event, which already fits comfortably in an event, so halving the
        // air time of each packet buys nothing. What is left is connection events and credit
        // round-trips. Kept anyway because it engages cleanly (verified tx=2 rx=2) and benefits
        // latency-sensitive console/telemetry traffic, but do NOT expect it to make OTA faster,
        // and do not re-derive that -- the numbers above are the answer.
        //
        // Must be a REQUEST, not just a default. LE Set Default PHY only states a preference for
        // when the PEER starts the procedure; it never initiates one, so a central that never asks
        // leaves the link on 1M. This actively runs the PHY update procedure.
        //
        // Deferred here for the same reason as the conn-params request above: issuing it from the
        // host connect callback trips a controller assert. Best effort — a peer or controller that
        // refuses simply stays on 1M, which is the pre-existing behaviour.
        {
            const uint16_t ch = bleServer->getConnId();
            const int rc = ble_gap_set_prefered_le_phy(ch, BLE_GAP_LE_PHY_2M_MASK,
                                                       BLE_GAP_LE_PHY_2M_MASK, 0);
            phySetRc = rc;
            phyReportAt = millis() + 3000; // see below
        }
#endif
    }

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
        if (os_msys_num_free() < 8) break;
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
        // Defer the fast-interval connection-param request to the network-loop drain (see TX_SETTLE_MS
        // note): requesting it here, in the host connect callback, trips controller assert lld_con.c:3275.
        connParamsPending = true;
        txArmSettle = true; // hold the TX drain until the link speeds up (see TX_SETTLE_MS)
        ESP_LOGI(TAG, "client connected");
    }

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
        // that forced connParamsPending off this callback).
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

void bleConsoleBegin(const std::string &deviceName, const std::string &security, uint32_t passkey) {
    if (bleStarted) return; // already advertising

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

    teleBleCreateChar(svc); // telemetry notify char (no-op unless WITH_BLE_TELE)

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    bleInited = true;
    bleStarted = true;
    teleAdvInit();
    ESP_LOGI(TAG, "advertising as '%s' (NUS console)", deviceName.c_str());
}

void bleConsoleEnd() {
    if (!bleStarted) return;
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
    loopConsole(bleRead, bleWrite, nowMs);
    consoleFlushHook = nullptr;
    bleTxDrain(nowMs); // flush queued console/log output, paced by NimBLE buffer availability
    otaBleTick(nowMs); // drain any staged OTA firmware bytes to flash (no-op when not updating)
    teleBleTick(nowMs); // batch + notify the telemetry stream (no-op unless streaming)
    teleAdvTick(nowMs); // broadcast payload refresh + adv-mode reconciler (no-op unless enabled)
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

const char *bleConsoleName() { return advName.c_str(); }

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

#else // !WITH_BLE — no-op stubs so callers (and the service wrapper) link without the BLE stack

void bleConsoleBegin(const std::string &, const std::string &, uint32_t) {}

void bleConsoleEnd() {}

void bleConsoleLoop(time_ms) {}

bool bleConsoleConnected() { return false; }

size_t bleConsoleChunk() { return 20; }

bool bleConsoleLinkSettled() { return false; }

void bleConsoleResumeAdv() {}

const char *bleConsoleName() { return ""; }

void bleConsoleAwaitTxDrain(unsigned, unsigned) {}

#endif
