#include "tele_ble.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "console_ble.h"
#include "telemetry.h"
#include "tele_core.h"
#include "compress.h"
#include "../conf.h"
#include "../mppt.h"

extern MpptController mppt;

#define NUS_TELE_CHAR_UUID "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"

static const char *TAG = "teleble";

static BLECharacteristic *teleChar = nullptr;
static bool streaming = false;
static volatile bool stopRequested = false;   // set on the NimBLE host task (disconnect)
static bool lastNotifyOk = true;

// Everything below is touched only from the network loop (producer telemetryAddPoint and the
// drain both run there); buffers are allocated on `tele-ble 1` and freed on stop/disconnect
// (internal heap is tight on no-PSRAM boards).
static TampCompress *comp = nullptr;
static std::string rawBatch;                 // uncompressed frames awaiting a packet
static std::string outBuf;                   // framed records awaiting notify
static std::string packedBuf;                // per-packet compressor output (reused)
static size_t outHead = 0;
static size_t droppedBytes = 0;
static time_ms lastFlushMs = 0;
static constexpr size_t RAW_CAP = 1024;      // heap is tight (no-PSRAM ~27 KB free); keep the
static constexpr size_t OUT_CAP = 3072;      // stream's footprint small, drop-newest covers bursts
static constexpr time_ms FLUSH_MS = 250;     // ship a partial batch after this long

class TeleTxCallbacks : public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic *, Status s, uint32_t) override {
        if (s != SUCCESS_NOTIFY) lastNotifyOk = false;
    }
};
static TeleTxCallbacks teleTxCallbacks;

void teleBleCreateChar(BLEService *svc) {
    teleChar = svc->createCharacteristic(NUS_TELE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    teleChar->setCallbacks(&teleTxCallbacks);
#if defined(CONFIG_BLUEDROID_ENABLED)
    teleChar->addDescriptor(new BLE2902());
#endif
}

bool teleBleStreaming() { return streaming; }

size_t teleBleDropped() { return droppedBytes; }

void teleBleRequestStop() { stopRequested = true; }

static void teleBleStop() {
    streaming = false;
    std::string().swap(rawBatch);
    std::string().swap(outBuf);
    std::string().swap(packedBuf);
    outHead = 0;
    delete comp;
    comp = nullptr;
    teleLoadWireConf();   // un-force g_teleBinary (back to tele.conf::binary)
}

const char *teleBleSetStreaming(bool on) {
    if (!on) {
        teleBleStop();
        return nullptr;
    }
    if (streaming) return nullptr;
    if (!ConfFile{"/littlefs/conf/tele.conf"}.getLong("ble", 1))
        return "disabled (tele.conf ble=0)";
    if (!timeSynced)
        return "clock not set (send `set-time <epoch_ms>` first)";
    if (!bleConsoleConnected())
        return "no BLE client connected";
    // The UDP wire caches its compressor at service start: flipping g_teleBinary under a running
    // text wire would route binary frames down its text branch and ship garbage to InfluxDB.
    if (g_teleUdpActive && !g_teleBinary)
        return "UDP telemetry runs the text wire (set-config tele.conf binary 1, svc rs tele)";
    // Reserve everything up front so the streaming path never allocates (see txBuf note in
    // console_ble.cpp — growing strings on this heap is how the device dies).
    try {
        comp = new TampCompress();
        rawBatch.reserve(RAW_CAP + 64);
        outBuf.reserve(OUT_CAP + 64);
        packedBuf.reserve(RAW_CAP * 3 / 2);
    } catch (const std::bad_alloc &) {
        teleBleStop();
        return "out of memory";
    }
    g_teleBinary = true;
    g_symtab.forceResend();   // shared table: burst also reaches a concurrent UDP wire (harmless)
    droppedBytes = 0;
    lastFlushMs = 0;
    streaming = true;
    return nullptr;
}

void teleBleEnqueue(const std::string &frame) {
    if (!streaming) return;
    // Single-task invariant: producer (telemetryAddPoint) and drain both run on the network
    // loop. Guards against a future producer on another task (e.g. dcdcDataChanged).
    static TaskHandle_t owner = nullptr;
    if (!owner) owner = xTaskGetCurrentTaskHandle();
    assert(xTaskGetCurrentTaskHandle() == owner);
    if (rawBatch.size() + frame.size() > RAW_CAP) { droppedBytes += frame.size(); return; }
    try { rawBatch += frame; } catch (const std::bad_alloc &) { droppedBytes += frame.size(); }
}

// <0x7E><varint len><cid><payload> — payload identical to one UDP telemetry datagram.
static void packetize(time_ms nowMs) {
    packedBuf.clear();
    try {
        if (comp->packet((const uint8_t *) rawBatch.data(), rawBatch.size(), packedBuf)) {
            if (outBuf.size() - outHead + packedBuf.size() + 6 > OUT_CAP) {
                droppedBytes += packedBuf.size();
            } else {
                outBuf += '\x7e';
                putVarint(outBuf, packedBuf.size() + 1);
                outBuf += (char) comp->id();
                outBuf += packedBuf;
            }
        } else {
            droppedBytes += rawBatch.size();   // compressor failure must not look like "no loss"
        }
    } catch (const std::bad_alloc &) {
        droppedBytes += rawBatch.size();       // heap-tight boards: drop, never unwind the net loop
    }
    rawBatch.clear();
    lastFlushMs = nowMs;
}

void teleBleTick(time_ms nowMs) {
    if (stopRequested) { stopRequested = false; teleBleStop(); }
    if (!streaming || !teleChar) return;
    if (!bleConsoleLinkSettled()) return;

    mppt.telemetry();   // produces points when the WiFi TelemetryService isn't ticking

    size_t chunk = bleConsoleChunk();
    if (!rawBatch.empty() &&
        (rawBatch.size() >= comp->maxBatchRaw(chunk) || nowMs - lastFlushMs > FLUSH_MS))
        packetize(nowMs);

    while (outHead < outBuf.size()) {
        // Leave more msys_1 headroom (floor 10 of 12 blocks) than the console drain (floor 8): telemetry is the
        // lower-priority stream and must not starve the console/log mirror sharing the link — and
        // ble_l2cap_tx running dry mid-send panics (see the console drain's floor comment).
        if (bleMsys1Free() < 10) break;
        size_t n = std::min(chunk, outBuf.size() - outHead);
        teleChar->setValue((uint8_t *) (outBuf.data() + outHead), n);
        lastNotifyOk = true;
        teleChar->notify();
        if (!lastNotifyOk) break;   // pool exhausted; retry this chunk next tick
        outHead += n;
    }
    if (outHead == outBuf.size()) { outBuf.clear(); }
    else if (outHead) outBuf.erase(0, outHead);   // compact: under sustained partial back-
    outHead = 0;                                  // pressure the sent prefix would grow unbounded
}
