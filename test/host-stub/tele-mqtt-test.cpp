// Host test for the MQTT telemetry sink (src/tele/tele_mqtt.cpp) against the stubbed esp-mqtt
// client: refusals, lease, record format, expiry, stop, service stop, reconnect, lease cap.
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -DWITH_TELE_BINARY -DWITH_TELE_MQTT -I test/host-stub/mqtt-stub \
//       -I test/host-stub -I src -o /tmp/tele-mqtt-test test/host-stub/tele-mqtt-test.cpp && /tmp/tele-mqtt-test

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

class String {
    std::string s;
public:
    String(const char *c = "") : s(c) {}
    String(std::string c) : s(std::move(c)) {}
    void trim() {}
    unsigned length() const { return s.size(); }
    const char *c_str() const { return s.c_str(); }
    friend String operator+(const String &a, const char *b) { return a.s + b; }
};
bool handleCommand(const String &) { return true; }
void logCmdResult(bool, const char *, int64_t) {}
const std::string &getHostname(bool) { static std::string h{"test"}; return h; }

#include "../../src/tele/mqtt.cpp"
#include "../../src/tele/tele_mqtt.cpp"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;

// --- tele_core stand-ins ---
SymbolTable g_symtab;
bool g_teleBinary = false;
bool timeSynced = false;
bool g_teleUdpActive = false;
static bool g_confBinary = false;   // tele.conf binary
void teleLoadWireConf() {
    g_teleBinary = g_confBinary;
    if (teleMqttActive()) g_teleBinary = true;
}
static size_t g_freeHeap = 100000;
size_t heap_caps_get_free_size(uint32_t) { return g_freeHeap; }

void teleProducePoint() {   // one point per tick, fanned out like telemetryAddPoint
    assert(g_teleBinary);
    TelePoint p("mppt");
    p.addTag("device", "test");
    p.addField("I", 1.5f);
    p.addField("Ui", 40.f);
    p.addField("P", 60.f);
    p.setTimeMs();
    teleMqttEnqueue(p.takeWire());
}

// --- esp-mqtt stub ---
struct esp_mqtt_client { esp_event_handler_t h = nullptr; void *arg = nullptr; };
static int g_live = 0;
static std::vector<std::pair<std::string, std::string>> g_pubs;

const char *esp_err_to_name(esp_err_t) { return "ERR"; }
uint32_t esp_get_free_heap_size() { return 8000; }
int64_t esp_timer_get_time() { return 0; }
TaskHandle_t xTaskGetCurrentTaskHandle() { return nullptr; }
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *) { ++g_live; return new esp_mqtt_client; }
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t c, esp_mqtt_event_id_t, esp_event_handler_t h, void *a) {
    c->h = h; c->arg = a; return ESP_OK;
}
esp_err_t esp_mqtt_client_unregister_event(esp_mqtt_client_handle_t c, esp_mqtt_event_id_t, esp_event_handler_t) {
    c->h = nullptr; return ESP_OK;
}
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t) { return ESP_OK; }
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t) { return ESP_OK; }
esp_err_t esp_mqtt_client_disconnect(esp_mqtt_client_handle_t) { return ESP_OK; }
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t c) { --g_live; delete c; return ESP_OK; }
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t, const char *, int) { return 1; }
int esp_mqtt_client_publish(esp_mqtt_client_handle_t, const char *t, const char *d, int len, int, int) {
    g_pubs.emplace_back(t, std::string(d, len ? (size_t) len : std::strlen(d)));
    return 0;
}

namespace {

struct PathPeek : MqttService {
    static void set(MqttService &m, const char *p) { m.*(&PathPeek::_confPath) = p; }
};
const char *kConf = "/tmp/_fugu_tele_mqtt_test.conf";

void deliver(esp_mqtt_event_id_t id) {
    esp_mqtt_event_t ev{};
    ev.event_id = id;
    ev.client = MQTT.client;
    MQTT.client->h(MQTT.client->arg, "MQTT_EVENTS", id, &ev);
}

time_ms now() { return wallClockMs(); }
void advance(time_ms ms) { setWallClockUs(wallClockUs() + ms * 1000); }

// one tick per 100 ms over `ms`
void run(time_ms ms) {
    for (time_ms t = 0; t < ms; t += 100) {
        advance(100);
        teleMqttTick(now());
    }
}

uint64_t varint(const uint8_t *&p) {
    uint64_t v = 0;
    for (int s = 0;; s += 7) {
        v |= uint64_t(*p & 0x7f) << s;
        if (!(*p++ & 0x80)) return v;
    }
}

// frame types of one record: <cid=0>(<varint len><type>...)*
std::vector<int> frames(const std::string &rec) {
    assert(!rec.empty() && rec[0] == 0);
    std::vector<int> out;
    auto *p = (const uint8_t *) rec.data() + 1, *end = (const uint8_t *) rec.data() + rec.size();
    while (p < end) {
        uint64_t n = varint(p);
        assert(n && p + n <= end);
        out.push_back(*p);
        p += n;
    }
    return out;
}

void connected() {
    MQTT.stop();
    assert(MQTT.start());
    deliver(MQTT_EVENT_CONNECTED);
    assert(MQTT.isConnected());
}

void reset() {
    teleMqttLease(0);
    connected();
    timeSynced = true;
    g_teleUdpActive = g_confBinary = false;
    teleLoadWireConf();
    g_pubs.clear();
}

void test_refusals() {
    reset();
    timeSynced = false;
    std::printf("no time: %s\n", teleMqttLease(20));
    assert(teleMqttLease(20) && !teleMqttActive());
    timeSynced = true;
    deliver(MQTT_EVENT_DISCONNECTED);
    std::printf("disconnected: %s\n", teleMqttLease(20));
    assert(teleMqttLease(20) && !teleMqttActive());
    deliver(MQTT_EVENT_CONNECTED);
    g_teleUdpActive = true;
    std::printf("udp text wire: %s\n", teleMqttLease(20));
    assert(teleMqttLease(20) && !teleMqttActive());
    g_teleUdpActive = false;
    g_freeHeap = 1000;
    std::printf("low heap: %s\n", teleMqttLease(20));
    assert(teleMqttLease(20) && !teleMqttActive());
    g_freeHeap = 100000;
    assert(!g_teleBinary && g_pubs.empty());
}

void test_lease_publish_expire() {
    reset();
    assert(!teleMqttLease(2) && teleMqttActive() && g_teleBinary);
    assert(teleMqttLeaseLeftS(now()) == 2);
    run(1500);
    assert(!g_pubs.empty());
    std::printf("records in 1.5 s: %zu, first %zu B\n", g_pubs.size(), g_pubs[0].second.size());
    assert(g_pubs[0].first == "pv/tele/test");
    auto f = frames(g_pubs[0].second);
    assert(f[0] == 2 && f.back() == 1);   // table then data
    size_t pts = 0;
    for (auto &r: g_pubs) for (int t: frames(r.second)) pts += t == 1;
    std::printf("points: %zu\n", pts);
    assert(pts >= 10);
    run(500);
    assert(!teleMqttActive() && !g_teleBinary);   // expired, wire back to conf (text)
    size_t n = g_pubs.size();
    run(1000);
    assert(g_pubs.size() == n);
}

void test_lease_zero_stops() {
    reset();
    g_confBinary = true;
    assert(!teleMqttLease(30) && teleMqttActive());
    run(300);
    assert(!teleMqttLease(0) && !teleMqttActive() && g_teleBinary);   // conf says binary
    size_t n = g_pubs.size();
    run(1000);
    assert(g_pubs.size() == n);
}

void test_service_stopped() {
    reset();
    assert(!teleMqttLease(30));
    run(300);
    MQTT.stop();
    teleMqttTick(now());
    assert(!teleMqttActive() && !g_teleBinary);
}

void test_reconnect_resends_table() {
    reset();
    assert(!teleMqttLease(60));
    run(3000);   // past the table burst
    assert(frames(g_pubs.back().second)[0] == 1);
    deliver(MQTT_EVENT_DISCONNECTED);
    size_t n = g_pubs.size();
    run(1000);
    assert(g_pubs.size() == n && teleMqttActive());
    deliver(MQTT_EVENT_CONNECTED);
    run(700);
    assert(g_pubs.size() > n);
    assert(frames(g_pubs[n].second)[0] == 2);
}

void test_lease_cap() {
    reset();
    assert(!teleMqttLease(5000));
    assert(teleMqttLeaseLeftS(now()) == 3600);
    assert(!teleMqttLease(10) && teleMqttLeaseLeftS(now()) == 10);   // re-lease sets the new end
}

} // namespace

int main(int argc, char **argv) {
    FILE *f = std::fopen(kConf, "w");
    std::fputs("broker_uri=mqtt://127.0.0.1:1883\n", f);
    std::fclose(f);
    PathPeek::set(MQTT, kConf);
    setWallClockUs(1000000);
    const struct { const char *name; void (*fn)(); } tests[] = {
            {"refusals", test_refusals},
            {"lease_publish_expire", test_lease_publish_expire},
            {"lease_zero_stops", test_lease_zero_stops},
            {"service_stopped", test_service_stopped},
            {"reconnect_resends_table", test_reconnect_resends_table},
            {"lease_cap", test_lease_cap},
    };
    for (auto &t: tests)
        if (argc < 2 || std::strstr(t.name, argv[1])) {
            std::printf("--- %s\n", t.name);
            t.fn();
        }
    teleMqttLease(0);
    MQTT.stop();
    assert(g_live == 0);
    std::remove(kConf);
    std::printf("tele-mqtt-test: all passed\n");
    return 0;
}
