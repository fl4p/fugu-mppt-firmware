// Host test for MqttService (src/tele/mqtt.cpp) against a stubbed esp-mqtt client: restart, failed
// init/start, a throwing preStart, and a CONNECTED event that lands while close() stops the client.
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub/mqtt-stub -I test/host-stub -I src \
//       -o /tmp/mqtt-service-test test/host-stub/mqtt-service-test.cpp && /tmp/mqtt-service-test

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

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
bool handleCommand(const String &, bool) { return true; }
void consoleDispatchAsync(const char *, size_t, bool) {}
void logCmdResult(bool, const char *, int64_t) {}
const std::string &getHostname(bool) { static std::string h{"test"}; return h; }

#include "../../src/tele/mqtt.cpp"

time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;

// --- esp-mqtt stub ---
struct esp_mqtt_client {
    esp_event_handler_t h = nullptr;
    void *arg = nullptr;
    bool started = false;
};

static int g_live = 0, g_initFail = 0, g_startFail = 0, g_subs = 0;
static bool g_connectInFlight = false; // stop() waits for this connect, which then succeeds

const char *esp_err_to_name(esp_err_t) { return "ERR"; }
uint32_t esp_get_free_heap_size() { return 8000; }
int64_t esp_timer_get_time() { return 0; }
TaskHandle_t xTaskGetCurrentTaskHandle() { return nullptr; }

static void deliver(esp_mqtt_client *c, esp_mqtt_event_id_t id) {
    esp_mqtt_event_t ev{};
    ev.event_id = id;
    ev.client = c;
    if (c->h) c->h(c->arg, "MQTT_EVENTS", id, &ev);
}

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *) {
    if (g_initFail > 0) { --g_initFail; return nullptr; }
    ++g_live;
    return new esp_mqtt_client;
}
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t c, esp_mqtt_event_id_t, esp_event_handler_t h, void *a) {
    c->h = h;
    c->arg = a;
    return ESP_OK;
}
esp_err_t esp_mqtt_client_unregister_event(esp_mqtt_client_handle_t c, esp_mqtt_event_id_t, esp_event_handler_t) {
    c->h = nullptr;
    return ESP_OK;
}
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t c) {
    if (g_startFail > 0) { --g_startFail; return ESP_FAIL; }
    c->started = true;
    return ESP_OK;
}
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t c) {
    if (!c->started) return ESP_FAIL;
    if (g_connectInFlight) { // the task holds the API lock while connecting; CONNECTED is dispatched first
        g_connectInFlight = false;
        deliver(c, MQTT_EVENT_CONNECTED);
    }
    c->started = false; // the task exits without a DISCONNECTED event
    return ESP_OK;
}
esp_err_t esp_mqtt_client_disconnect(esp_mqtt_client_handle_t) { return ESP_OK; }
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t c) {
    --g_live;
    delete c;
    return ESP_OK;
}
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t, const char *, int) { return ++g_subs; }
static std::string g_pubData; // last published payload, as esp-mqtt sends it (len 0 = strlen)
int esp_mqtt_client_publish(esp_mqtt_client_handle_t, const char *, const char *d, int len, int, int) {
    g_pubData.assign(d, len ? (size_t) len : std::strlen(d));
    return 0;
}

// --- test helpers ---
namespace {

struct PathPeek : MqttService {
    static void set(MqttService &m, const char *p) { m.*(&PathPeek::_confPath) = p; }
};

const char *kConf = "/tmp/_fugu_mqtt_service_test.conf";
int g_bmsSubs = 0;      // preStart runs
bool g_preThrow = false;

void setup() {
    FILE *f = std::fopen(kConf, "w");
    std::fputs("broker_uri=mqtt://127.0.0.1:1883\n", f);
    std::fclose(f);
    PathPeek::set(MQTT, kConf);
    MQTT.preStart = [](const ConfFile &) {
        ++g_bmsSubs;
        MQTT.subscribeTopic("bms/cell_max", [](const char *, int) {});
        if (g_preThrow) throw std::runtime_error("preStart");
        MQTT.subscribeTopic("bms/ibat", [](const char *, int) {});
    };
}

void connect() { deliver(MQTT.client, MQTT_EVENT_CONNECTED); }

// healthy, connected service
void fresh() {
    MQTT.stop();
    g_initFail = g_startFail = 0;
    g_connectInFlight = g_preThrow = false;
    assert(MQTT.start());
    connect();
    assert(MQTT.isConnected() && MQTT.statusDetail() == "connected");
}

// a restart that hits a successful in-flight connect (2026-11-16): the late CONNECTED must not stick
void test_restart_races_connect() {
    fresh();
    MQTT.client->started = true;
    g_connectInFlight = true;
    int subs = g_bmsSubs;
    bool ok = MQTT.restart();
    std::printf("restart racing a connect: %s, state %s, detail '%s'\n", ok ? "ok" : "FAILED",
                stateStr(MQTT.state()), MQTT.statusDetail().c_str());
    assert(ok && MQTT.state() == ServiceState::Running);
    assert(g_bmsSubs == subs + 1); // preStart re-subscribed
    assert(!MQTT.isConnected());   // the new client has not connected yet
    connect();
    assert(MQTT.isConnected());
}

// (a) client init fails once (low heap), then heap recovers: the retry brings it back
void test_init_fails_once() {
    fresh();
    g_initFail = 1;
    assert(!MQTT.restart());
    assert(MQTT.state() == ServiceState::Failed && !MQTT.client && MQTT.statusDetail().empty());
    assert(MQTT.configured()); // the network guard still counts it as down
    ServiceManager mgr;
    mgr.registerService(&MQTT);
    assert(mgr.retryFailedNetworkServices() == 1);
    assert(MQTT.state() == ServiceState::Running && MQTT.client);
    connect();
    assert(MQTT.isConnected() && g_live == 1);
}

// (b) start fails after the flag was set: CONNECTED lands during close(), then the task cannot start
void test_start_fails_after_flag_set() {
    fresh();
    g_connectInFlight = true;
    g_startFail = 1;
    assert(!MQTT.restart());
    std::printf("after failed start: state %s, detail '%s', client %p\n", stateStr(MQTT.state()),
                MQTT.statusDetail().c_str(), (void *) MQTT.client);
    assert(MQTT.statusDetail().empty() && !MQTT.isConnected() && g_live == 0);
    bool ok = MQTT.start(); // what the 30 s retry does
    std::printf("retry: %s\n", ok ? "ok" : "FAILED");
    assert(ok && MQTT.state() == ServiceState::Running);
}

// (c) preStart throws half-way: Failed, nothing allocated, still counted as configured, retryable
void test_prestart_throws() {
    fresh();
    g_preThrow = true;
    assert(!MQTT.restart());
    assert(MQTT.state() == ServiceState::Failed && !MQTT.client && g_live == 0);
    std::printf("preStart threw: configured=%d\n", MQTT.configured());
    assert(MQTT.configured());
    g_preThrow = false;
    assert(MQTT.start() && MQTT.client);
    connect();
    assert(MQTT.isConnected());
}

// (e) the incident state: flag true, client null, state Failed. restart() must succeed.
void test_incident_state_restart() {
    fresh();
    g_initFail = 1;
    assert(!MQTT.restart());                 // Failed, client null
    esp_mqtt_client stale;                   // a late CONNECTED from the destroyed client sets the flag
    esp_mqtt_event_t ev{};
    ev.event_id = MQTT_EVENT_CONNECTED;
    ev.client = &stale;
    MQTT._handleEvent("MQTT_EVENTS", MQTT_EVENT_CONNECTED, &ev);
    std::printf("incident state: state %s, detail '%s', client %p\n", stateStr(MQTT.state()),
                MQTT.statusDetail().c_str(), (void *) MQTT.client);
    const bool detailOk = MQTT.statusDetail().empty() && !MQTT.isConnected(); // no "connected" when Failed
    bool ok = MQTT.restart();
    std::printf("svc restart mqtt: %s\n", ok ? "ok" : "FAILED");
    assert(detailOk);
    assert(ok && MQTT.state() == ServiceState::Running && MQTT.client);
}

// a Failed service whose tick threw keeps its client; it must not report "connected"
void test_failed_with_live_client_not_connected() {
    fresh();
    MQTT.tickHook = [] { throw std::runtime_error("tick"); };
    setWallClockUs(wallClockUs() + 10000000); // past the 3 s tick throttle
    MQTT.tick();
    MQTT.tickHook = nullptr;
    assert(MQTT.state() == ServiceState::Failed);
    assert(MQTT.client && MQTT.statusDetail().empty() && !MQTT.isConnected());
}

// a dropped DISCONNECTED (no heap for the event copy) is corrected by the next BEFORE_CONNECT
void test_before_connect_clears_flag() {
    fresh();
    deliver(MQTT.client, MQTT_EVENT_BEFORE_CONNECT);
    assert(!MQTT.isConnected());
}

// a binary payload with an embedded NUL arrives with its full length; text still uses strlen
void test_publish_binary_len() {
    fresh();
    const char bin[] = {0, 2, 'a', 0, 'b'};
    assert(MQTT.publish("t", bin, sizeof bin, 0, false));
    assert(g_pubData == std::string(bin, sizeof bin));
    assert(MQTT.publish("t", "txt", 0, false) && g_pubData == "txt");
    deliver(MQTT.client, MQTT_EVENT_DISCONNECTED);
    assert(!MQTT.publish("t", bin, sizeof bin, 0, false));
}

} // namespace

int main(int argc, char **argv) {
    setup();
    // optional argv[1]: run only the test whose name contains it
    const struct { const char *name; void (*fn)(); } tests[] = {
            {"restart_races_connect", test_restart_races_connect},
            {"init_fails_once", test_init_fails_once},
            {"start_fails_after_flag_set", test_start_fails_after_flag_set},
            {"prestart_throws", test_prestart_throws},
            {"incident_state_restart", test_incident_state_restart},
            {"failed_with_live_client", test_failed_with_live_client_not_connected},
            {"before_connect_clears_flag", test_before_connect_clears_flag},
            {"publish_binary_len", test_publish_binary_len},
    };
    for (auto &t: tests)
        if (argc < 2 || std::strstr(t.name, argv[1])) {
            std::printf("--- %s\n", t.name);
            t.fn();
        }
    MQTT.stop();
    assert(g_live == 0);
    std::remove(kConf);
    std::printf("mqtt-service-test: all passed\n");
    return 0;
}
