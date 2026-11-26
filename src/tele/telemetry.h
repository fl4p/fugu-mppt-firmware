#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif


//#include "../adc/sampling.h"
//#include "../store.h"
#include <string>

class String; // Arduino String (handleCommand); avoid dragging <Arduino.h> into every includer


extern bool timeSynced;


const char *getChipId();

const std::string &getHostname(bool reload = false);

std::string getDeviceId();

bool add_ap(const std::string &ssid, const std::string &psk);

void wifi_load_conf();

void connect_wifi_async();

// Tear down WiFi. Ends mDNS first (while the netif is still up) so its tcpip-thread group-leave
// doesn't run against a freed netif when `off` deinits the stack. `off` mirrors WiFi.disconnect().
void disconnect_wifi(bool off);

bool wait_for_wifi();

void wifi_retry_now(); // skip the backoff after a failed attempt

void wifiLoop(bool connect = false);


#include "line_protocol.h"

#ifdef CONFIG_FUGU_WITH_TELE_BINARY
#include <variant>
#include "sym_line_protocol.h"

// Telemetry wire is chosen at boot from tele.conf::binary (0=text, 1=binary).
// TelePoint wraps both builders in a variant and forwards the shared API so
// producers write the same code regardless of the wire format; the binary
// branch needs g_symtab (shared interning table per device).
extern SymbolTable g_symtab;
extern bool g_teleBinary;     // set by TelemetryService::onStart from tele.conf

class TelePoint {
    using V = std::variant<LineProtocol, BinaryLineProtocol>;
    V v_;
public:
    explicit TelePoint(const char *measurement)
        : v_(g_teleBinary
                 ? V(std::in_place_type<BinaryLineProtocol>, g_symtab, measurement)
                 : V(std::in_place_type<LineProtocol>, measurement)) {}

    void addTag(const char *k, const char *v) { std::visit([&](auto &p) { p.addTag(k, v); }, v_); }
    void addField(const char *k, float v, int dp = 2) { std::visit([&](auto &p) { p.addField(k, v, dp); }, v_); }
    void addField(const char *k, int v) { std::visit([&](auto &p) { p.addField(k, v); }, v_); }
    void addField(const char *k, bool v) { std::visit([&](auto &p) { p.addField(k, v); }, v_); }
    void setTimeMs() { std::visit([](auto &p) { p.setTimeMs(); }, v_); }
    bool hasTime() const { return std::visit([](const auto &p) { return p.hasTime(); }, v_); }
    std::string takeWire() { return std::visit([](auto &p) { return p.takeWire(); }, v_); }
    bool isBinary() const { return v_.index() == 1; }
};

#else
constexpr bool g_teleBinary = false;  // text-only build (no CONFIG_FUGU_WITH_TELE_BINARY)

class TelePoint : public LineProtocol {
public:
    using LineProtocol::LineProtocol;
    bool isBinary() const { return false; }
};
#endif

inline TelePoint makeTelePoint(const char *measurement) { return TelePoint(measurement); }

void teleLoadWireConf();  // re-read tele.conf::binary (cache for the hot path; binary wire = tamp)

void telemetryAddPoint(TelePoint &p, uint16_t maxQueue = 40);

class ADC_Sampler;
struct Sensor;

void dcdcDataChanged(const ADC_Sampler &dcdc, const Sensor &sensor);

//void onTelnetConnect(String ip);
//void onTelnetDisconnect(String ip);


