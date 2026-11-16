// Host-side tests for src/etc/net_guard.h (NetGuard, LogLimiter) and
// ServiceManager::retryFailedNetworkServices().
//
// Build & run:
//   clang++ -std=gnu++20 -fexceptions -I test/host-stub -I src \
//       -o /tmp/net-guard-test test/host-stub/net-guard-test.cpp && /tmp/net-guard-test

#include <cassert>
#include <cstdio>
#include <functional>

#include "../../src/service.h"
#include "../../src/etc/net_guard.h"

namespace {

constexpr uint32_t S = 1000, MIN = 60 * S;
constexpr uint32_t HEAP_OK = 29 * 1024, HEAP_LOW = 4 * 1024;

class FakeService : public Service {
public:
    int startCount = 0, stopCount = 0;
    bool startResult = true;

    FakeService(const char *name, bool reqNet, bool en = true)
        : Service(name, "/tmp/_fugu_test_nonexistent.conf", reqNet, en) {}

    bool onStart() override { ++startCount; return startResult; }
    void onStop() override { ++stopCount; }
};

NetGuardIn base(uint32_t now) {
    NetGuardIn in{};
    in.nowMs = now;
    in.freeHeap = HEAP_OK;
    in.largest = 15 * 1024;
    in.wifiUp = true;
    in.mqttActive = true;
    in.mqttRunning = true;
    in.mqttConnected = false;
    in.rebootOk = true;
    return in;
}

struct Count { int retry = 0, restart = 0, reboot = 0; uint32_t firstRestart = 0, firstReboot = 0; };

// Steps once per second over [t0, t0+durMs), with `mod` adjusting the inputs.
Count run(NetGuard &g, uint32_t t0, uint32_t durMs, const std::function<void(NetGuardIn &)> &mod = {}) {
    Count c;
    for (uint32_t t = 0; t < durMs; t += S) {
        auto in = base(t0 + t);
        if (mod) mod(in);
        uint8_t a = g.step(in);
        if (a & NG_RETRY_FAILED) ++c.retry;
        if (a & NG_RESTART_MQTT) { if (!c.restart++) c.firstRestart = t; }
        if (a & NG_REBOOT) { if (!c.reboot++) c.firstReboot = t; }
    }
    return c;
}

void test_failed_service_retried() {
    ServiceManager mgr;
    FakeService net{"net", true}, loc{"loc", false}, off{"off", true, false}, ok{"ok", true};
    mgr.registerService(&net);
    mgr.registerService(&loc);
    mgr.registerService(&off);
    mgr.registerService(&ok);
    net.startResult = false;
    loc.startResult = false;
    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed && loc.state() == ServiceState::Failed);
    assert(ok.state() == ServiceState::Running);

    NetGuard g;
    int retries = 0;
    for (uint32_t t = 0; t < 29 * S; t += S)
        if (g.step(base(t)) & NG_RETRY_FAILED) ++retries;
    assert(retries == 0); // not before 30 s

    // Wi-Fi down: no retry however long
    for (uint32_t t = 29 * S; t < 200 * S; t += S) {
        auto in = base(t);
        in.wifiUp = false;
        assert(!(g.step(in) & NG_RETRY_FAILED));
    }
    // Wi-Fi back: first retry 30 s later
    uint32_t t = 200 * S, at = 0;
    for (; t < 300 * S; t += S)
        if (g.step(base(t)) & NG_RETRY_FAILED) { at = t; break; }
    assert(at == 229 * S); // 30 s after the last Wi-Fi-down sample

    assert(mgr.retryFailedNetworkServices() == 1);
    assert(net.startCount == 2 && net.stopCount == 0); // a Failed service is started without onStop
    assert(loc.startCount == 1);                      // not a network service
    assert(off.startCount == 0);                      // disabled
    assert(ok.startCount == 1);                       // Running: untouched

    net.startResult = true;
    assert(mgr.retryFailedNetworkServices() == 1);
    assert(net.state() == ServiceState::Running);
    assert(mgr.retryFailedNetworkServices() == 0);
    assert(net.startCount == 3);

    // Stopped but enabled (e.g. a restart whose stop ran but whose start never did) is started too
    net.stop();
    assert(mgr.retryFailedNetworkServices() == 1 && net.state() == ServiceState::Running);
    off.stop();
    assert(mgr.retryFailedNetworkServices() == 0); // disabled stays Stopped
}

void test_retry_period() {
    NetGuard g;
    auto c = run(g, 0, 10 * MIN);
    assert(c.retry == 19); // at 30, 60, ... 570 s
}

void test_mqtt_restart_after_120s() {
    NetGuard g;
    auto c = run(g, 0, 121 * S);
    assert(c.restart == 0);
    c = run(g, 121 * S, 2 * S);
    assert(c.restart == 1 && g.reinits == 1);
    // then the gap doubles: 240 s, 480 s, ...
    c = run(g, 123 * S, 238 * S);
    assert(c.restart == 0);
    c = run(g, 361 * S, 2 * S);
    assert(c.restart == 1 && c.firstRestart == 1 * S);
    c = run(g, 363 * S, 481 * S);
    assert(c.restart == 1 && c.firstRestart == 480 * S); // at 843 s
}

// a connect resets the backoff
void test_mqtt_restart_backoff_resets_on_connect() {
    NetGuard g;
    auto c = run(g, 0, 13 * MIN);
    assert(c.restart == 2);
    c = run(g, 13 * MIN, 1 * S, [](NetGuardIn &in) { in.mqttConnected = true; });
    c = run(g, 13 * MIN + S, 122 * S);
    assert(c.restart == 1 && c.firstRestart == 120 * S); // 121 s after the connect
}

// a Failed service is restarted by the 30 s retry, not by NG_RESTART_MQTT; the gap counts from Running
void test_mqtt_restart_waits_after_failed() {
    NetGuard g;
    auto c = run(g, 0, 5 * MIN, [](NetGuardIn &in) { in.mqttRunning = false; });
    assert(c.restart == 0);
    c = run(g, 5 * MIN, 120 * S);
    assert(c.restart == 0);
    c = run(g, 7 * MIN, 2 * S);
    assert(c.restart == 1);
}

void test_mqtt_no_restart_wifi_down_or_disabled() {
    NetGuard g;
    auto c = run(g, 0, 30 * MIN, [](NetGuardIn &in) { in.wifiUp = false; });
    assert(c.restart == 0 && c.reboot == 0 && c.retry == 0);
    // Wi-Fi back: the 120 s count starts then
    c = run(g, 30 * MIN, 120 * S);
    assert(c.restart == 0);
    c = run(g, 32 * MIN, 2 * S);
    assert(c.restart == 1);
}

void test_mqtt_flapping_wifi_resets_count() {
    NetGuard g;
    // Wi-Fi drops for 1 s every 100 s: MQTT is never "down with Wi-Fi up" for 120 s
    auto c = run(g, 0, 60 * MIN, [](NetGuardIn &in) { if ((in.nowMs / S) % 100 == 99) in.wifiUp = false; });
    assert(c.restart == 0);
}

void test_mqtt_connected_or_unconfigured_no_action() {
    NetGuard g;
    auto c = run(g, 0, 60 * MIN, [](NetGuardIn &in) { in.mqttConnected = true; in.freeHeap = HEAP_LOW; });
    assert(c.restart == 0 && c.reboot == 0);
    NetGuard g2;
    c = run(g2, 0, 60 * MIN, [](NetGuardIn &in) { in.mqttActive = false; in.mqttRunning = false; in.freeHeap = HEAP_LOW; });
    assert(c.restart == 0 && c.reboot == 0);
}

void test_failed_mqtt_not_restarted_but_counts_for_reboot() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { in.mqttRunning = false; in.freeHeap = HEAP_LOW; });
    assert(c.restart == 0);
    assert(c.reboot == 1);
}

void test_broker_down_healthy_heap_reboots_after_15min() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN);
    assert(c.restart == 3); // 121, 362, 843 s
    assert(c.reboot == 0);
    c = run(g, 15 * MIN, 2 * S);
    assert(c.reboot == 1 && c.firstReboot == 1 * S); // 901 s
    // heap at the threshold is not low: the 15 min rule, not the 10 min one
    NetGuard g2;
    c = run(g2, 0, 15 * MIN + 2 * S, [](NetGuardIn &in) { in.freeHeap = NetGuard::kHeapLow; });
    assert(c.reboot == 1 && c.firstReboot == 901 * S);
}

// (d) broker down for 3 h (8 h): esp-mqtt keeps reconnecting, the guard re-inits with backoff and
// reboots once; the persisted hold then blocks another reboot for 6 h of uptime.
struct Episode { int restart = 0, reboot = 0, boots = 1; };

Episode brokerDown(uint32_t totalMs, uint32_t connectAtMs = UINT32_MAX) {
    Episode e;
    bool held = false; // the RTC flag
    uint32_t t = 0;
    while (t < totalMs) {
        NetGuard g; // a fresh boot
        uint32_t boot = t;
        for (; t < totalMs; t += S) {
            auto in = base(t - boot);
            in.rebootOk = t - boot > 5 * MIN;
            in.rebootHeld = held;
            if (t >= connectAtMs) { in.mqttConnected = true; held = false; }
            uint8_t a = g.step(in);
            if (a & NG_RESTART_MQTT) ++e.restart;
            if (a & NG_REBOOT) { ++e.reboot; held = true; t += 10 * S; break; } // ~10 s to reboot
        }
        if (t < totalMs) ++e.boots;
    }
    return e;
}

void test_broker_down_3h_bounded() {
    auto e = brokerDown(180 * MIN);
    std::printf("broker down 3 h: %d re-inits, %d reboots\n", e.restart, e.reboot);
    assert(e.reboot == 1);
    assert(e.restart == 3 + 8); // 2/6/14 min, reboot at 15 min; then 2/6/14/30/60/90/120/150 min
    e = brokerDown(8 * 60 * MIN);
    std::printf("broker down 8 h: %d re-inits, %d reboots\n", e.restart, e.reboot);
    assert(e.reboot == 2); // 15 min, then 6 h after that boot
    // a connect clears the hold: the next outage may reboot again at once
    e = brokerDown(60 * MIN, 30 * MIN);
    assert(e.reboot == 1);
}

// the 2026-11-16 state after the fix: Failed (start keeps failing), configured, not connected,
// healthy heap: retried every 30 s and rebooted at 15 min
void test_failed_mqtt_healthy_heap_reboots() {
    NetGuard g;
    auto c = run(g, 0, 16 * MIN, [](NetGuardIn &in) { in.mqttRunning = false; in.freeHeap = 50 * 1024; in.largest = 25 * 1024; });
    assert(c.retry == 31 && c.restart == 0);
    assert(c.reboot == 1 && c.firstReboot == 901 * S);
}

void test_low_heap_reboots_once() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; });
    assert(c.reboot == 1);
    assert(c.firstReboot == 601 * S);
}

void test_low_heap_must_persist() {
    NetGuard g;
    // healthy heap until 9.5 min, then low: the 60 s hold delays the reboot to ~10.5 min
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { if (in.nowMs < 570 * S) in.freeHeap = HEAP_OK; else in.freeHeap = HEAP_LOW; });
    assert(c.reboot == 1 && c.firstReboot == 629 * S);
    // a short dip (30 s every 5 min) does not bring the low-heap reboot forward
    NetGuard g2;
    c = run(g2, 0, 15 * MIN + 2 * S, [](NetGuardIn &in) { in.freeHeap = (in.nowMs / S) % 300 < 30 ? HEAP_LOW : HEAP_OK; });
    assert(c.reboot == 1 && c.firstReboot == 901 * S);
}

void test_no_reboot_when_not_ok_or_heap_unknown() {
    NetGuard g;
    auto c = run(g, 0, 60 * MIN, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; in.rebootOk = false; });
    assert(c.reboot == 0);
    // allowed again later: fires at once since the outage is long enough
    c = run(g, 60 * MIN, 2 * S, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; });
    assert(c.reboot == 1 && c.firstReboot == 0);

    NetGuard g2;
    c = run(g2, 0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = NetGuard::kHeapUnknown; });
    assert(c.reboot == 0);
}

// held: no reboot for 6 h of uptime, low heap or not; then allowed
void test_reboot_held() {
    NetGuard g;
    auto c = run(g, 0, 6 * 60 * MIN, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; in.rebootHeld = true; });
    assert(c.reboot == 0 && c.restart > 0);
    c = run(g, 6 * 60 * MIN, 2 * S, [](NetGuardIn &in) { in.rebootHeld = true; });
    assert(c.reboot == 1 && c.firstReboot == 0);
}

// the 2026-11 outage: free above 12 KB but no 8 KB block for the mqtt task stack
void test_fragmented_heap_reboots_once() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = 13700; in.largest = 7600; });
    assert(c.reboot == 1 && c.firstReboot == 601 * S);
    // fragmented, but MQTT connected
    NetGuard g2;
    c = run(g2, 0, 60 * MIN, [](NetGuardIn &in) { in.freeHeap = 13700; in.largest = 7600; in.mqttConnected = true; });
    assert(c.reboot == 0);
}

void test_normal_heap_broker_down_no_early_reboot() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = 28 * 1024; in.largest = 11200; });
    assert(c.reboot == 0);
    // block exactly at the threshold is not low
    NetGuard g2;
    c = run(g2, 0, 15 * MIN, [](NetGuardIn &in) { in.largest = NetGuard::kBlockLow; });
    assert(c.reboot == 0);
}

void test_largest_unknown_no_early_reboot() {
    NetGuard g;
    auto c = run(g, 0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = 13700; in.largest = NetGuard::kHeapUnknown; });
    assert(c.reboot == 0);
}

void test_reboot_wifi_down_resets() {
    NetGuard g;
    // low heap, Wi-Fi drops 1 s every 5 min: never 10 min of "MQTT down with Wi-Fi up"
    auto c = run(g, 0, 60 * MIN, [](NetGuardIn &in) {
        in.freeHeap = HEAP_LOW;
        if ((in.nowMs / S) % 300 == 299) in.wifiUp = false;
    });
    assert(c.reboot == 0);
}

void test_reboot_refused_rearms_after_10min() {
    NetGuard g;
    auto c = run(g, 0, 25 * MIN, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; });
    assert(c.reboot == 2); // 601 s and 1202 s
}

void test_clock_wrap() {
    NetGuard g;
    const uint32_t t0 = UINT32_MAX - 60 * S;
    auto c = run(g, t0, 121 * S);
    assert(c.restart == 0);
    c = run(g, t0 + 121 * S, 2 * S);
    assert(c.restart == 1);
    NetGuard g2;
    c = run(g2, t0, 15 * MIN, [](NetGuardIn &in) { in.freeHeap = HEAP_LOW; });
    assert(c.reboot == 1 && c.firstReboot == 601 * S);
}

void test_log_limiter() {
    LogLimiter l{5, 10000};
    int n = 0;
    for (uint32_t t = 0; t < 100; ++t) n += l.allow(t);
    assert(n == 5);
    n = 0;
    for (uint32_t t = 100; t < 60100; t += 100) n += l.allow(t);
    assert(n == 5); // 10100, 20100, ..., 50100
    LogLimiter w{1, 60000};
    assert(w.allow(0) && !w.allow(59999) && w.allow(60000));
}

} // namespace

int main() {
    test_failed_service_retried();
    test_retry_period();
    test_mqtt_restart_after_120s();
    test_mqtt_restart_backoff_resets_on_connect();
    test_mqtt_restart_waits_after_failed();
    test_mqtt_no_restart_wifi_down_or_disabled();
    test_mqtt_flapping_wifi_resets_count();
    test_mqtt_connected_or_unconfigured_no_action();
    test_failed_mqtt_not_restarted_but_counts_for_reboot();
    test_broker_down_healthy_heap_reboots_after_15min();
    test_broker_down_3h_bounded();
    test_failed_mqtt_healthy_heap_reboots();
    test_reboot_held();
    test_low_heap_reboots_once();
    test_low_heap_must_persist();
    test_no_reboot_when_not_ok_or_heap_unknown();
    test_fragmented_heap_reboots_once();
    test_normal_heap_broker_down_no_early_reboot();
    test_largest_unknown_no_early_reboot();
    test_reboot_wifi_down_resets();
    test_reboot_refused_rearms_after_10min();
    test_clock_wrap();
    test_log_limiter();
    printf("net-guard-test: all passed\n");
    return 0;
}
