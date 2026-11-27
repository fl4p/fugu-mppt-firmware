// Host-side unit tests for src/service.h ServiceManager / Service state machine.
//
// Build & run:
//   clang++ -std=gnu++17 -fexceptions -I test/host-stub -I src \
//       -o /tmp/service-test test/host-stub/service-test.cpp && /tmp/service-test
//
// Covers the cases discussed for boards that boot without wifi.conf:
//   - startEnabledAtBoot(networkUp=false) leaves requiresNetwork services Stopped (not Failed)
//   - startEnabledAtBoot(networkUp=true)  starts everything enabled
//   - startEnabledNetworkServices() self-heals Stopped/Failed network services on a WiFi-up edge
//   - startEnabledNetworkServices() does not re-enter Running services or touch local ones
//   - onStart()==false or throwing => Failed; stop() clears Failed back to Stopped
//   - disabled services are never started; tickAll only ticks Running

#include <cassert>
#include <cstdio>
#include <stdexcept>

#include "../../src/service.h"

namespace {

class FakeService : public Service {
public:
    int startCount = 0, stopCount = 0, tickCount = 0;
    bool startResult = true;
    bool throwOnStart = false;

    FakeService(const char *name, bool reqNet, bool enabledDefault = true)
        : Service(name, "/tmp/_fugu_test_nonexistent.conf", reqNet, enabledDefault) {}

    bool onStart() override {
        ++startCount;
        if (throwOnStart) throw std::runtime_error("fake throw");
        return startResult;
    }
    void onStop() override { ++stopCount; }
    void onTick() override { ++tickCount; }
};

void test_boot_skips_network_when_wifi_down() {
    ServiceManager mgr;
    FakeService net{"net", /*reqNet*/ true};
    FakeService loc{"loc", /*reqNet*/ false};
    mgr.registerService(&net);
    mgr.registerService(&loc);

    mgr.startEnabledAtBoot(/*networkUp*/ false);

    assert(net.state() == ServiceState::Stopped);
    assert(net.startCount == 0);
    assert(loc.state() == ServiceState::Running);
    assert(loc.startCount == 1);
}

void test_boot_starts_everything_when_wifi_up() {
    ServiceManager mgr;
    FakeService net{"net", true};
    FakeService loc{"loc", false};
    mgr.registerService(&net);
    mgr.registerService(&loc);

    mgr.startEnabledAtBoot(/*networkUp*/ true);

    assert(net.state() == ServiceState::Running);
    assert(loc.state() == ServiceState::Running);
}

void test_selfheal_starts_pending_network_only() {
    ServiceManager mgr;
    FakeService net{"net", true};
    FakeService loc{"loc", false};
    mgr.registerService(&net);
    mgr.registerService(&loc);

    mgr.startEnabledAtBoot(false);
    assert(net.state() == ServiceState::Stopped);
    assert(loc.state() == ServiceState::Running);

    int locStarts = loc.startCount;
    mgr.startEnabledNetworkServices();

    assert(net.state() == ServiceState::Running);
    assert(net.startCount == 1);
    assert(loc.startCount == locStarts);
}

void test_selfheal_does_not_restart_running() {
    ServiceManager mgr;
    FakeService net{"net", true};
    mgr.registerService(&net);
    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Running);
    assert(net.startCount == 1);

    mgr.startEnabledNetworkServices();
    assert(net.startCount == 1);  // no second start
}

void test_selfheal_retries_failed() {
    ServiceManager mgr;
    FakeService net{"net", true};
    net.startResult = false;
    mgr.registerService(&net);

    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed);

    net.startResult = true;
    mgr.startEnabledNetworkServices();
    assert(net.state() == ServiceState::Running);
    assert(net.startCount == 2);
}

void test_onstart_exception_is_failure() {
    ServiceManager mgr;
    FakeService net{"net", true};
    net.throwOnStart = true;
    mgr.registerService(&net);

    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed);
}

void test_stop_clears_failed() {
    ServiceManager mgr;
    FakeService net{"net", true};
    net.startResult = false;
    mgr.registerService(&net);

    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed);

    net.stop();
    assert(net.state() == ServiceState::Stopped);

    net.startResult = true;
    assert(net.start());
    assert(net.state() == ServiceState::Running);
}

void test_disabled_services_never_start() {
    ServiceManager mgr;
    FakeService disabledNet{"dn", true, /*enabled*/ false};
    FakeService disabledLoc{"dl", false, /*enabled*/ false};
    mgr.registerService(&disabledNet);
    mgr.registerService(&disabledLoc);

    mgr.startEnabledAtBoot(true);
    assert(disabledNet.startCount == 0);
    assert(disabledLoc.startCount == 0);

    mgr.startEnabledNetworkServices();
    assert(disabledNet.startCount == 0);
}

void test_tick_only_running() {
    ServiceManager mgr;
    FakeService a{"a", false};                // will Run
    FakeService b{"b", true};                 // will Fail (wifi down at boot)
    FakeService c{"c", false, /*en*/ false};  // disabled -> Stopped
    mgr.registerService(&a);
    mgr.registerService(&b);
    mgr.registerService(&c);

    b.startResult = false;
    mgr.startEnabledAtBoot(true);  // b actively fails (startResult false), a runs, c skipped
    assert(a.state() == ServiceState::Running);
    assert(b.state() == ServiceState::Failed);
    assert(c.state() == ServiceState::Stopped);

    mgr.tickAll();
    assert(a.tickCount == 1);
    assert(b.tickCount == 0);
    assert(c.tickCount == 0);
}

void test_stopall_stops_running_only() {
    ServiceManager mgr;
    FakeService a{"a", false};
    FakeService b{"b", true};
    b.startResult = false;
    mgr.registerService(&a);
    mgr.registerService(&b);

    mgr.startEnabledAtBoot(true);  // a Running, b Failed
    mgr.stopAll();

    assert(a.state() == ServiceState::Stopped);
    assert(a.stopCount == 1);
    assert(b.state() == ServiceState::Stopped);  // stop() clears Failed
    assert(b.stopCount == 1);
}

// start fails once (e.g. no heap for the client), then succeeds: the periodic retry restores it
void test_retry_after_failed_start() {
    ServiceManager mgr;
    FakeService net{"net", true};
    mgr.registerService(&net);
    net.startResult = false;
    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed);
    net.startResult = true;
    assert(mgr.startEnabledNetworkServices() == 1);
    assert(net.state() == ServiceState::Running);
}

// a throwing start leaves it Failed and retryable; restart() of a Failed service works
void test_throw_then_restart() {
    ServiceManager mgr;
    FakeService net{"net", true};
    mgr.registerService(&net);
    net.throwOnStart = true;
    mgr.startEnabledAtBoot(true);
    assert(net.state() == ServiceState::Failed);
    net.throwOnStart = false;
    assert(net.restart() && net.state() == ServiceState::Running);
    assert(net.stopCount == 1);
}

// enabled but Stopped network services are retried too; disabled ones and local ones are not
void test_retry_starts_stopped_enabled() {
    ServiceManager mgr;
    FakeService net{"net", true}, off{"off", true, false}, loc{"loc", false};
    mgr.registerService(&net);
    mgr.registerService(&off);
    mgr.registerService(&loc);
    mgr.startEnabledAtBoot(false); // net stays Stopped
    assert(net.state() == ServiceState::Stopped);
    int n = mgr.startEnabledNetworkServices();
    std::printf("retry with an enabled Stopped service: %d started\n", n);
    assert(n == 1 && net.state() == ServiceState::Running);
    assert(off.startCount == 0 && loc.startCount == 1);
}

}  // namespace

int main() {
    test_boot_skips_network_when_wifi_down();
    test_boot_starts_everything_when_wifi_up();
    test_selfheal_starts_pending_network_only();
    test_selfheal_does_not_restart_running();
    test_selfheal_retries_failed();
    test_onstart_exception_is_failure();
    test_stop_clears_failed();
    test_disabled_services_never_start();
    test_tick_only_running();
    test_stopall_stops_running_only();
    test_retry_after_failed_start();
    test_throw_then_restart();
    test_retry_starts_stopped_enabled();

    std::printf("OK: all service tests passed\n");
    return 0;
}
