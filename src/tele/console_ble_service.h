#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif


// BleConsoleService: BLE (NimBLE NUS) serial console, the fallback transport when Wi-Fi is down.
// Compiled only with -DCONFIG_FUGU_WITH_BLE; otherwise this header is empty. Disabled by default so the BLE
// radio only comes up on boards that opt in via ble.conf. No Wi-Fi precondition.

#ifdef CONFIG_FUGU_WITH_BLE

#include <algorithm>

#include "service.h"
#include "console_ble.h"
#include "tele_ble.h"
#include "util.h"
#ifdef CONFIG_FUGU_WITH_NETW
#include "tele/telemetry.h"
#else
#include "storage/key-value.h"
extern KeyValueStorage nvs;
#endif

class BleConsoleService : public Service {
public:
    BleConsoleService() : Service("ble", "/littlefs/conf/ble.conf", /*requiresNetwork*/ false,
                                  /*enabledDefault*/ false) { // it exposes the console, default off
    }

    std::string statusDetail() const override {
        if (!bleConsoleConnected()) return "";
        return teleBleStreaming() ? "connected, tele stream" : "connected";
    }

protected:
    bool onStart() override {
        ConfFile c{"/littlefs/conf/ble.conf", /*no_warn_if_not_open*/ true};
#ifdef CONFIG_FUGU_WITH_NETW
        std::string hn = getHostname();
#else
        nvs.open();
        std::string hn = nvs.readString("hostname", "fugu");
        nvs.close();
#endif
        bleConsoleBegin(        hn.starts_with("fugu-") ? hn : ("fugu-" + hn),
                        c.getString("ble_security", "justworks"),
                        (uint32_t) c.getLong("ble_passkey", 0),
                        (uint32_t) std::max(c.getLong("pair_window_s", 300), 0L));
        return true;
    }

    // Free the tele stream synchronously: bleConsoleEnd()'s disconnect only *requests* the stop,
    // and a stopped service never ticks again to consume it (runs on the net loop, so it's safe).
    void onStop() override { teleBleSetStreaming(false); bleConsoleEnd(); }
    void onTick() override { bleConsoleLoop(wallClockMs()); }
};

inline BleConsoleService bleConsoleService;

#endif // CONFIG_FUGU_WITH_BLE
