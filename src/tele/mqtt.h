#pragma once

#include <functional>
#include <string>

#include "../conf.h"
#include "../service.h"
#include "../util.h"
#include "mqtt_client.h"

class MqttService : public Service {
public:
    typedef std::function<void(const char *, int len)> MqttMsgCallback;
    esp_mqtt_client_handle_t client{nullptr};

    MqttService() : Service("mqtt", "/littlefs/conf/mqtt.conf", /*requiresNetwork*/ true) {}

    // Wired by main.cpp before start(): does the charger BMS subscriptions + sets onConnected,
    // keeping mppt/home-assistant dependencies out of this translation unit. Called inside
    // onStart() with the loaded conf, so `svc restart mqtt` re-subscribes.
    std::function<void(const ConfFile &)> preStart{nullptr};

    // Wired by main.cpp: the throttled periodic publish (HA power, etc.). Kept here as a hook so
    // mqtt.cpp stays free of mppt/home-assistant deps. Invoked from onTick() at ~3s while Running.
    std::function<void()> tickHook{nullptr};

private:
    std::unordered_map<std::string, MqttMsgCallback> mqttMsgHandlers{};
    bool mqttConnected = false;
    bool _configured = false;
    time_ms _lastTickMs = 0;

public:
    bool isConnected()const  { return mqttConnected; }

    // A broker_uri was set by the last onStart().
    bool configured() const { return _configured; }

    std::string statusDetail() const override { return isConnected() ? "connected" : ""; }
    void (*onConnected)() = nullptr;

    void subscribeTopic(const std::string &topic, MqttMsgCallback fn);

    // false while disconnected or when the client rejects it
    bool publish(const char *topic, const char *payload, int qos, bool retain);

    // Creates and starts the client; false if that fails (nothing left allocated). True without a broker.
    bool init(const ConfFile &conf);
    void close();

    bool onStart() override;
    void onStop() override;
    void onTick() override;

    void _handleEvent(esp_event_base_t base, int32_t event_id, void *event_data);

    // Test-only: deliver a synthetic message to the registered handler for `topic`,
    // bypassing the esp_mqtt event loop. Used by test/test_charger.cpp. Asserts the
    // topic has been subscribed.
    void _invokeForTest(const std::string &topic, const char *data, int len);
};

extern MqttService MQTT;



