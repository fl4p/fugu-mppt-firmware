#pragma once

#include <functional>
#include <string>

#include "../conf.h"
#include "mqtt_client.h"

class Service {
};

class MqttService : public Service {
public:
    typedef std::function<void(const char *, int len)> MqttMsgCallback;
    esp_mqtt_client_handle_t client{nullptr};

private:
    std::unordered_map<std::string, MqttMsgCallback> mqttMsgHandlers{};
    bool mqttConnected = false;

public:
    bool isConnected()const  { return mqttConnected; }
    void (*onConnected)() = nullptr;

    void subscribeTopic(const std::string &topic, MqttMsgCallback fn);

    void init(const ConfFile &conf);
    void close();

    void _handleEvent(esp_event_base_t base, int32_t event_id, void *event_data);
};

extern MqttService MQTT;



