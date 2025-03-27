#pragma once

#include <functional>
#include <string>

#include "mqtt_client.h"

typedef std::function<void(const char *, int len)> MqttMsgCallback;


void mqtt_subscribe_topic(const std::string &topic, MqttMsgCallback fn);
void mqtt_init();

void mqttUpdateSensors(const float &power);
