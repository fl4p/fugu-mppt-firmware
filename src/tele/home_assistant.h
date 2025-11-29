#pragma once

struct HAMqttFields {
    const float &power;
};

void haMqttBegin();
void haMqttSendDiscovery();
void haMqttUpdate(const HAMqttFields &fields);