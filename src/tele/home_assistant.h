#pragma once

struct HAMqttFields {
    const float &power;
};

void haMqttSendDiscovery();
void haMqttUpdate(const HAMqttFields &fields);