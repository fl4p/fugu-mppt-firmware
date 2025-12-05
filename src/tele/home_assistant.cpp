#include "HAMqttDevice.h"
#include "home_assistant.h"

#include "mqtt.h"
#include "mqtt_client.h"
#include "telemetry.h"

//HAMqttDevice *powerSensor = nullptr;
uint16_t numUpdates = 0;
std::string powerSensorTopic;


void haMqttSendDiscovery() {
    // https://www.home-assistant.io/integrations/sensor.mqtt/

    /*
     *         hass_config_data["device_class"] = 'power'
        hass_config_data["unit_of_measurement"] = 'W'
     */
    String hn = getHostname().c_str();
    hn.replace('-', ' ');

    const std::string id = getDeviceId() + "-power";

    auto powerSensor = HAMqttDevice(id.c_str(),
                                    HAMqttDevice::SENSOR,
                                    "homeassistant"
    );
    // https://www.home-assistant.io/integrations/sensor.mqtt/
    // https://www.home-assistant.io/integrations/mqtt/#supported-abbreviations-in-mqtt-discovery-messages
    (powerSensor)
            .addConfigVar("name", "Power")
            .addConfigVar("uniq_id", id.c_str())
            .addConfigVar("dev_cla", "power")
            .addConfigVar("unit_of_meas", "W")
            //.addConfigVar("retain", "true")
            .addConfigVar("ic", "mdi:flash")
            .addConfigVar("native_unit_of_measurement", "W")
            .addConfigVar("suggested_unit_of_measurement", "W")
            .addConfigVar("sug_dsp_prc", 1)
            .addConfigVar("stat_cla", "measurement")
            .addConfigVar("exp_aft", 30)
            .addConfigVar("dev",
                          (R"({"name":")" + getHostname() + R"(","ids":[")" + getDeviceId() + R"("]})").c_str());
    powerSensorTopic = powerSensor.getConfigTopic().c_str();

    auto payload = powerSensor.getConfigPayload();
    auto res = esp_mqtt_client_publish(MQTT.client, powerSensor.getConfigTopic().c_str(), payload.c_str(),
                                       payload.length(), 0, 1);
    if (res < 0)
        ESP_LOGE("mqtt", "publish error %d", res);
    else
        ESP_LOGD("mqtt", "published %s: %s", powerSensor.getConfigTopic().c_str(), payload.c_str());
}


void haMqttUpdate(const HAMqttFields &fields) {
    if (!MQTT.isConnected()) return;
    char buf[16];
    if (std::isfinite(fields.power)) {

        if (numUpdates % 1000 == 0) {
            haMqttSendDiscovery();
        }

        auto len = snprintf(buf, 16, "%5.*f", fields.power > 999 ? 0 : 3, fields.power);
        if (len < 1)
            ESP_LOGE("mqtt", "snprintf error %d", len);
        else {
            len = esp_mqtt_client_publish(MQTT.client, powerSensorTopic.c_str(), buf, len, 0, 0);
            if (len < 0)
                ESP_LOGE("mqtt", "publish error %d", len);
            else
                ESP_LOGD("mqtt", "published %s: %s", powerSensorTopic.c_str(), buf);
        }

        ++numUpdates;
    }
}
