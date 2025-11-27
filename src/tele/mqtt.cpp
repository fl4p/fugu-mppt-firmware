#include "../mqtt.h"
#include "../conf.h"
#include "../util.h"

#include <HAMqttDevice.h>
#include "../telemetry.h"

#define TAG "mqtt"


esp_mqtt_client_handle_t client;
std::unordered_map<std::string, MqttMsgCallback> mqttMsgHandlers{};
bool mqttConnected = false;

HAMqttDevice * powerSensor = nullptr;

void setupSensor() {

    /*
     *         hass_config_data["device_class"] = 'power'
        hass_config_data["unit_of_measurement"] = 'W'
     */
    powerSensor = new HAMqttDevice((getHostname() + " Power").c_str(), HAMqttDevice::SENSOR);
    (*powerSensor)
            .addConfigVar("device_class", "power")
            .addConfigVar("unit_of_measurement", "W")
            .addConfigVar("retain", "false")
            .addConfigVar("unique_id", (getHostname() + "_power").c_str());

    powerSensor->enableStateTopic();

    auto payload = powerSensor->getConfigPayload();
    auto res = esp_mqtt_client_publish(client, powerSensor->getConfigTopic().c_str(), payload.c_str(),
                                       payload.length(), 0, 0);
    if (res < 0) ESP_LOGE("mqtt", "publish error %d", res);
    else
        ESP_LOGI("mqtt", "published %s: %s", powerSensor->getConfigTopic().c_str(), payload.c_str());

}

void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void mqtt_subscribe_topic(const std::string &topic, MqttMsgCallback fn) {
    // TODO lock?
    assert_throw(!mqttConnected, "");
    mqttMsgHandlers[topic] = std::move(fn);
    if (mqttConnected) {
        ESP_LOGI("mqtt", "Subscribe to '%s", topic.c_str());
        assert_throw(esp_mqtt_client_subscribe(client, topic.c_str(), 0) != 1, "");
    }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    auto event = (esp_mqtt_event_handle_t) event_data;
    auto client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            for (auto &topic: mqttMsgHandlers) {
                msg_id = esp_mqtt_client_subscribe(client, topic.first.c_str(), 0);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d topic=%s", msg_id, topic.first.c_str());
            }
            mqttConnected = true;

            setupSensor();

            break;
        case MQTT_EVENT_DISCONNECTED:
            mqttConnected = false;
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT_EVENT_DATA '%.*s' = '%.*s'", event->topic_len, event->topic, event->data_len,
                     event->data);
            //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            {
                auto handler = mqttMsgHandlers.find(std::string{event->topic, event->topic + event->topic_len});
                if (handler != mqttMsgHandlers.end()) {
                    handler->second(event->data, event->data_len);
                } else {
                    ESP_LOGW(TAG, "no handler for topic %s", event->topic);
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",
                                     event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}


void mqtt_init(const ConfFile &conf) {


    if (!conf)return;

    auto brokerUri = conf.getString("broker_uri", "");
    auto username = conf.getString("username", "");
    auto password = conf.getString("password", "");

    if (brokerUri.empty())return;

    ESP_LOGI("mqtt", "connecting to broker %s@%s (pw %u chars)", username.c_str(), brokerUri.c_str(),
             password.length());


    esp_mqtt_client_config_t mqtt_cfg{};
    mqtt_cfg.broker.address.uri = brokerUri.c_str();
    //mqtt_cfg.broker.verification.
    if (!username.empty())mqtt_cfg.credentials.username = username.c_str();
    if (!password.empty())mqtt_cfg.credentials.authentication.password = password.c_str();

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == nullptr) ESP_LOGE("mqtt", "error");
    //assert_throw(client, "");
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);


}

void mqttUpdateSensors(const float &power) {
    char buf[16];
    auto len = snprintf(buf, 16, "%5.*f", power > 999 ? 0 : 3, power);
    if (len < 1)
        ESP_LOGE("mqtt", "snprintf error %d", len);
    else {
        len = esp_mqtt_client_publish(client, powerSensor->getStateTopic().c_str(), buf, len, 0, 0);
        if (len < 0) ESP_LOGE("mqtt", "publish error %d", len);
        else
            ESP_LOGI("mqtt", "published %s: %s", powerSensor->getStateTopic().c_str(), buf);
    }
}


#undef TAG