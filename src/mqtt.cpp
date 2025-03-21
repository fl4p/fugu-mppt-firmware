#include "mqtt.h"
#include "conf.h"

#define TAG "mqtt"

esp_mqtt_client_handle_t client;
std::unordered_map<std::string, MqttMsgCallback> mqttMsgHandlers{};
bool mqttConnected = false;

void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void mqtt_subscribe_topic(const std::string &topic, MqttMsgCallback fn) {
    // TODO lock?
    assert(!mqttConnected);
    mqttMsgHandlers[topic] = std::move(fn);
    if (mqttConnected) {
        esp_mqtt_client_subscribe(client, topic.c_str(), 0);
    }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            for (auto &[topic, fn]: mqttMsgHandlers) {
                msg_id = esp_mqtt_client_subscribe(client, topic.c_str(), 0);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d topic=%s", msg_id, topic.c_str());
            }
            mqttConnected = true;
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
            ESP_LOGI(TAG, "MQTT_EVENT_DATA %s = %s", event->topic, event->data);
            //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            {
                auto handler = mqttMsgHandlers.find(event->topic);
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



void mqtt_init() {

    ConfFile conf{"/littlefs/conf/mqtt.conf"};

    if (!conf)return;

    auto brokerUri = conf.getString("broker_uri","");

    if(brokerUri.empty())return;

    ESP_LOGI("mqtt", "connecting to broker %s", brokerUri.c_str());


    esp_mqtt_client_config_t mqtt_cfg{};
    mqtt_cfg.broker.address.uri = brokerUri.c_str();

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

}


#undef TAG