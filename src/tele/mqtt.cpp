#include <Arduino.h> // String
#if CONFIG_FUGU_WITH_HTTPS
#include <esp_crt_bundle.h>
#endif
#include "mqtt.h"
#include "../conf.h"
#include "../util.h"

#include "HAMqttDevice.h"
#include "logging.h"
#include "telemetry.h"

#define TAG "mqtt"


static void mqttLogCallback(const char *str, uint16_t len);

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void MqttService::subscribeTopic(const std::string &topic, MqttMsgCallback fn) {
    // TODO lock?
    assert_throw(!mqttConnected, "");
    mqttMsgHandlers[topic] = std::move(fn);
    if (mqttConnected) {
        ESP_LOGI(TAG, "Subscribe to '%s", topic.c_str());
        assert_throw(esp_mqtt_client_subscribe(client, topic.c_str(), 0) != 1, "");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    auto svc = (MqttService *) handler_args;
    svc->_handleEvent(base, event_id, event_data);
}

void MqttService::_handleEvent(esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    auto event = (esp_mqtt_event_handle_t) event_data;
    auto client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            for (auto &topic: mqttMsgHandlers) {
                msg_id = esp_mqtt_client_subscribe(client, topic.first.c_str(), 0);
                ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d topic=%s", msg_id, topic.first.c_str());
            }
            mqttConnected = true;

            addLogCallback(mqttLogCallback);
            if (onConnected) onConnected();
        }
        break;

        case MQTT_EVENT_DISCONNECTED:
            mqttConnected = false;
            removeLogCallback(mqttLogCallback);
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

static void mqttLogCallback(const char *str, uint16_t len) {
    static char *topic = nullptr;
    static volatile TaskHandle_t inPublish = nullptr;
    if (!MQTT.isConnected()) return;
    if (strnstr(str, ") mqtt:", len) != nullptr) return; // don't publish mqtt messages

    // On WiFi teardown esp-mqtt's transport floods errors from mqtt_task; mirroring them re-enters
    // publish on that same task (write to dead socket -> another error -> ...) and overflows its
    // stack. Drop any log produced while we're already publishing on this task.
    auto self = xTaskGetCurrentTaskHandle();
    if (inPublish == self) return;

    if (topic == nullptr) {
        auto t = "pv/log/" + getHostname(true);
        topic = new char[t.length() + 1];
        t.copy(topic, t.length()); // does not append null !
        topic[t.length()] = '\0'; // terminate
        ESP_LOGI(TAG, "console log topic='%s'", topic);
    }

    inPublish = self;
    if (esp_mqtt_client_publish(MQTT.client, topic, str, len, 0, 0) < 0)
        ESP_LOGE(TAG, "publish error %d", len);
    inPublish = nullptr;
    //esp_mqtt_client_publish(MQTT.client, topic, str, len, 0, 0);
    //esp_mqtt_client_enqueue(MQTT.client, topic, str, len, 0, 0, false);
}


void MqttService::init(const ConfFile &conf) {
    if (!conf)return;

    auto brokerUri = conf.getString("broker_uri", "");
    auto username = conf.getString("username", "");
    auto password = conf.getString("password", "");

    if (brokerUri.empty())return;

    ESP_LOGI(TAG, "connecting to broker %s@%s (pw %u chars)", username.c_str(), brokerUri.c_str(),
             password.length());


    esp_mqtt_client_config_t mqtt_cfg{};
    mqtt_cfg.broker.address.uri = brokerUri.c_str();
    mqtt_cfg.task.stack_size = 8192; // default ~6 KB overflows during the transport-error storm on WiFi teardown
#if CONFIG_FUGU_WITH_HTTPS
    mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    if (!username.empty())mqtt_cfg.credentials.username = username.c_str();
    if (!password.empty())mqtt_cfg.credentials.authentication.password = password.c_str();

    client = esp_mqtt_client_init(&mqtt_cfg);
    assert_throw(client, "");
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, &MQTT);
    esp_mqtt_client_start(client);
}

void MqttService::close() {
    if (client) {
        if (isConnected()) esp_mqtt_client_disconnect(client);
        mqttConnected = false;
        esp_mqtt_client_stop(client);
        esp_mqtt_client_unregister_event(client, MQTT_EVENT_ANY, mqtt_event_handler);
        esp_mqtt_client_destroy(client);
        client = nullptr; // else a restart with empty broker_uri (init() early-returns) leaves it dangling
    }
    removeLogCallback(mqttLogCallback); // drop the sink; stop may not fire MQTT_EVENT_DISCONNECTED
    mqttMsgHandlers.clear(); // onStart/preStart rebuilds subscriptions, so a restart honors conf (e.g. cmd_input)
}

bool MqttService::onStart() {
    ConfFile conf{_confPath, /*no_warn_if_not_open*/ true};
    if (!conf) {
        ESP_LOGI(TAG, "no mqtt.conf - mqtt idle");
        return true; // nothing configured: idle is OK, not Failed
    }
    if (preStart) preStart(conf); // charger BMS subscriptions + onConnected (wired by main.cpp)

    // Console-command input over MQTT (output is mirrored to pv/log/<hostname> by mqttLogCallback).
    if (conf.getByte("cmd_input", 0)) {
        subscribeTopic("pv/log/" + getHostname(true) + "/cmd", [](const char *data, int len) {
            std::string cmd(data, len);
            // Defer to loopNetwork_task (where process_queued_tasks runs). Running handleCommand
            // inline here executes on the MQTT client event callback (mqtt_task); a teardown command
            // (svc rs/off mqtt, wifi off, restart) would then stop/destroy the client + clear the
            // handler map from inside its own event while this std::function is executing -> UAF.
            enqueue_task([cmd]() {
                String inp(cmd.c_str());
                inp.trim();
                if (!inp.length()) return;
                bool ok = handleCommand(inp);
                UART_LOG("%s: %s", ok ? "OK" : "ERR", inp.c_str());
            });
        });
    }

    init(conf);                   // connection is async; client task runs on its own
    return true;
}

void MqttService::onStop() {
    close();
}

void MqttService::onTick() {
    // Throttled periodic publish (HA power, etc.). The body is wired by main.cpp via tickHook to
    // keep mppt/home-assistant deps out of this TU; haMqttUpdate() itself no-ops until connected.
    if (!tickHook) return;
    auto now = wallClockMs();
    if (now - _lastTickMs < 3000) return;
    _lastTickMs = now;
    tickHook();
}

void MqttService::_invokeForTest(const std::string &topic, const char *data, int len) {
    auto h = mqttMsgHandlers.find(topic);
    assert(h != mqttMsgHandlers.end());
    h->second(data, len);
}

MqttService MQTT;


#undef TAG
