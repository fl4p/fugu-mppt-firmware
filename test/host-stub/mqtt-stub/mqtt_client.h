#pragma once
// Host stub of esp-mqtt's mqtt_client.h with the declarations src/tele/mqtt.cpp uses.
// The functions are defined by the test (mqtt-service-test.cpp).
#include <cinttypes>
#include <cstdint>
#include <cstring>

typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#define ESP_FAIL (-1)
#endif
const char *esp_err_to_name(esp_err_t);
uint32_t esp_get_free_heap_size();
int64_t esp_timer_get_time();
typedef void *TaskHandle_t;
TaskHandle_t xTaskGetCurrentTaskHandle();

typedef const char *esp_event_base_t;
typedef void (*esp_event_handler_t)(void *arg, esp_event_base_t base, int32_t id, void *data);

typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

typedef enum {
    MQTT_EVENT_ANY = -1, MQTT_EVENT_ERROR = 0, MQTT_EVENT_CONNECTED, MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_SUBSCRIBED, MQTT_EVENT_UNSUBSCRIBED, MQTT_EVENT_PUBLISHED, MQTT_EVENT_DATA,
    MQTT_EVENT_BEFORE_CONNECT, MQTT_EVENT_DELETED,
} esp_mqtt_event_id_t;

enum { MQTT_ERROR_TYPE_NONE = 0, MQTT_ERROR_TYPE_TCP_TRANSPORT };

struct esp_mqtt_error_codes_t {
    esp_err_t esp_tls_last_esp_err;
    int esp_tls_stack_err;
    int error_type;
    int esp_transport_sock_errno;
};

struct esp_mqtt_event_t {
    esp_mqtt_event_id_t event_id;
    esp_mqtt_client_handle_t client;
    char *data;
    int data_len;
    char *topic;
    int topic_len;
    int msg_id;
    esp_mqtt_error_codes_t *error_handle;
};
typedef esp_mqtt_event_t *esp_mqtt_event_handle_t;

struct esp_mqtt_client_config_t {
    struct { struct { const char *uri; } address; } broker;
    struct { const char *username; struct { const char *password; } authentication; } credentials;
    struct { int stack_size; } task;
};

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *);
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t, esp_mqtt_event_id_t, esp_event_handler_t, void *);
esp_err_t esp_mqtt_client_unregister_event(esp_mqtt_client_handle_t, esp_mqtt_event_id_t, esp_event_handler_t);
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t);
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t);
esp_err_t esp_mqtt_client_disconnect(esp_mqtt_client_handle_t);
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t);
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t, const char *topic, int qos);
int esp_mqtt_client_publish(esp_mqtt_client_handle_t, const char *topic, const char *data, int len, int qos, int retain);
