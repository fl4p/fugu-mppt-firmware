// Runtime half of the host closed-loop sim stubs (see mppt-sim.cpp): simulated clock, ADC timer
// tick, /littlefs redirection, logging, and link stubs for code that is never reached in the loop.
#include <cstdarg>
#include <cstring>
#include <string>
#include <functional>

#include "esp_timer.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "logging.h"
#include "etc/rt.h"
#include "viz/lcd.h"
#include "tele/mqtt.h"
#include "adc/adc_esp32_cont.h"
#include "sim_alloc.h"

#undef fopen
#undef rename
#undef unlink

int64_t g_simUs = 0;
void (*g_simDelayHook)(uint32_t ms) = nullptr;
bool g_simRunTasks = false;
float g_simChipTemp = 40.f;
uint32_t g_simTickUs = 333;
std::string g_simFsRoot;
void (*g_simLogHook)(char lvl, const char *tag, const char *msg) = nullptr;

static gptimer_t *const kTimer = (gptimer_t *) 1;

esp_err_t gptimer_new_timer(const gptimer_config_t *, gptimer_handle_t *h) {
    *h = kTimer;
    return ESP_OK;
}

esp_err_t gptimer_set_alarm_action(gptimer_handle_t, const gptimer_alarm_config_t *c) {
    g_simTickUs = (uint32_t) c->alarm_count; // resolution is 1 MHz
    return ESP_OK;
}

// One ADC timer alarm per call: advance the sim clock, report one notification.
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t) {
    g_simUs += g_simTickUs;
    return 1;
}

FILE *sim_fopen(const char *path, const char *mode) {
    if (strncmp(path, "/littlefs/", 10) == 0)
        return fopen((g_simFsRoot + path).c_str(), mode);
    return fopen(path, mode);
}

static std::string simPath(const char *path) {
    return strncmp(path, "/littlefs/", 10) == 0 ? g_simFsRoot + path : path;
}

int sim_rename(const char *from, const char *to) { return rename(simPath(from).c_str(), simPath(to).c_str()); }

int sim_unlink(const char *path) { return unlink(simPath(path).c_str()); }

// target RT logging captures the line raw, formatting happens on core 0: not counted here
static void emit(char lvl, const char *tag, const char *fmt, va_list ap) {
    SimAllocPause pause;
    char buf[512];
    vsnprintf(buf, sizeof buf, fmt, ap);
    if (g_simLogHook) g_simLogHook(lvl, tag, buf);
}

void sim_log(char lvl, const char *tag, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    emit(lvl, tag, fmt, ap);
    va_end(ap);
}

void UART_LOG(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    emit('U', "uart", fmt, ap);
    va_end(ap);
}

void printf_mux(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    emit('U', "uart", fmt, ap);
    va_end(ap);
}

void loggingEnableDefer() {}
void enable_esp_log_to_telnet() {}
void flush_async_uart_log() {}
void process_queued_tasks() {}
void enqueue_task(std::function<void(void)> &&) {} // LCD messages, meter commits, deferred calibration
bool rt_defer(RtDeferredFn, void *, float, float) { return true; } // same: core-0 work is not simulated
void addLogCallback(LogCallback, bool) {}
void removeLogCallback(LogCallback) {}
esp_err_t esp_intr_dump(FILE *) { return 0; }

volatile bool rtcount_en = false;
void rtcount(const char *) {}
void assertPinState(uint8_t, bool, const char *, bool) {}
void scan_i2c() {}
float strntof(const char *dat, int len) {
    std::string s(dat, len);
    return strtof(s.c_str(), nullptr);
}

bool LCD::init(uint8_t) { return false; }
void LCD::periodicInit() {}
void LCD::displayMessage(const std::string &, uint16_t) {}
void LCD::displayMessageF(const std::string &, uint16_t, ...) {}
void LCD::updateValues(const LcdValues &) {}

MqttService MQTT;
void MqttService::subscribeTopic(const std::string &, MqttMsgCallback) {}
bool MqttService::init(const ConfFile &) { return true; }
void MqttService::close() {}
bool MqttService::onStart() { return false; }
void MqttService::onStop() {}
void MqttService::onTick() {}
void MqttService::_handleEvent(esp_event_base_t, int32_t, void *) {}
void MqttService::_invokeForTest(const std::string &, const char *, int) {}

// sensor_setup.cpp can construct it for adc=esp32adc1; never selected by the vconv board config.
void ADC_ESP32_Cont::start() { abort(); }
uint32_t ADC_ESP32_Cont::read(SampleCallback &&) { abort(); }

volatile bool g_hwOvTrip = false;
