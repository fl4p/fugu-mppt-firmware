#pragma once

#include "adc/sampling.h"
#include "../.pio/libdeps/esp32dev/ESP8266 Influxdb/src/Point.h"
#include "web/server.h"
#include <InfluxDbClient.h>

#include <WiFiMulti.h>
//#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include <AsyncUDP.h>

#include <Preferences.h>

WiFiMulti wifiMulti;
//WiFiUDP udp;
AsyncUDP asyncUdp;

const char *getChipId();

uint64_t bytesSent = 0;

Preferences preferences;

bool noSsid = false;

void connect_wifi_async() {

    if (!preferences.begin("net", false, "littlefs")) {
        ESP_LOGE(__FILENAME__, "Error opening preferences");
    }

    auto ssidAndPw = preferences.getString("wifi_cred", "");
    noSsid = ssidAndPw.isEmpty();
    if (!noSsid) {
        auto br = ssidAndPw.indexOf('\n');
        auto ssid = ssidAndPw.substring(0,br);
        auto psk = ssidAndPw.substring(br+1);
        ESP_LOGI(__FILENAME__, "Add WiFi SSID %s", ssid.c_str());
        WiFi.mode(WIFI_STA);
        //WiFi.setAutoReconnect(true);
        wifiMulti.addAP(ssid.c_str(), psk.isEmpty() ? nullptr : psk.c_str());
    }

}

static bool timeSynced = false;

IPAddress ha_host{};

void wifiLoop(bool connect = false) {
    if(noSsid) return;

    if (connect && WiFi.status() != WL_CONNECTED) {
        ESP_LOGI("tele", "Connecting WiFi");
        wifiMulti.run();
    }

    if (!timeSynced/*&& status == WL_CONNECTED*/) {
        uint8_t status = WiFi.status(); // NOLINT(readability-static-accessed-through-instance)

        if (status != WL_CONNECTED)
            return;

        Serial.printf("Connected to WiFi, RSSI %hhi IP %s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
        Serial.println();

        String hostname = "fugu-" + String(getChipId());

        if (!MDNS.begin(hostname.c_str())) { // abc.local
            ESP_LOGE("tele", "Error setting up MDNS responder!");
        } else {
            ESP_LOGI("tele", "Set hostname %s", hostname.c_str());
        }

        ESP_LOGI("tele", "Syncing time ...");
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "pt.pool.ntp.org", "time.nis.gov");
        ESP_LOGI("tele", "Time synchronized");
        timeSynced = true;


        ha_host = MDNS.queryHost("homeassistant.local");
        ESP_LOGI("tele", "resolved to %s", ha_host.toString().c_str());

        webserver_begin();
    }
}

bool wait_for_wifi() {
    if(noSsid) return false;

    ESP_LOGI("tele", "Connecting WiFi...");
    auto t_start = millis();
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(50);
        //Serial.print(".");
        if (millis() - t_start > 6000) {
            ESP_LOGW("tele", "WiFi connection timeout");
            return false;
        }
    }
    ESP_LOGI("tele", "Connected to WiFi, RSSI %hhi IP %s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
    return true;
}

void udpFlushString(const IPAddress &host, uint16_t port, String &msg) {
    if (msg.length() > CONFIG_TCP_MSS) {
        ESP_LOGW("tele", "Payload len %d > TCP_MSS: %s", msg.length(), msg.substring(0, 200).c_str());
        msg.clear();
        return;
    }

    bytesSent += asyncUdp.writeTo((uint8_t *) msg.c_str(), msg.length(), host, port);

    //udp.beginPacket(host, port);
    //udp.print(msg);
    //udp.endPacket();

    msg.clear();
}


void influxWritePointsUDP(const Point *p, uint8_t len) {
    constexpr int MTU = CONFIG_TCP_MSS;

    if(noSsid) return ;

    static IPAddress host{};


    if (uint32_t(host) == 0) {
        if (uint32_t(ha_host) == 0) {
            if (WiFi.localIP().toString().startsWith("192.168.178")) {
                host = IPAddress({192, 168, 178, 28});
            } else {
                host = IPAddress({192, 168, 0, 185});
            }
        } else {
            host = ha_host;
        }
        ESP_LOGI("tele", "using udp target host %s", host.toString().c_str());
    }

    auto port = 8086;


    String msg;

    for (uint8_t i = 0; i < len; ++i) {
        auto lp = p[i].toLineProtocol();
        if (msg.length() + lp.length() >= MTU) {
            udpFlushString(host, port, msg);
        }
        msg += lp + '\n';
    }

    if (msg.length() > 0) {
        udpFlushString(host, port, msg);
    }
}

const char *getChipId() {
    static char ssid[25]{0};
    if (!strlen(ssid))
        snprintf(ssid, 25, "%s-%llX", CONFIG_IDF_TARGET, ESP.getEfuseMac());
    return ssid;
}

void telemetryAddPoint(const Point &p, uint16_t maxQueue = 40) {
    static std::vector<Point> points_frame;

    assert(p.hasTime());
    points_frame.push_back(p);
    if (!p.hasTags())
        points_frame.back().addTag("mcu", getChipId());

    if (points_frame.size() >= maxQueue) {
        if (timeSynced)
            influxWritePointsUDP(&points_frame[0], points_frame.size());
        points_frame.clear();
    }
}

extern VIinVout<const ADC_Sampler::Sensor *> sensors;

void dcdcDataChanged(const ADC_Sampler &dcdc, const ADC_Sampler::Sensor &sensor) {
    if (timeSynced && sensor.params.rawTelemetry && !sensor.params.teleName.empty() && WiFi.isConnected()) {
        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField(sensor.params.teleName.c_str(), sensor.last, 3);
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point, 600);
    }
}


