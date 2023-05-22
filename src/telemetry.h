#pragma once

#include "sampling.h"
#include "../.pio/libdeps/esp32dev/ESP8266 Influxdb/src/Point.h"
#include "web/server.h"
#include <InfluxDbClient.h>

#include <WiFiMulti.h>
//#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include <AsyncUDP.h>

WiFiMulti wifiMulti;
//WiFiUDP udp;
AsyncUDP asyncUdp;

const char *getChipId();

uint64_t bytesSent = 0;


void connect_wifi_async() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    wifiMulti.addAP("^__^", "modellbau");
    wifiMulti.addAP("mentha", "modellbau");

}

static bool timeSynced = false;

void wifiLoop() {
    uint8_t status = WiFi.status(); // NOLINT(readability-static-accessed-through-instance)

    if (status != WL_CONNECTED) {
        ESP_LOGI("tele", "Connecting WiFi");
        status = wifiMulti.run();
    }

    if (!timeSynced && status == WL_CONNECTED) {
        Serial.printf("Connected to WiFi, RSSI %hhi IP %s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
        Serial.println();

        String hostname = "fugu-" + String(getChipId());

        if (!MDNS.begin(hostname.c_str())) { // abc.local
            ESP_LOGE("tele", "Error setting up MDNS responder!");
        } else {
            ESP_LOGI("tele", "Set hostname %s", hostname.c_str());
        }

        Serial.println("Syncing time...");
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "pt.pool.ntp.org", "time.nis.gov");
        Serial.println("Time synchronized");
        timeSynced = true;

        webserver_begin();
    }
}

void wait_for_wifi() {
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(50);
        //Serial.print(".");
    }
    Serial.printf("Connected to WiFi, RSSI %hhi IP %s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
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

    /*
    static IPAddress host{};
    if(uint32_t(host) == 0) {
        ESP_LOGI("tele", "resolving hostname");
        host = MDNS.queryHost("homeassistant.local");
        ESP_LOGI("tele", "resolved to %s",host.toString());
    }
     */

    // byte host[] = {192, 168, 0, 185};
    byte host[] = {192, 168, 178, 28};

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
    points_frame.back().addTag("mcu", getChipId());

    if (points_frame.size() >= maxQueue) {
        if (timeSynced)
            influxWritePointsUDP(&points_frame[0], points_frame.size());
        points_frame.clear();
    }
}

ThreeChannelUnion<float> dcdcData;


void dcdcDataChanged(const DCDC_PowerSampler &dcdc, uint8_t ch) {
    dcdcData[ch] = dcdc.last[ch];

    if (ch == 1 or ch == 2) {
        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        if (&dcdc.last[ch] == &dcdc.last.s.chVout)
            point.addField("U_out_raw", dcdcData.s.chVout, 2);
        else
            point.addField("I_raw", dcdcData.s.chIin, 1);
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point);
    }


}


