#pragma once

#include "sampling.h"
#include "../.pio/libdeps/esp32dev/ESP8266 Influxdb/src/Point.h"
#include <InfluxDbClient.h>

#include <WiFiMulti.h>
#include <WiFiUdp.h>

WiFiMulti wifiMulti;
WiFiUDP udp;


void connect_wifi_async(const std::string &ssid, const std::string &pw) {
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid.c_str(), pw.c_str());
}

static bool timeSynced = false;

void wifiLoop() {
    if (wifiMulti.run() != WL_CONNECTED) {
        // do nothing
    } else {
        if (!timeSynced) {
            Serial.printf("Connected to WiFi, RSSI %hhi IP %s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
            Serial.println();
            Serial.println("Syncing time...");
            timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "pt.pool.ntp.org", "time.nis.gov");
            Serial.println("Time synchronized");
            timeSynced = true;
        }
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
    udp.beginPacket(host, port);
    udp.print(msg);
    udp.endPacket();
    msg.clear();
}


void influxWritePointsUDP(const Point *p, uint8_t len) {

    constexpr int MTU = 1300;
    byte host[] = {192, 168, 0, 177};
    auto port = 8002;


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

const char * getChipId() {
    static char ssid[20] {0};
    if(!strlen(ssid))
        snprintf(ssid, 20, "%s-%llX", CONFIG_IDF_TARGET, ESP.getEfuseMac());
    return ssid;
}

void telemetryAddPoint(const Point &p, uint16_t maxQueue=40) {
    static std::vector<Point> points_frame;

    assert(p.hasTime());
    points_frame.push_back(p);
    points_frame.back().addTag("mcu", getChipId());

    if (points_frame.size() >= maxQueue) {
        influxWritePointsUDP(&points_frame[0], points_frame.size());
        points_frame.clear();
    }
}

ThreeChannelUnion<float> dcdcData;

void dcdcDataChanged(const DCDC_PowerSampler &dcdc, uint8_t ch) {
    dcdcData[ch] = dcdc.last[ch];

    if (ch == 2) {
        Point point("mppt");
        point.addTag("device", "esp32_proto");
        point.addField("U_in", dcdcData.s.chVin, 3);
        point.addField("U_out", dcdcData.s.chVout, 3);
        point.addField("I_in", dcdcData.s.chIin, 2);
        point.addField("I_in_ewma", dcdc.ewm.s.chIin.avg.get(), 3);
        point.addField("I_in_ewms", dcdc.ewm.s.chIin.std.get(), 3);
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point);
    }


}

