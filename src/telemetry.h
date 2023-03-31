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

void wait_for_wifi() {
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(50);
        //Serial.print(".");
    }
    Serial.print("Connected to WiFi, RSSI ");
    Serial.print(WiFi.RSSI());
    Serial.print(", IP ");
    Serial.println(WiFi.localIP());
}


ThreeChannelUnion<float> dcdcData;
std::vector<Point> points_frame;

void influxWritePointsUDP(const Point *p, uint8_t len) {

    byte host[] = {192, 168, 178, 23};
    udp.beginPacket(host, 8002);
    String msg;
    for(uint8_t i = 0; i < len; ++i) {
        msg += p[i].toLineProtocol() + '\n';
    }
    udp.print(msg);
    udp.endPacket();
}


void dcdcDataChanged(const DCDC_PowerSampler &dcdc, uint8_t ch) {
    dcdcData[ch] = dcdc.last[ch];

    if(ch == 2) {
        Point point("mppt");
        point.addTag("device", "esp32_proto");
        point.addField("U_in", dcdcData.s.chVin, 3);
        point.addField("U_out", dcdcData.s.chVout, 3);
        point.addField("I_in", dcdcData.s.chIin, 2);
        point.addField("I_in_ewma", dcdc.ewm.s.chIin.avg.get(), 3);
        point.addField("I_in_ewms", dcdc.ewm.s.chIin.std.get(), 3);

        struct timeval u_time;
        gettimeofday(&u_time, NULL);
        point.setTime(getTimeStamp(&u_time, 3));
        points_frame.push_back(point);
    }

    if (points_frame.size() >= 12)
    {
        influxWritePointsUDP(&points_frame[0], points_frame.size());
        points_frame.clear();
    }
}