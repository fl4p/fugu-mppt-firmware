#pragma once

#include "adc/sampling.h"
#include "Point.h"
#include "web/server.h"
#include "store.h"
#include <InfluxDbClient.h>

#include <WiFiMulti.h>
//#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include <AsyncUDP.h>

//#include <Preferences.h>

#include <SimpleFTPServer.h>

WiFiMulti wifiMulti;
//WiFiUDP udp;
AsyncUDP asyncUdp;

FtpServer ftpSrv;


void ftpUpdate() {
    ftpSrv.handleFTP();
}

void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace){
    switch (ftpOperation) {
        case FTP_CONNECT:
            Serial.println(F("FTP: Connected!"));
            break;
        case FTP_DISCONNECT:
            Serial.println(F("FTP: Disconnected!"));
            break;
        case FTP_FREE_SPACE_CHANGE:
            Serial.printf("FTP: Free space change, free %u of %u!\n", freeSpace, totalSpace);
            break;
        default:
            break;
    }
};
void _transferCallback(FtpTransferOperation ftpOperation, const char* name, unsigned int transferredSize){
    switch (ftpOperation) {
        case FTP_UPLOAD_START:
            Serial.println(F("FTP: Upload start!"));
            break;
        case FTP_UPLOAD:
            Serial.printf("FTP: Upload of file %s byte %u\n", name, transferredSize);
            break;
        case FTP_TRANSFER_STOP:
            Serial.println(F("FTP: Finish transfer!"));
            break;
        case FTP_TRANSFER_ERROR:
            Serial.println(F("FTP: Transfer error!"));
            break;
        default:
            break;
    }

    /* FTP_UPLOAD_START = 0,
     * FTP_UPLOAD = 1,
     *
     * FTP_DOWNLOAD_START = 2,
     * FTP_DOWNLOAD = 3,
     *
     * FTP_TRANSFER_STOP = 4,
     * FTP_DOWNLOAD_STOP = 4,
     * FTP_UPLOAD_STOP = 4,
     *
     * FTP_TRANSFER_ERROR = 5,
     * FTP_DOWNLOAD_ERROR = 5,
     * FTP_UPLOAD_ERROR = 5
     */
};

void ftpBegin() {
    ftpSrv.setCallback(_callback);
    ftpSrv.setTransferCallback(_transferCallback);

    ftpSrv.begin("user","password");    //username, password for ftp.   (default 21, 50009 for PASV)

    Serial.println("FTP server started!");

}

const char *getChipId();

uint64_t bytesSent = 0;


bool noSsid = true;

void wifi_load_conf() {
    ConfFile wifiConf{"/littlefs/wifi"};

    auto starts_with = [](const std::string &s, const std::string &t) { return s.substr(0, t.length()) == t; };
    auto ends_with = [](const std::string &s, const std::string &t) { return s.substr(s.length()- t.length()) == t; };

    noSsid = true;
    for (auto k: wifiConf.keys()) {
        if (starts_with(k, "ssid") && !ends_with(k, "_psk")) {
            auto ssid = wifiConf.getString(k).c_str();
            auto psk = wifiConf.c(k + "_psk", nullptr);
            if (!wifiMulti.addAP(wifiConf.getString(k).c_str(), psk)) {
                ESP_LOGW("tele", "Failed to add ap  %s", ssid);
            } else {
                ESP_LOGI(__FILENAME__, "Add WiFi SSID %s (psk %s)", ssid, psk ? psk : "<none>");
                noSsid = false;
            }
        }
    }
}

void add_ap(const std::string &ssid, const std::string &psk) {
    ConfFile wifiConf{"/littlefs/wifi"};
    wifiConf.add({
                         {"ssid_" + ssid,          ssid},
                         {"ssid_" + ssid + "_psk", psk.c_str()}});
}

void connect_wifi_async() {
    if (noSsid) wifi_load_conf();
    if (!noSsid) WiFi.mode(WIFI_STA);
}

static bool timeSynced = false;

IPAddress ha_host{};

void wifiLoop(bool connect = false) {
    if (noSsid) return;

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
        ftpBegin();
    }
}

bool wait_for_wifi() {
    if (noSsid) return false;

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

    if (noSsid) return;

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


