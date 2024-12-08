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

#include <Arduino.h>

#include <SimpleFTPServer.h>
#include <ESPTelnet.h>

#include <etc/readerwriterqueue.h>

#include "tele/scope.h"

WiFiMulti wifiMulti;
//WiFiUDP udp;
AsyncUDP asyncUdp;

FtpServer ftpSrv;

ESPTelnet telnet;


void setupTelnet();

void ftpUpdate() {
    //ftpSrv.handleFTP(); // poor perfNetworkServer::accept() (NetworkServer.cpp) / lwip_accept (sockets.c)
    telnet.loop();
    if (scope) scope->update();
    //MDNS.update();
}

void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace) {
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

void _transferCallback(FtpTransferOperation ftpOperation, const char *name, unsigned int transferredSize) {
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

    ftpSrv.begin("user", "password");    //username, password for ftp.   (default 21, 50009 for PASV)

    Serial.println("FTP server started!");

}

const char *getChipId();


bool noSsid = true;

void wifi_load_conf() {
    ConfFile wifiConf{"/littlefs/conf/wifi.conf"};

    auto starts_with = [](const std::string &s, const std::string &t) { return s.substr(0, t.length()) == t; };
    auto ends_with = [](const std::string &s, const std::string &t) { return s.substr(s.length() - t.length()) == t; };

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
    auto confPath = "/littlefs/conf/wifi.conf";
    ConfFile wifiConf{confPath, true};
    wifiConf.add({
                         {"ssid_" + ssid,          ssid},
                         {"ssid_" + ssid + "_psk", psk.c_str()}});
    ESP_LOGI("tele", "Added Wifi AP %s to %s", ssid.c_str(), confPath);
}

void connect_wifi_async() {
    if (noSsid) wifi_load_conf();
    if (!noSsid) {
        WiFi.mode(WIFI_STA);
    }
}

static bool timeSynced = false;

IPAddress ha_host{};

bool timeSyncAsync(const char *tzInfo, const char *ntpServer1, const char *ntpServer2 = nullptr,
                   const char *ntpServer3 = nullptr) {
    static unsigned long tSyncStarted = 0;

    if (!tSyncStarted) {
        ESP_LOGI("tele", "Starting time sync");
        tSyncStarted = millis() + 1;
        configTzTime(tzInfo, ntpServer1, ntpServer2, ntpServer3);
    } else if (time(nullptr) > 1000000000l) {
        ESP_LOGI("tele", "Time synced!");
        tSyncStarted = 0;
        return true;
    } else if ((millis() - tSyncStarted) > (20 * 1000)) {
        ESP_LOGW("tele", "Timeout syncing time! (%s)", ntpServer1);
        tSyncStarted = 0;
    }
    return false;
}

std::string getHostname() {
    return "fugu-" + std::string(getChipId());
}

void _wifiConnected() {
    if (!WiFi.isConnected()) return;

    if (unlikely(!timeSynced)) {
        if (timeSyncAsync("CET-1CEST,M3.5.0,M10.5.0/3", "pt.pool.ntp.org", "time.nis.gov")) {
            timeSynced = true;
        }
    }

    String hostname = String(getHostname().c_str());

    if (!MDNS.begin(hostname.c_str())) { // abc.local
        ESP_LOGE("tele", "Error setting up MDNS responder!");
    } else {
        ESP_LOGI("tele", "Set hostname %s", hostname.c_str());
    }

    MDNS.setInstanceName(hostname);

    ha_host = MDNS.queryHost("homeassistant.local");
    ESP_LOGI("tele", "%s resolved to %s", "homeassistant.local", ha_host.toString().c_str());

    setupTelnet();
    ftpBegin();

    //webserver_begin();

    scope = new Scope();
    if (!scope->begin(24)) {
        ESP_LOGE("tele", "scope setup failed");
    } else {
        if(!MDNS.addService("_scope", "_tcp", 24)) {
            ESP_LOGE("tele", "scope setup failed");
        }
        ESP_LOGI("tele", "Scope server listening on port 24");
    }
}

void wifiLoop(bool connect = false) {
    //static bool initialized = false;
    if (noSsid) return;

    if (connect && !WiFi.isConnected()) {
        ESP_LOGI("tele", "Connecting WiFi");
        wifiMulti.run();
    }

    //if (unlikely(!initialized) && WiFi.isConnected()) {
    //    initialized = true;
    //    _wifiConnected();
    // }
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

    _wifiConnected();

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


void influxWritePointsUDP(moodycamel::ReaderWriterQueue<Point> &q) {
    constexpr int MTU = CONFIG_TCP_MSS;
    // notice that MTU is not the UDP max message size, here we use MTU from ip4 as a "safe" value

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


    static String msg;

    static Point p{""};
    while (q.try_dequeue(p)) {
        auto lp = p.toLineProtocol();
        if (msg.length() + lp.length() >= MTU) {
            udpFlushString(host, port, msg);
        }
        msg += lp + '\n';
    }
}

const char *getChipId() {
    static char ssid[25]{0};
    if (!strlen(ssid))
        snprintf(ssid, 25, "%s-%llX", CONFIG_IDF_TARGET, ESP.getEfuseMac());
    return ssid;
}


static moodycamel::ReaderWriterQueue<Point> pointsQ{};

void telemetryAddPoint(Point &p, uint16_t maxQueue = 40) {
    //static std::vector<Point> points_frame;

    assert(p.hasTime());

    if (!p.hasTags())
        p.addTag("mcu", getChipId());

    if (pointsQ.size_approx() < maxQueue)
        pointsQ.enqueue(p);

    //if (points_frame.size() >= maxQueue) {
    //    if (timeSynced)
    //        influxWritePointsUDP(&points_frame[0], points_frame.size());
    //    points_frame.clear();
    //}
}

void telemetryFlushPointsQ() {
    static Point p{""};
    while (pointsQ.try_dequeue(p)) {
        influxWritePointsUDP(pointsQ);
    }
}

extern VIinVout<const Sensor *> sensors;

void dcdcDataChanged(const ADC_Sampler &dcdc, const Sensor &sensor) {
    if (timeSynced && sensor.params.rawTelemetry && !sensor.params.teleName.empty() && WiFi.isConnected()) {
        Point point("mppt");
        point.addTag("device", "fugu_" + String(getChipId()));
        point.addField(sensor.params.teleName.c_str(), sensor.last, 3);
        point.setTime(WritePrecision::MS);
        telemetryAddPoint(point, 600);
    }
}

void onTelnetConnect(String ip) {
    ESP_LOGI("telnet", "Client %s connected", ip.c_str());
    telnet.println("\nWelcome to " + String(getHostname().c_str()) + " (" + telnet.getIP() + ")");
    telnet.println("(Use ^] + q  to disconnect.)");

    set_logging_telnet(&telnet);
}

void onTelnetDisconnect(String ip) {
    set_logging_telnet(nullptr);
    ESP_LOGI("telnet", "Client %s disconnected", ip.c_str());
}

bool handleCommand(const String &inp);

void setupTelnet() {
    // passing on functions for various telnet events
    telnet.onConnect(onTelnetConnect);
    //telnet.onConnectionAttempt(onTelnetConnectionAttempt);
    //telnet.onReconnect(onTelnetReconnect);
    telnet.onDisconnect(onTelnetDisconnect);

    // passing a lambda function
    telnet.onInputReceived([](String str) {
        // checks for a certain command
        if (str == "ping") {
            telnet.println("> pong");
            Serial.println("- Telnet: pong");
        } else {
            handleCommand(str);
        }
    });

    Serial.print("- Telnet: ");
    if (telnet.begin(23)) {
        MDNS.addService("telnet", "tcp", 23);
        ESP_LOGI("tele", "Telnet server running.");
    } else {
        ESP_LOGE("tele", "Telnet server start error");
    }
}
