#include "ftp_service.h"

#include <Arduino.h>
#include <WiFi.h>

#include "telemetry.h"            // getChipId
#include "../storage/key-value.h"

extern KeyValueStorage nvs;

namespace {
    // SimpleFTPServer callbacks are plain function pointers, so they can't be members. They only log
    // progress (no server access needed).
    void _callback(FtpOperation ftpOperation, uint32_t freeSpace, uint32_t totalSpace) {
        switch (ftpOperation) {
            case FTP_CONNECT:
                //ftpService._clientConnected = true;
                Serial.println(F("FTP: Connected!"));
                break;
            case FTP_DISCONNECT:
                //ftpService._clientConnected = false;
                Serial.println(F("FTP: Disconnected!"));
                break;
            case FTP_FREE_SPACE_CHANGE:
                Serial.printf("FTP: Free space change, free %lu of %lu!\n", freeSpace, totalSpace);
                break;
            default:
                break;
        }
    }

    void _transferCallback(FtpTransferOperation ftpOperation, const char *name, uint32_t transferredSize) {
        switch (ftpOperation) {
            case FTP_UPLOAD_START:
                Serial.println(F("FTP: Upload start!"));
                break;
            case FTP_UPLOAD:
                Serial.printf("FTP: Upload of file %s byte %lu\n", name, transferredSize);
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
    }
}

bool FtpService::onStart() {
    if (!WiFi.isConnected()) return false;

    // FTP exposes the littlefs /conf partition (charger/limits/sensor calibration, wifi & mqtt
    // secrets) read-write over the LAN, so the credentials must not be hard-coded. Prefer nvs,
    // fall back to ftp.conf, and warn loudly if neither is provisioned.
    nvs.open();
    _user = nvs.readString("ftp_user", "");
    _pass = nvs.readString("ftp_pass", "");
    nvs.close();

    if (_user.empty() || _pass.empty()) {
        ConfFile ftpConf{"/littlefs/conf/ftp.conf", true};
        if (_user.empty()) _user = ftpConf.getString("ftp_user", _user);
        if (_pass.empty()) _pass = ftpConf.getString("ftp_pass", _pass);
    }

    if (_user.empty() || _pass.empty()) {
        _user = "user";
        _pass = getChipId();
        ESP_LOGW("ftp", "no ftp credentials set, using user=%s, password = chip ID", _user.c_str());
    }

    ftpSrv.setCallback(_callback);
    ftpSrv.setTransferCallback(_transferCallback);
    ftpSrv.end();
    // begin() keeps the pointers, so _user/_pass (members) must stay alive past this call.
    ftpSrv.begin(_user.c_str(), _pass.c_str()); // default ports 21, 50009 for PASV
    ESP_LOGI("ftp", "FTP server started, user=%s", _user.c_str());
    return true;
}

void FtpService::onStop() { ftpSrv.end(); }

// poor perf: NetworkServer::accept() (NetworkServer.cpp) / lwip_accept (sockets.c)
void FtpService::onTick() { ftpSrv.handleFTP(); }
