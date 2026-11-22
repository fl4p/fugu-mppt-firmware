#include "ftp_service.h"

#include <Arduino.h>
#include <WiFi.h>

#include "conf_creds.h"

bool FtpService::onStart() {
    if (!WiFi.isConnected()) return false;
    std::string user, pass;
    confCreds(user, pass);
    if (!ftpSrv.begin(user.c_str(), pass.c_str(), "/littlefs")) return false; // ports 21, 50009+ (PASV)
    ESP_LOGI("ftp", "FTP server started, user=%s", user.c_str());
    return true;
}

void FtpService::onTick() { ftpSrv.poll(millis()); }
