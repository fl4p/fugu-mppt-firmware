#include "ftp_service.h"

#include <Arduino.h>
#include <WiFi.h>

#include "conf_creds.h"

bool FtpService::onStart() {
    if (!WiFi.isConnected()) return false;
    std::string user, pass;
    confCreds(user, pass);
    ConfFile c{"/littlefs/conf/ftp.conf", true};
    long pasv = c.getLong("pasv_port", 50009);
    if (pasv < 1024 || pasv > 65534) pasv = 50009;
    if (!ftpSrv.begin(user.c_str(), pass.c_str(), "/littlefs", 21, (uint16_t) pasv)) return false;
    ESP_LOGI("ftp", "FTP server started, user=%s, pasv %ld+", user.c_str(), pasv);
    return true;
}

void FtpService::onTick() { ftpSrv.poll(millis()); }
