#pragma once

// Credentials for the config-partition file servers: NVS ftp_user/ftp_pass, else ftp.conf, else
// user / chip ID.

#include <string>

#include <esp_log.h>

#include "../conf.h"
#include "../storage/key-value.h"
#include "telemetry.h"

extern KeyValueStorage nvs;

inline void confCreds(std::string &user, std::string &pass) {
    nvs.open();
    user = nvs.readString("ftp_user", "");
    pass = nvs.readString("ftp_pass", "");
    nvs.close();
    if (user.empty() || pass.empty()) {
        ConfFile c{"/littlefs/conf/ftp.conf", true};
        if (user.empty()) user = c.getString("ftp_user", user);
        if (pass.empty()) pass = c.getString("ftp_pass", pass);
    }
    if (user.empty() || pass.empty()) {
        user = "user";
        pass = getChipId();
        ESP_LOGW("creds", "no ftp credentials set, using user=%s, password = chip ID", user.c_str());
    }
}
