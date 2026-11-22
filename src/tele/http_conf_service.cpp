#include "http_conf_service.h"

#include <WiFi.h>

#include "conf_creds.h"

bool HttpConfService::onStart() {
    if (!WiFi.isConnected()) return false;
    std::string user, pass;
    confCreds(user, pass);
    _auth = HttpConfServer::basicToken(user, pass);
    ConfFile c{_confPath, true};
    auto port = (uint16_t) c.getLong("port", 80);
    if (!srv.begin(port, "/littlefs/conf", _auth.c_str())) return false;
    ESP_LOGI("http", "conf server on port %u, user=%s", port, user.c_str());
    return true;
}
