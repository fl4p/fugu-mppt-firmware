#pragma once

// HttpConfService: HttpConfServer on /littlefs/conf while Wi-Fi is up, polled from onTick().

#include "../service.h"
#include "http_conf.h"

class HttpConfService : public Service {
public:
    HttpConfService() : Service("http", "/littlefs/conf/http.conf", /*requiresNetwork*/ true) {}

    [[nodiscard]] std::string statusDetail() const override {
        int n = srv.clients();
        return n ? std::to_string(n) + " client(s)" : "";
    }

protected:
    bool onStart() override;

    void onStop() override { srv.end(); }

    void onTick() override { srv.tick(); }

private:
    std::string _auth;
    HttpConfServer srv;
};

inline HttpConfService httpConfService;
