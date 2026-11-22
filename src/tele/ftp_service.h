#pragma once

// FtpService: exposes the littlefs partition over FTP while Wi-Fi is up (ftp_server.h).

#include "ftp_server.h"
#include "../service.h"

class FtpService : public Service {
public:
    FtpService() : Service("ftp", "/littlefs/conf/ftp.conf", /*requiresNetwork*/ true) {
    }

    [[nodiscard]] std::string statusDetail() const override { return ftpSrv.clients(); }

protected:
    bool onStart() override;

    void onStop() override { ftpSrv.end(); }

    void onTick() override;

private:
    FtpServer ftpSrv;
};

inline FtpService ftpService;
