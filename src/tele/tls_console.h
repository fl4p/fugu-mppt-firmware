#pragma once

// TlsConsoleService: the console over TCP wrapped in TLS 1.2 with a pre-shared key
// (TLS-PSK-WITH-AES-128-GCM-SHA256). tlscon.conf: psk_id, psk (32..64 hex digits), port. Refuses to
// start without a key. One client at a time; it receives the muxed log, its lines run as commands.
// The TLS state is allocated only while a client is connected.

#include "../service.h"

class TlsConsoleService : public Service {
public:
    TlsConsoleService() : Service("tlscon", "/littlefs/conf/tlscon.conf", /*requiresNetwork*/ true,
                                  /*enabledDefault*/ false) {
    }

    [[nodiscard]] std::string statusDetail() const override;

protected:
    bool onStart() override;

    void onStop() override;

    void onTick() override;
};

inline TlsConsoleService tlsConsoleService;
