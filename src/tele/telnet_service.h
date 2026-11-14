#pragma once

// TelnetService: remote console on port 23 while Wi-Fi is up. The telnet client also doubles as a
// log sink, so onStop() drops that sink before the server dies. Owns the ESPTelnet object, its
// event handlers and lifecycle (implemented in telnet_service.cpp); no dependency on the main.cpp
// globals.

#include <ESPTelnet.h>

#include "../service.h"

class TelnetService : public Service {
public:
    TelnetService() : Service("telnet", "/littlefs/conf/telnet.conf", /*requiresNetwork*/ true) {
    }

    [[nodiscard]] std::string statusDetail() const override {
        return const_cast<ESPTelnet &>(telnet).isConnected()
                   ? std::string(telnet.getIP().c_str())
                   : "";
    }

    // Reboot teardown helpers (see systemRestart): half-close the client (send FIN, keep the socket
    // readable so the close can be observed), then poll closePending() until the peer closes or a
    // timeout. Avoids leaving the telnet client half-open when our FIN is lost on a weak link.
    void beginClose();

    bool closePending() { return telnet.isConnected(); }

protected:
    bool onStart() override;

    void onStop() override;

    void onTick() override;

private:
    bool setupTelnet();

    void closeConnection();

    // ESPTelnet event handlers. Static (function-pointer callbacks), so they reach the server
    // through the global `telnetService` instance rather than `this`.
    static void onConnect(String ip);

    static void onDisconnect(String ip);

    ESPTelnet telnet;
};

inline TelnetService telnetService;

// Enqueue a telnet command line for deferred execution on loopNetwork_task (process_queued_tasks),
// rather than running handleCommand inline inside the ESPTelnet input callback (UAF on teardown
// commands). Implemented in telnet_service.cpp; exposed for the defer-not-inline regression test.
void telnetDispatchCommandAsync(const char *line);
