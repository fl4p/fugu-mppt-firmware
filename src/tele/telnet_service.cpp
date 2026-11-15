#include "telnet_service.h"

#include <Arduino.h> // String, Serial
#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>

#include "../logging.h"   // set_logging_telnet
#include "../etc/version.h"
#include "telemetry.h"    // getHostname, handleCommand

extern time_us lastTimeOutUs;

void telnetDispatchCommandAsync(const char *line) {
    // Defer to loopNetwork_task (where process_queued_tasks runs). Running handleCommand inline
    // inside the ESPTelnet input callback lets a teardown command (wifi off, svc off telnet,
    // restart) stop/destroy this same ESPTelnet from within its own callback -> UAF. Mirror the
    // MQTT cmd_input path. The OK/ERR confirmation rides the log mux (telnet is a sink via
    // set_logging_telnet), so the deferred task no longer touches the telnet object.
    std::string cmd(line);
    enqueue_task([cmd]() {
        String inp(cmd.c_str());
        inp.trim();
        if (!inp.length()) return;
        const auto t0 = esp_timer_get_time();
        logCmdResult(handleCommand(inp), inp.c_str(), t0);
    });
}

void TelnetService::closeConnection() {
    if (telnet.isConnected()) {
        telnet.flush();
        telnet.disconnectClient();
    }
}

void TelnetService::beginClose() {
    if (!telnet.isConnected()) return;
    telnet.flush();
    // Half-close: send our FIN now but keep the socket readable so closePending() can watch the peer
    // close in response. A full disconnectClient() here would shut the fd and abort that observation.
    shutdown(telnet.getClient().fd(), SHUT_WR);
}

bool TelnetService::onStart() {
    if (!WiFi.isConnected()) return false;
    return setupTelnet();
}

void TelnetService::onStop() {
    // Must drop the log sink before the server dies, or logging.cpp keeps writing to a stopped
    // ESPTelnet (telnet doubles as a log sink, see onConnect / set_logging_telnet).
    set_logging_telnet(nullptr);
    closeConnection();
    telnet.stop(true);
}

void TelnetService::onTick() { telnet.loop(); }

bool TelnetService::setupTelnet() {
    telnet.onConnect(onConnect);
    telnet.onDisconnect(onDisconnect);
    telnet.onInputReceived([](String str) {
        // Captureless lambda (function-pointer callback) -> reach the server via the global service.
        auto &t = telnetService.telnet;
        if (str == "ping") {
            t.println("> pong");
            Serial.println("- Telnet: pong");
        } else if (str == "exit") {
            t.println("goodbye!");
            t.flush();
            t.disconnectClient();
        } else {
            telnetDispatchCommandAsync(str.c_str());
        }
    });

    Serial.print("- Telnet: ");
    telnet.stop();
    if (telnet.begin(23)) {
        MDNS.addService("telnet", "tcp", 23);
        ESP_LOGI("telnet", "Telnet server running.");
        return true;
    }
    ESP_LOGE("telnet", "Telnet server start error");
    return false;
}

void TelnetService::onConnect(String ip) {
    auto &t = telnetService.telnet;
    ESP_LOGI("telnet", "Client %s connected", ip.c_str());
    t.println("\nWelcome to " + String(getHostname().c_str()) + " (" +
              WiFi.localIP().toString().c_str() + ")");
    t.println(format_version());
    t.println("(Use ^] + q  to disconnect.)");

    set_logging_telnet(&t);
    lastTimeOutUs = 0; // trigger output
}

void TelnetService::onDisconnect(String ip) {
    set_logging_telnet(nullptr);
    ESP_LOGI("telnet", "Client %s disconnected", ip.c_str());
}
