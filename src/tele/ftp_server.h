#pragma once

// Small non-blocking FTP server on BSD sockets and the POSIX file API. No task of its own: poll()
// does a bounded amount of work per call (one command and one buffer chunk per session).
// The passive data port of session i is pasvPort + i, so sessions never share a data listener.

#include <cstdint>
#include <string>

#include <sys/select.h>

class FtpServer {
public:
    static constexpr uint8_t MAX_SESSIONS = 2;
    static constexpr uint16_t CMD_MAX = 256, PATH_LEN = 192, BUF = 1024;

    struct Timeouts {
        uint32_t authMs = 10000, idleMs = 300000, dataMs = 10000;
    } timeouts;

    ~FtpServer() { end(); }

    // root: directory shown as "/" (copied, < 32 chars)
    bool begin(const char *user, const char *pass, const char *root, uint16_t port = 21, uint16_t pasvPort = 50009);

    void end();

    void poll(uint32_t nowMs);

    // comma separated peer IPs of the connected sessions
    [[nodiscard]] std::string clients() const;

private:
    struct Session;

    void accept(uint32_t now);

    void closeSession(uint8_t i);

    bool service(Session &s, fd_set &rd, fd_set &wr, uint32_t now);

    bool command(Session &s, uint32_t c, char *arg, uint32_t now);

    bool toFs(Session &s, const char *arg, char *fs, char *v = nullptr);

    void xfer(Session &s, char *arg, uint8_t kind);

    void pump(Session &s, uint32_t now);

    uint16_t list(Session &s);

    void finish(Session &s, bool ok, const char *err);

    void tmpPath(const Session &s, char *out) const;

    Session *_s[MAX_SESSIONS]{};
    std::string _user, _pass, _root;
    int _lfd = -1;
    uint16_t _pasvPort = 0;
};
