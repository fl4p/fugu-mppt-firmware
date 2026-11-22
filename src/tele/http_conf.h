#pragma once

// Minimal non-blocking HTTP/1.1 file server for one flat directory, polled from tick() (no task).
// GET /conf/ lists "name<TAB>size" lines, GET/PUT/DELETE /conf/<name> serve, replace (via a temp
// file renamed on full receipt) and delete files. HTTP Basic auth, Connection: close, CORS *.

#include <cstdint>
#include <cstddef>
#include <string>
#include <dirent.h>

class HttpConfServer {
public:
    static constexpr int MAX_CONN = 2;
    static constexpr size_t BUF_SIZE = 1024;
    static constexpr uint32_t MAX_BODY = 16384;
    static constexpr size_t MAX_NAME = 32;

    uint32_t timeoutMs = 10000;

    // root and auth are kept by pointer; auth is base64("user:pass") (see basicToken()).
    bool begin(uint16_t port, const char *root, const char *auth);

    void end();

    void tick();

    int clients() const;

    static std::string basicToken(const std::string &user, const std::string &pass);

private:
    enum : uint8_t { RecvHdr = 1, RecvBody, Send, Drain };

    struct Conn {
        int fd = -1, file = -1;
        DIR *dir = nullptr;
        char *buf = nullptr;
        uint16_t len = 0, off = 0;
        uint32_t left = 0, deadline = 0;
        uint8_t state = 0;
        bool created = false;
        char name[MAX_NAME + 1] = {};
    };

    int _lfd = -1;
    uint32_t _now = 0;
    const char *_root = nullptr, *_auth = nullptr;
    Conn _c[MAX_CONN];

    void accept_();
    void step(Conn &c);
    void handle(Conn &c, char *body);
    void writeBody(Conn &c, const char *p, size_t n);
    void finishPut(Conn &c);
    void refill(Conn &c);
    void reply(Conn &c, int code, const char *hdr = "", long clen = 0);
    void drop(Conn &c);
    bool authOk(const char *h) const;
    void path(char *out, size_t sz, const char *name, bool tmp) const;
};
