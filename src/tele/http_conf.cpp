#include "http_conf.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <netinet/in.h>

#include <esp_log.h>
#include <esp_timer.h>

static const char *TAG = "http";

static bool wouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }

static bool validName(const char *s) {
    size_t n = 0;
    for (; s[n]; ++n) {
        char ch = s[n];
        if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
              ch == '.' || ch == '_' || ch == '-'))
            return false;
    }
    return n && n <= HttpConfServer::MAX_NAME && s[0] != '.';
}

static const char *reason(int code) {
    switch (code) {
        case 200: return "OK";
        case 201: return "Created";
        case 204: return "No Content";
        case 400: return "Bad Request";
        case 401: return "Unauthorized";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 411: return "Length Required";
        case 413: return "Payload Too Large";
        case 414: return "URI Too Long";
        case 431: return "Request Header Fields Too Large";
        default: return "Internal Server Error";
    }
}

std::string HttpConfServer::basicToken(const std::string &user, const std::string &pass) {
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string in = user + ':' + pass, out;
    for (size_t i = 0; i < in.size(); i += 3) {
        uint32_t v = (uint8_t) in[i] << 16;
        size_t r = in.size() - i;
        if (r > 1) v |= (uint8_t) in[i + 1] << 8;
        if (r > 2) v |= (uint8_t) in[i + 2];
        for (int k = 0; k < 4; ++k)
            out += k <= (int) (r > 2 ? 3 : r) ? T[(v >> (18 - 6 * k)) & 63] : '=';
    }
    return out;
}

bool HttpConfServer::begin(uint16_t port, const char *root, const char *auth) {
    end();
    _root = root;
    _auth = auth;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (sockaddr *) &a, sizeof a) || listen(fd, MAX_CONN) ||
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "listen on %u failed: %d", port, errno);
        close(fd);
        return false;
    }
    _lfd = fd;
    return true;
}

void HttpConfServer::end() {
    for (auto &c: _c) if (c.fd >= 0) drop(c);
    if (_lfd >= 0) close(_lfd);
    _lfd = -1;
}

int HttpConfServer::clients() const {
    int n = 0;
    for (auto &c: _c) n += c.fd >= 0;
    return n;
}

void HttpConfServer::tick() {
    if (_lfd < 0) return;
    fd_set rd, wr;
    FD_ZERO(&rd);
    FD_ZERO(&wr);
    int mx = -1;
    bool slot = clients() < MAX_CONN;
    if (slot) {
        FD_SET(_lfd, &rd);
        mx = _lfd;
    }
    for (auto &c: _c)
        if (c.fd >= 0) {
            FD_SET(c.fd, c.state == Send ? &wr : &rd);
            if (c.fd > mx) mx = c.fd;
        }
    if (mx < 0) return;
    timeval tv = {};
    int n = select(mx + 1, &rd, &wr, nullptr, &tv);
    _now = (uint32_t) (esp_timer_get_time() / 1000);
    for (auto &c: _c) {
        if (c.fd < 0) continue;
        if (n > 0 && (FD_ISSET(c.fd, &rd) || FD_ISSET(c.fd, &wr))) step(c);
        if (c.fd >= 0 && (int32_t) (_now - c.deadline) > 0) drop(c);
    }
    if (n > 0 && slot && FD_ISSET(_lfd, &rd)) accept_();
}

void HttpConfServer::accept_() {
    int fd = accept(_lfd, nullptr, nullptr);
    if (fd < 0) return; // e.g. LWIP_MAX_SOCKETS exhausted
    for (auto &c: _c)
        if (c.fd < 0) {
            c.buf = (char *) malloc(BUF_SIZE);
            if (!c.buf) break;
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
            c.fd = fd;
            c.state = RecvHdr;
            c.len = 0;
            c.deadline = _now + timeoutMs;
            return;
        }
    close(fd);
}

void HttpConfServer::drop(Conn &c) {
    if (c.file >= 0) close(c.file);
    if (c.state == RecvBody) {
        char p[80];
        path(p, sizeof p, c.name, true);
        unlink(p);
        ESP_LOGW(TAG, "PUT %s aborted", c.name);
    }
    if (c.dir) closedir(c.dir);
    close(c.fd);
    free(c.buf);
    c = Conn{};
}

void HttpConfServer::path(char *out, size_t sz, const char *name, bool tmp) const {
    snprintf(out, sz, "%s/%s%s", _root, tmp ? "." : "", name);
}

void HttpConfServer::step(Conn &c) {
    int n;
    switch (c.state) {
        case RecvHdr: {
            n = recv(c.fd, c.buf + c.len, BUF_SIZE - 1 - c.len, 0);
            if (n <= 0) {
                if (n < 0 && wouldBlock()) return;
                return drop(c);
            }
            c.len += n;
            c.buf[c.len] = 0;
            char *e = strstr(c.buf, "\r\n\r\n");
            if (e) {
                c.deadline = _now + timeoutMs;
                handle(c, e + 4);
            } else if (c.len >= BUF_SIZE - 1)
                reply(c, strstr(c.buf, "\r\n") ? 431 : 414);
            return;
        }
        case RecvBody:
            n = recv(c.fd, c.buf, c.left < BUF_SIZE ? c.left : BUF_SIZE, 0);
            if (n <= 0) {
                if (n < 0 && wouldBlock()) return;
                return drop(c);
            }
            return writeBody(c, c.buf, n);
        case Send:
            n = send(c.fd, c.buf + c.off, c.len - c.off, 0);
            if (n < 0) {
                if (wouldBlock()) return;
                return drop(c);
            }
            c.off += n;
            if (c.off == c.len) refill(c);
            return;
        default: // Drain: discard until the peer closes, so close() doesn't RST the response
            n = recv(c.fd, c.buf, BUF_SIZE, 0);
            if (n == 0 || (n < 0 && !wouldBlock())) drop(c);
    }
}

bool HttpConfServer::authOk(const char *h) const {
    if (!h || strncasecmp(h, "Basic ", 6)) return false;
    h += 6;
    size_t n = strlen(h), m = strlen(_auth);
    while (n && h[n - 1] == ' ') --n;
    uint8_t d = n != m;
    for (size_t i = 0; i < n && i < m; ++i) d |= h[i] ^ _auth[i];
    return !d;
}

void HttpConfServer::handle(Conn &c, char *body) {
    size_t extra = c.buf + c.len - body;
    char *target = strchr(c.buf, ' ');
    char *ver = target ? strchr(target + 1, ' ') : nullptr;
    if (!ver) return reply(c, 400);
    *target++ = 0;
    *ver = 0;
    const char *auth = nullptr;
    long clen = -1;
    bool expect = false;
    for (char *l = strstr(ver + 1, "\r\n") + 2; l < body - 2;) {
        char *eol = strstr(l, "\r\n");
        *eol = 0;
        char *v = strchr(l, ':');
        if (v) {
            ++v;
            while (*v == ' ' || *v == '\t') ++v;
            if (!strncasecmp(l, "authorization:", 14)) auth = v;
            else if (!strncasecmp(l, "expect:", 7)) expect = true;
            else if (!strncasecmp(l, "content-length:", 15)) {
                char *end;
                clen = strtol(v, &end, 10);
                while (*end == ' ' || *end == '\t') ++end;
                if (end == v || *end || clen < 0) return reply(c, 400);
            }
        }
        l = eol + 2;
    }

    const char *m = c.buf;
    if (!strcmp(m, "OPTIONS"))
        return reply(c, 204, "Access-Control-Allow-Methods: GET, PUT, DELETE\r\n"
                             "Access-Control-Allow-Headers: Authorization, Content-Type\r\n"
                             "Access-Control-Allow-Private-Network: true\r\n");
    if (!authOk(auth)) return reply(c, 401, "WWW-Authenticate: Basic realm=\"conf\"\r\n");

    if (char *q = strchr(target, '?')) *q = 0;
    if (strncmp(target, "/conf", 5) || (target[5] && target[5] != '/')) return reply(c, 404);
    const char *name = target[5] ? target + 6 : "";
    bool get = !strcmp(m, "GET");

    if (!*name) {
        if (!get) return reply(c, 405);
        c.dir = opendir(_root);
        if (!c.dir) return reply(c, 500);
        return reply(c, 200, "Content-Type: text/plain\r\n", -1);
    }
    if (!validName(name)) return reply(c, 400);

    char p[80];
    path(p, sizeof p, name, false);
    struct stat st;
    bool exists = !stat(p, &st);
    if (get) {
        if (!exists) return reply(c, 404);
        c.file = open(p, O_RDONLY);
        if (c.file < 0) return reply(c, 500);
        c.left = st.st_size;
        return reply(c, 200, "Content-Type: text/plain\r\n", st.st_size);
    }
    if (!strcmp(m, "DELETE")) {
        ESP_LOGI(TAG, "DELETE %s", name);
        return reply(c, !unlink(p) ? 204 : errno == ENOENT ? 404 : 500);
    }
    if (strcmp(m, "PUT")) return reply(c, 405);
    if (clen < 0) return reply(c, 411);
    if (clen > (long) MAX_BODY) return reply(c, 413);
    path(p, sizeof p, name, true);
    c.file = open(p, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (c.file < 0) return reply(c, 500);
    strcpy(c.name, name);
    c.created = !exists;
    c.left = clen;
    c.state = RecvBody;
    if (expect && !extra) send(c.fd, "HTTP/1.1 100 Continue\r\n\r\n", 25, 0);
    writeBody(c, body, extra < (size_t) clen ? extra : clen);
}

void HttpConfServer::writeBody(Conn &c, const char *p, size_t n) {
    if (n && write(c.file, p, n) != (ssize_t) n) {
        close(c.file);
        c.file = -1;
        char t[80];
        path(t, sizeof t, c.name, true);
        unlink(t);
        return reply(c, 500);
    }
    c.left -= n;
    if (!c.left) finishPut(c);
}

void HttpConfServer::finishPut(Conn &c) {
    char tmp[80], dst[80];
    path(tmp, sizeof tmp, c.name, true);
    path(dst, sizeof dst, c.name, false);
    bool ok = !fsync(c.file);
    ok &= !close(c.file);
    c.file = -1;
    ok = ok && !rename(tmp, dst);
    if (!ok) unlink(tmp);
    ESP_LOGI(TAG, "PUT %s %s", c.name, ok ? "ok" : "failed");
    reply(c, ok ? (c.created ? 201 : 204) : 500);
}

void HttpConfServer::reply(Conn &c, int code, const char *hdr, long clen) {
    int n = snprintf(c.buf, BUF_SIZE, "HTTP/1.1 %d %s\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n%s",
                     code, reason(code), hdr);
    if (clen >= 0) n += snprintf(c.buf + n, BUF_SIZE - n, "Content-Length: %ld\r\n", clen);
    n += snprintf(c.buf + n, BUF_SIZE - n, "\r\n");
    if (code >= 400) ESP_LOGW(TAG, "-> %d", code);
    c.len = n;
    c.off = 0;
    c.state = Send;
}

void HttpConfServer::refill(Conn &c) {
    c.off = c.len = 0;
    if (c.file >= 0 && c.left) {
        int n = read(c.file, c.buf, c.left < BUF_SIZE ? c.left : BUF_SIZE);
        if (n <= 0) return drop(c);
        c.len = n;
        c.left -= n;
        return;
    }
    if (c.dir) {
        dirent *e;
        while (c.len < BUF_SIZE - 96 && (e = readdir(c.dir))) {
            if (e->d_name[0] == '.' || strlen(e->d_name) > MAX_NAME) continue;
            char p[80];
            path(p, sizeof p, e->d_name, false);
            struct stat st;
            if (stat(p, &st) || !S_ISREG(st.st_mode)) continue;
            c.len += snprintf(c.buf + c.len, BUF_SIZE - c.len, "%s\t%ld\n", e->d_name, (long) st.st_size);
        }
        if (c.len) return;
        closedir(c.dir);
        c.dir = nullptr;
    }
    if (c.file >= 0) close(c.file);
    c.file = -1;
    shutdown(c.fd, SHUT_WR);
    c.state = Drain;
    c.deadline = _now + 1000;
}
