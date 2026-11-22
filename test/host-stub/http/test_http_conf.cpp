// Host e2e test for HttpConfServer: real loopback TCP against a temp-dir root.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <poll.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "http_conf.h"

static constexpr uint16_t PORT = 18087;
static std::string root, auth;
static int pass = 0, failed = 0;

#define CHECK(cond) do { if (cond) ++pass; else { ++failed; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static int dial() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(PORT);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (sockaddr *) &a, sizeof a)) { perror("connect"); return -1; }
    return fd;
}

static void sendAll(int fd, const std::string &s) {
    size_t o = 0;
    while (o < s.size()) {
        ssize_t n = send(fd, s.data() + o, s.size() - o, 0);
        if (n <= 0) return;
        o += n;
    }
}

// Reads until EOF or timeout; returns what arrived. *eof tells whether the server closed.
static std::string readAll(int fd, int timeoutMs = 3000, bool *eof = nullptr) {
    std::string r;
    char b[2048];
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    if (eof) *eof = false;
    while (std::chrono::steady_clock::now() < end) {
        pollfd p = {fd, POLLIN, 0};
        if (poll(&p, 1, 20) <= 0) continue;
        ssize_t n = recv(fd, b, sizeof b, 0);
        if (n <= 0) { if (eof) *eof = true; break; }
        r.append(b, n);
    }
    return r;
}

static int status(const std::string &r) { return r.size() > 12 ? atoi(r.c_str() + 9) : -1; }

static std::string body(const std::string &r) {
    auto p = r.find("\r\n\r\n");
    return p == std::string::npos ? "" : r.substr(p + 4);
}

static std::string req(const std::string &raw) {
    int fd = dial();
    sendAll(fd, raw);
    auto r = readAll(fd);
    close(fd);
    return r;
}

static std::string rq(const char *method, const std::string &target, const std::string &b = "",
                      bool withLen = true, const std::string &a = auth, const std::string &extra = "") {
    std::string s = std::string(method) + " " + target + " HTTP/1.1\r\nHost: x\r\n";
    if (!a.empty()) s += "Authorization: Basic " + a + "\r\n";
    if (withLen && (!b.empty() || !strcmp(method, "PUT"))) s += "Content-Length: " + std::to_string(b.size()) + "\r\n";
    return req(s + extra + "\r\n" + b);
}

static std::string slurp(const std::string &name) {
    std::ifstream f(root + "/" + name, std::ios::binary);
    if (!f) return "<missing>";
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void put(const std::string &name, const std::string &data) {
    std::ofstream(root + "/" + name, std::ios::binary) << data;
}

static bool exists(const std::string &name) {
    struct stat st;
    return !stat((root + "/" + name).c_str(), &st);
}

int main() {
    signal(SIGPIPE, SIG_IGN);
    char tmpl[] = "/tmp/httpconfXXXXXX";
    root = mkdtemp(tmpl);
    put("a.conf", "hello");
    put(".hidden", "x");
    mkdir((root + "/sub").c_str(), 0755);

    CHECK(HttpConfServer::basicToken("user", "pass") == "dXNlcjpwYXNz");
    CHECK(HttpConfServer::basicToken("u", "p1") == "dTpwMQ==");
    CHECK(HttpConfServer::basicToken("ab", "c") == "YWI6Yw==");
    CHECK(HttpConfServer::basicToken("ab", "cd") == "YWI6Y2Q=");
    auth = HttpConfServer::basicToken("admin", "s3cret");

    HttpConfServer srv;
    srv.timeoutMs = 1500;
    if (!srv.begin(PORT, root.c_str(), auth.c_str())) { fprintf(stderr, "begin failed\n"); return 1; }
    std::atomic<bool> run{true};
    std::thread t([&] { while (run) { srv.tick(); sleepMs(1); } });

    // auth
    auto r = rq("GET", "/conf/", "", true, "");
    CHECK(status(r) == 401);
    CHECK(r.find("WWW-Authenticate: Basic") != std::string::npos);
    r = rq("GET", "/conf/", "", true, HttpConfServer::basicToken("admin", "wrong"));
    CHECK(status(r) == 401);
    r = rq("GET", "/conf/", "", true, auth + "x");
    CHECK(status(r) == 401);
    r = req("GET /conf/a.conf HTTP/1.1\r\nauthorization: basic " + auth + "  \r\n\r\n");
    CHECK(status(r) == 200);

    // list + get
    r = rq("GET", "/conf/");
    CHECK(status(r) == 200);
    CHECK(body(r) == "a.conf\t5\n");
    CHECK(r.find("Access-Control-Allow-Origin: *") != std::string::npos);
    r = rq("GET", "/conf");
    CHECK(status(r) == 200 && body(r) == "a.conf\t5\n");
    r = rq("GET", "/conf/a.conf?t=1");
    CHECK(status(r) == 200 && body(r) == "hello");
    CHECK(r.find("Content-Length: 5\r\n") != std::string::npos);
    CHECK(status(rq("GET", "/conf/nope.conf")) == 404);
    CHECK(status(rq("GET", "/other")) == 404);
    CHECK(status(rq("GET", "/conference")) == 404);

    // put new, overwrite, large (chunked through the 1 KB buffer)
    r = rq("PUT", "/conf/b.conf", "k=v\n");
    CHECK(status(r) == 201 && slurp("b.conf") == "k=v\n");
    r = rq("PUT", "/conf/a.conf", "world!");
    CHECK(status(r) == 204 && slurp("a.conf") == "world!");
    std::string big;
    for (int i = 0; i < 12000; ++i) big += char('a' + i % 26);
    r = rq("PUT", "/conf/big.conf", big);
    CHECK(status(r) == 201 && slurp("big.conf") == big);
    r = rq("GET", "/conf/big.conf");
    CHECK(status(r) == 200 && body(r) == big);
    r = rq("PUT", "/conf/empty.conf", "");
    CHECK(status(r) == 201 && slurp("empty.conf").empty());
    {
        int fd = dial();
        sendAll(fd, "PUT /conf/e.conf HTTP/1.1\r\nAuthorization: Basic " + auth +
                    "\r\nContent-Length: 3\r\nExpect: 100-continue\r\n\r\n");
        CHECK(readAll(fd, 300) == "HTTP/1.1 100 Continue\r\n\r\n");
        sendAll(fd, "abc");
        CHECK(status(readAll(fd)) == 201 && slurp("e.conf") == "abc");
        close(fd);
    }

    // aborted upload: peer closes mid-body
    {
        int fd = dial();
        sendAll(fd, "PUT /conf/a.conf HTTP/1.1\r\nAuthorization: Basic " + auth +
                    "\r\nContent-Length: 100\r\n\r\npartial");
        sleepMs(50);
        CHECK(exists(".a.conf"));
        close(fd);
        sleepMs(100);
        CHECK(slurp("a.conf") == "world!");
        CHECK(!exists(".a.conf"));
    }
    // aborted upload: peer stalls mid-body until the timeout
    {
        int fd = dial();
        sendAll(fd, "PUT /conf/a.conf HTTP/1.1\r\nAuthorization: Basic " + auth +
                    "\r\nContent-Length: 100\r\n\r\nstall");
        bool eof;
        readAll(fd, 2500, &eof);
        CHECK(eof);
        close(fd);
        CHECK(slurp("a.conf") == "world!" && !exists(".a.conf"));
    }

    // delete
    CHECK(status(rq("DELETE", "/conf/b.conf")) == 204 && !exists("b.conf"));
    CHECK(status(rq("DELETE", "/conf/b.conf")) == 404);

    // names
    for (const char *bad: {"/conf/../x", "/conf/..", "/conf/.hidden", "/conf/a%2fb", "/conf/sub/x",
                           "/conf/0123456789012345678901234567890123"}) {
        r = rq("GET", bad);
        CHECK(status(r) == 400);
        if (status(r) != 400) fprintf(stderr, "  name %s -> %d\n", bad, status(r));
    }
    CHECK(status(rq("PUT", "/conf/../evil", "x")) == 400 && !exists("../evil"));
    CHECK(status(rq("GET", "/conf/01234567890123456789012345678901")) == 404); // 32 chars ok

    // limits
    r = rq("PUT", "/conf/huge.conf", std::string(16385, 'x'));
    CHECK(status(r) == 413 && !exists("huge.conf") && !exists(".huge.conf"));
    r = rq("PUT", "/conf/max.conf", std::string(16384, 'y'));
    CHECK(status(r) == 201 && slurp("max.conf").size() == 16384);
    CHECK(status(rq("PUT", "/conf/c.conf", "", false)) == 411);
    CHECK(status(rq("PUT", "/conf/c.conf", "", false, auth, "Transfer-Encoding: chunked\r\n")) == 411);
    CHECK(status(rq("PUT", "/conf/c.conf", "", false, auth, "Content-Length: -1\r\n")) == 400);
    CHECK(status(rq("PUT", "/conf/c.conf", "", false, auth, "Content-Length: 12x\r\n")) == 400);
    CHECK(status(req("GET /" + std::string(1100, 'a') + " HTTP/1.1\r\n\r\n")) == 414);
    CHECK(status(req("GET / HTTP/1.1\r\nX-Pad: " + std::string(1100, 'a') + "\r\n\r\n")) == 431);
    CHECK(status(req("GARBAGE\r\n\r\n")) == 400);

    // methods
    CHECK(status(rq("POST", "/conf/a.conf", "x")) == 405);
    CHECK(status(rq("DELETE", "/conf/")) == 405);
    r = req("OPTIONS /conf/a.conf HTTP/1.1\r\nOrigin: http://x\r\nAccess-Control-Request-Method: PUT\r\n\r\n");
    CHECK(status(r) == 204);
    CHECK(r.find("Access-Control-Allow-Methods: GET, PUT, DELETE") != std::string::npos);
    CHECK(r.find("Access-Control-Allow-Headers: Authorization, Content-Type") != std::string::npos);
    CHECK(r.find("Access-Control-Allow-Origin: *") != std::string::npos);

    // 2 concurrent + 3rd queued until a slot frees
    {
        std::string get = "GET /conf/a.conf HTTP/1.1\r\nAuthorization: Basic " + auth + "\r\n";
        int c1 = dial(), c2 = dial();
        sendAll(c1, get);
        sendAll(c2, get);
        sleepMs(50);
        CHECK(srv.clients() == 2);
        int c3 = dial();
        CHECK(c3 >= 0);
        sendAll(c3, get + "\r\n");
        auto r3 = readAll(c3, 300);
        CHECK(r3.empty());
        sendAll(c1, "\r\n");
        auto r1 = readAll(c1);
        CHECK(status(r1) == 200 && body(r1) == "world!");
        r3 += readAll(c3);
        CHECK(status(r3) == 200 && body(r3) == "world!");
        sendAll(c2, "\r\n");
        CHECK(status(readAll(c2)) == 200);
        close(c1);
        close(c2);
        close(c3);
    }

    // slow loris: header dribbled, server closes after timeoutMs
    {
        int fd = dial();
        auto t0 = std::chrono::steady_clock::now();
        bool eof = false;
        for (int i = 0; i < 40 && !eof; ++i) {
            sendAll(fd, "G");
            readAll(fd, 100, &eof);
        }
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        CHECK(eof);
        CHECK(ms >= 1400 && ms < 2500);
        close(fd);
    }
    sleepMs(50);
    CHECK(srv.clients() == 0);

    run = false;
    t.join();
    srv.end();
    std::string cmd = "rm -rf " + root;
    system(cmd.c_str());
    printf("%d passed, %d failed\n", pass, failed);
    return failed != 0;
}
