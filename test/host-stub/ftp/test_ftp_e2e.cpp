// Host-side end-to-end tests for src/tele/ftp_server.cpp. The server is compiled natively
// (POSIX sockets + file API), serves a fresh mkdtemp() root on loopback ports and is driven over
// raw TCP from forked test processes. See SPEC.md.
//
// Build: cmake -S test/host-stub/ftp -B test/host-stub/ftp/build && cmake --build test/host-stub/ftp/build

#include <arpa/inet.h>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "../../../src/tele/ftp_server.h"

namespace {

constexpr uint16_t kCmdPort  = 12121;
constexpr uint16_t kPasvPort = 12122;
constexpr const char *USER = "ftp";
constexpr const char *PASS = "secret";

FtpServer server;
std::mutex serverMu;
std::atomic<bool> running{true};

uint32_t nowMs() {
    using namespace std::chrono;
    return (uint32_t) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void ftpLoop() {
    while (running.load(std::memory_order_relaxed)) {
        {
            std::lock_guard<std::mutex> lk(serverMu);
            server.poll(nowMs());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Tiny FTP client over raw TCP. readCode() returns the 3-digit code of the
// next complete reply, honoring RFC 959 multi-line continuations
// ("ddd-..." lines until "ddd ...").
struct FtpDial {
    int fd = -1;
    std::string buf;
    std::string last; // last reply line

    explicit FtpDial(uint16_t port = kCmdPort) {
        fd = ::socket(AF_INET, SOCK_STREAM, 0);
        assert(fd >= 0);
        sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_port   = htons(port);
        sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        int rc = ::connect(fd, (sockaddr *)&sa, sizeof(sa));
        assert(rc == 0 && "connect to FTP control socket failed");
        timeval tv{0, 200000};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    ~FtpDial() { if (fd >= 0) ::close(fd); }

    void send(const std::string &line) {
        std::string s = line + "\r\n";
        ssize_t n = ::send(fd, s.data(), s.size(), 0);
        (void)n;
    }

    // -1 on timeout, -2 once the peer closed with nothing pending
    int readCode(int timeoutMs = 3000) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        int lastCode = -1;
        while (std::chrono::steady_clock::now() < deadline) {
            for (;;) {
                auto nl = buf.find('\n');
                if (nl == std::string::npos) break;
                std::string line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (line.size() >= 4) {
                    int code = std::atoi(line.substr(0, 3).c_str());
                    char sep = line[3];
                    if (code > 0) {
                        lastCode = code;
                        if (sep == ' ') { last = line; return code; }
                    }
                }
            }
            char tmp[256];
            ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
            if (n > 0) buf.append(tmp, tmp + (size_t)n);
            else if (n == 0) return lastCode == -1 ? -2 : lastCode;
        }
        return lastCode;
    }

    int login() {
        if (readCode() != 220) return -1;
        send(std::string("USER ") + USER);
        if (readCode() != 331) return -1;
        send(std::string("PASS ") + PASS);
        return readCode();
    }

    // PASV -> port from "227 ... (h1,h2,h3,h4,p1,p2)", 0 on failure
    uint16_t pasv() {
        send("PASV");
        if (readCode() != 227) return 0;
        auto lp = last.find('(');
        unsigned a, b, c, d, p1, p2;
        if (lp == std::string::npos ||
            std::sscanf(last.c_str() + lp + 1, "%u,%u,%u,%u,%u,%u", &a, &b, &c, &d, &p1, &p2) != 6) return 0;
        return (uint16_t)((p1 << 8) | p2);
    }

    // EPSV -> port from "229 ... (|||port|)", 0 on failure
    uint16_t epsv() {
        send("EPSV");
        if (readCode() != 229) return 0;
        auto lp = last.find("(|||");
        unsigned p;
        if (lp == std::string::npos || std::sscanf(last.c_str() + lp + 4, "%u", &p) != 1) return 0;
        return (uint16_t) p;
    }
};

// Open a fresh TCP socket to 127.0.0.1:port for the FTP data channel.
int openData(uint16_t port) {
    int dfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (dfd < 0) return -1;
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(dfd, (sockaddr *)&sa, sizeof(sa)) < 0) { ::close(dfd); return -1; }
    timeval tv{5, 0};
    ::setsockopt(dfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return dfd;
}

// Drain everything readable on a data socket until peer closes; return all
// bytes received.
std::string drainData(int dfd) {
    std::string out;
    char buf[1024];
    for (;;) {
        ssize_t n = ::recv(dfd, buf, sizeof(buf), 0);
        if (n > 0) out.append(buf, buf + n);
        else break;  // 0 = peer closed; -1 = timeout
    }
    return out;
}

void sendAll(int dfd, const std::string &s) {
    size_t off = 0;
    while (off < s.size()) {
        ssize_t n = ::send(dfd, s.data() + off, s.size() - off, 0);
        assert(n > 0);
        off += (size_t)n;
    }
}

std::string pattern(size_t n, unsigned seed) {
    std::string p(n, '\0');
    for (size_t i = 0; i < n; ++i) p[i] = (char)((i * 7 + seed) % 251);
    return p;
}

// PASV + STOR of a whole payload
void put(FtpDial &c, const std::string &path, const std::string &payload) {
    uint16_t pport = c.pasv();        assert(pport > 0);
    int dfd = openData(pport);        assert(dfd >= 0);
    c.send("STOR " + path);           assert(c.readCode() == 150);
    sendAll(dfd, payload);
    ::close(dfd);
    assert(c.readCode() == 226);
}

// PASV + RETR, returns the bytes
std::string get(FtpDial &c, const std::string &path) {
    uint16_t pport = c.pasv();        assert(pport > 0);
    int dfd = openData(pport);        assert(dfd >= 0);
    c.send("RETR " + path);
    int code = c.readCode();
    if (code != 150) { ::close(dfd); return "<" + std::to_string(code) + ">"; }
    std::string got = drainData(dfd);
    ::close(dfd);
    assert(c.readCode() == 226);
    return got;
}

// ---- regressions carried over from the SimpleFTPServer fork ---------------------------------------------------

// PASS-without-USER bypass (a SimpleFTPServer bug once replied 503 and then
// authenticated anyway, queuing 503 + 230): PASS must not log in and PWD
// must not succeed.
void test_pass_without_user() {
    FtpDial c(kCmdPort);
    assert(c.readCode() == 220);

    c.send(std::string("PASS ") + PASS);

    // Read all replies the server sends in response to the bare PASS. On the
    // patched code this is exactly one (503). On buggy code there are two
    // (503 then 230). A 230 here directly proves the bypass.
    int first  = c.readCode();
    int second = c.readCode();
    std::fprintf(stderr, "  PASS -> first=%d, second=%d (patched: 503/-1; buggy: 503/230)\n",
                 first, second);
    assert(first == 503 && "PASS without USER must be rejected with 503");
    assert(second != 230 && "PASS-before-USER must NOT also send 230 (bypass)");

    // Belt-and-braces: a follow-up PWD must NOT succeed (257).
    c.send("PWD");
    int pwdCode = c.readCode();
    std::fprintf(stderr, "  PWD->%d (must NOT be 257)\n", pwdCode);
    assert(pwdCode != 257 && "PASS-before-USER must not authenticate the session");
}

// Path overflow: cwd (101 B) + "/" + a 200 B relative arg exceeds the
// path buffer (FtpServer::PATH_LEN) while the line fits CMD_MAX. Must be
// rejected with 500 and leave the server alive.
void test_long_cwd_path() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    // 100-char base dir → workingDir becomes "/" + 100 chars = 101 B.
    std::string base(100, 'B');
    c.send("MKD /" + base);  (void)c.readCode();  // 257 or 550 — don't care
    c.send("CWD /" + base);
    int cwdOk = c.readCode();
    std::fprintf(stderr, "  CWD long base ->%d\n", cwdOk);
    assert((cwdOk == 250 || cwdOk == 257) && "setup CWD failed");

    // 200-char relative arg fits in cmd line ("CWD " + 200 = 204 < 263) but
    // combined with the 101-B workingDir blows the 263-B path buffer.
    std::string huge(200, 'A');
    c.send("CWD " + huge);
    int code = c.readCode();
    std::fprintf(stderr, "  CWD long arg ->%d (expect 500)\n", code);
    assert(code == 500 && "long CWD must be rejected with 500, not overflow");

    c.send("NOOP");
    int noop = c.readCode();
    std::fprintf(stderr, "  NOOP->%d (server must stay alive)\n", noop);
    assert(noop == 200 && "server must remain responsive after long CWD");
}

// Bug #3: PORT NULL deref. "PORT 1" crashed the server. Patched: 501.
void test_malformed_port() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    c.send("PORT 1");
    int code = c.readCode();
    std::fprintf(stderr, "  PORT 1->%d (expect 501)\n", code);
    assert(code == 501 && "malformed PORT must return 501, not crash");

    c.send("NOOP");
    assert(c.readCode() == 200);
}

// Bug #4: FTP-bounce. PORT to a non-peer IP must be refused.
void test_port_bounce() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    c.send("PORT 192,168,1,1,0,21");
    int code = c.readCode();
    std::fprintf(stderr, "  PORT 192.168.1.1->%d (expect 501)\n", code);
    assert(code == 501 && "PORT to non-peer IP must be refused");
}

// ---- data-channel coverage -------------------------------------------------

// STOR then RETR a small file. Verifies the full passive-mode data path
// (PASV → 227, parse port, open data conn, STOR → 150 → bytes → 226, then
// RETR → 150 → bytes → 226).
void test_upload_download() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    const std::string payload = "hello fugu ftp host-shim\n";

    // --- Upload ---
    uint16_t pport = c.pasv();
    std::fprintf(stderr, "  PASV->%u\n", pport);
    assert(pport > 0 && "PASV must advertise a data port");
    int dfd = openData(pport);
    assert(dfd >= 0 && "data socket connect failed");

    c.send("STOR /upload.bin");
    int stor = c.readCode();
    std::fprintf(stderr, "  STOR ->%d (expect 150)\n", stor);
    assert(stor == 150);

    ssize_t sent = ::send(dfd, payload.data(), payload.size(), 0);
    assert(sent == (ssize_t)payload.size());
    ::close(dfd);  // FIN triggers server to finalize and send 226

    int done = c.readCode();
    std::fprintf(stderr, "  STOR done ->%d (expect 226)\n", done);
    assert(done == 226);

    // --- Download (round-trip what we just uploaded) ---
    pport = c.pasv();
    assert(pport > 0);
    dfd = openData(pport);
    assert(dfd >= 0);

    c.send("RETR /upload.bin");
    int retr = c.readCode();
    std::fprintf(stderr, "  RETR ->%d (expect 150)\n", retr);
    assert(retr == 150);

    std::string got = drainData(dfd);
    ::close(dfd);
    std::fprintf(stderr, "  RETR got %zu bytes (expect %zu)\n", got.size(), payload.size());
    assert(got == payload && "RETR payload mismatch");

    int complete = c.readCode();
    std::fprintf(stderr, "  RETR done ->%d (expect 226)\n", complete);
    assert(complete == 226);
}

// MLSD with an explicit pathname argument (recently added upstream as
// commit 488d4d4 "Honor optional pathname argument in LIST/NLST/MLSD").
// Setup: create /mlsdsub/ and drop a marker file in it. Verify MLSD /mlsdsub
// lists the marker — not whatever's at cwd.
void test_mlsd_subdir() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    c.send("MKD /mlsdsub");
    (void)c.readCode();  // 257 if fresh, 550 if it survived from a prior run

    // Drop a marker file inside the subdir.
    uint16_t pport = c.pasv();
    assert(pport > 0);
    int dfd = openData(pport);
    assert(dfd >= 0);
    c.send("STOR /mlsdsub/marker.txt");
    assert(c.readCode() == 150);
    const char *payload = "x";
    ::send(dfd, payload, 1, 0);
    ::close(dfd);
    assert(c.readCode() == 226);

    // MLSD /mlsdsub should list marker.txt — not the contents of cwd.
    pport = c.pasv();
    assert(pport > 0);
    dfd = openData(pport);
    assert(dfd >= 0);

    c.send("MLSD /mlsdsub");
    int mlsd = c.readCode();
    std::fprintf(stderr, "  MLSD /mlsdsub ->%d (expect 150)\n", mlsd);
    assert(mlsd == 150);

    std::string listing = drainData(dfd);
    ::close(dfd);
    std::fprintf(stderr, "  MLSD listing (%zu B): %.120s\n", listing.size(), listing.c_str());
    assert(c.readCode() == 226);

    assert(listing.find("marker.txt") != std::string::npos
           && "MLSD with subdir arg must list files in that subdir");
    assert(listing.find("mlsdsub") == std::string::npos
           && "MLSD listing must contain only entries *inside* the subdir, not the subdir name itself");
}

// CWD "../" from root must stay at root.
void test_cwd_dotdot_clamps_at_root() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    c.send("CWD /");                     assert(c.readCode() == 250);
    c.send("PWD");
    // PWD reply is "257 \"/\" is the current directory."
    int pwd0 = c.readCode();
    std::fprintf(stderr, "  PWD->%d at root\n", pwd0);
    assert(pwd0 == 257);

    c.send("CWD ..");                    int cwdUp = c.readCode();
    std::fprintf(stderr, "  CWD .. ->%d (from /)\n", cwdUp);
    assert(cwdUp == 250 || cwdUp == 550);  // either succeed-as-noop or refuse

    c.send("PWD");                       int pwd1 = c.readCode();
    std::fprintf(stderr, "  PWD->%d after CWD ..\n", pwd1);
    assert(pwd1 == 257);
    assert(c.last.rfind("257 \"/\"", 0) == 0 && "CWD .. from / must stay at /");
}

// Multi-chunk STOR + RETR: 10 KB crosses the 1 KB transfer buffer
// (FtpServer::BUF) several times.
void test_large_file_round_trip() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    // 10 KB deterministic pattern; every byte distinct mod 251 makes
    // any off-by-one corruption easy to spot.
    std::string payload(10 * 1024, '\0');
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = (char)(i % 251);

    // Upload
    uint16_t pport = c.pasv();        assert(pport > 0);
    int dfd = openData(pport);        assert(dfd >= 0);
    c.send("STOR /large.bin");        assert(c.readCode() == 150);
    // Drain-as-we-write to avoid stalling on the kernel send buffer.
    size_t off = 0;
    while (off < payload.size()) {
        ssize_t n = ::send(dfd, payload.data() + off, payload.size() - off, 0);
        assert(n > 0);
        off += (size_t)n;
    }
    ::close(dfd);
    assert(c.readCode() == 226);

    // Download
    pport = c.pasv();                 assert(pport > 0);
    dfd = openData(pport);            assert(dfd >= 0);
    c.send("RETR /large.bin");        assert(c.readCode() == 150);
    std::string got = drainData(dfd);
    ::close(dfd);
    assert(c.readCode() == 226);

    std::fprintf(stderr, "  got %zu bytes (expect %zu)\n", got.size(), payload.size());
    assert(got.size() == payload.size() && "RETR size mismatch on 10 KB");
    assert(got == payload && "RETR content mismatch on 10 KB");
}

// DELE happy path + verify a follow-up RETR fails with the not-found code.
void test_dele_then_retr_fails() {
    FtpDial c(kCmdPort);
    assert(c.login() == 230);

    // Upload a victim file
    uint16_t pport = c.pasv();        assert(pport > 0);
    int dfd = openData(pport);        assert(dfd >= 0);
    c.send("STOR /todelete.bin");     assert(c.readCode() == 150);
    const char *body = "doomed";
    ::send(dfd, body, std::strlen(body), 0);
    ::close(dfd);
    assert(c.readCode() == 226);

    // Delete it
    c.send("DELE /todelete.bin");
    int del = c.readCode();
    std::fprintf(stderr, "  DELE ->%d (expect 250)\n", del);
    assert(del == 250);

    // Now RETR must fail with 550
    pport = c.pasv();                 assert(pport > 0);
    dfd = openData(pport);            assert(dfd >= 0);
    c.send("RETR /todelete.bin");
    int retr = c.readCode();
    ::close(dfd);
    std::fprintf(stderr, "  RETR-deleted ->%d (expect 550)\n", retr);
    assert(retr == 550 && "RETR of a deleted file must return 550");
}

// ---- multi-session, passive/active modes, timeouts ----------------------

// Two logged-in sessions with interleaved transfers: A's STOR stays open
// while B runs a LIST, then A RETRs while B STORs. Each session gets its own
// passive port, so the data connections cannot cross.
void test_two_sessions_interleaved() {
    FtpDial a, b;
    assert(a.login() == 230);
    assert(b.login() == 230);

    const std::string pa = pattern(6000, 1), pb = pattern(5000, 2);

    uint16_t portA = a.pasv(), portB = b.pasv();
    std::fprintf(stderr, "  PASV A=%u B=%u\n", portA, portB);
    assert(portA && portB && portA != portB);

    int da = openData(portA);                 assert(da >= 0);
    a.send("STOR /a.bin");                    assert(a.readCode() == 150);
    sendAll(da, pa.substr(0, 3000));

    int db = openData(portB);                 assert(db >= 0);
    b.send("LIST");                           assert(b.readCode() == 150);
    std::string listing = drainData(db);
    ::close(db);
    assert(b.readCode() == 226);
    std::fprintf(stderr, "  B LIST during A STOR: %zu B\n", listing.size());

    sendAll(da, pa.substr(3000));
    ::close(da);
    assert(a.readCode() == 226);

    portA = a.pasv();                         assert(portA);
    portB = b.pasv();                         assert(portB);
    da = openData(portA);                     assert(da >= 0);
    db = openData(portB);                     assert(db >= 0);
    a.send("RETR /a.bin");                    assert(a.readCode() == 150);
    b.send("STOR /b.bin");                    assert(b.readCode() == 150);
    sendAll(db, pb.substr(0, 2500));
    std::string gotA = drainData(da);
    ::close(da);
    sendAll(db, pb.substr(2500));
    ::close(db);
    assert(a.readCode() == 226);
    assert(b.readCode() == 226);
    assert(gotA == pa && "A's RETR returned wrong bytes");
    assert(get(a, "/b.bin") == pb && "B's upload corrupted");
}

// A third and fourth connection get 421 and are closed; the two sessions
// keep working, and a freed slot is reusable.
void test_over_limit_421() {
    static_assert(FtpServer::MAX_SESSIONS == 2, "test assumes two sessions");
    FtpDial a, b;
    assert(a.login() == 230);
    assert(b.login() == 230);
    {
        FtpDial c, d;
        int cc = c.readCode(), dc = d.readCode();
        std::fprintf(stderr, "  3rd->%d 4th->%d (expect 421)\n", cc, dc);
        assert(cc == 421 && dc == 421);
        assert(c.readCode() == -2 && "over-limit connection must be closed");
    }
    a.send("NOOP");                           assert(a.readCode() == 200);
    b.send("NOOP");                           assert(b.readCode() == 200);
    assert(get(a, "/upload.bin").size() > 0);

    b.send("QUIT");                           assert(b.readCode() == 221);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    FtpDial e;
    assert(e.login() == 230 && "slot must be free after QUIT");
}

// EPSV + NLST, data connection opened after the command
void test_epsv() {
    FtpDial c;
    assert(c.login() == 230);
    uint16_t port = c.epsv();
    std::fprintf(stderr, "  EPSV->%u\n", port);
    assert(port == kPasvPort || port == kPasvPort + 1);
    c.send("NLST /");                         // connect only after the command
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int dfd = openData(port);                 assert(dfd >= 0);
    assert(c.readCode() == 150);
    std::string names = drainData(dfd);
    ::close(dfd);
    assert(c.readCode() == 226);
    assert(names.find("upload.bin\r\n") != std::string::npos);
    c.send("EPSV ALL");                       assert(c.readCode() == 200);
}

// Active mode: PORT and EPRT to a listener on the control peer
void test_active_port() {
    FtpDial c;
    assert(c.login() == 230);
    put(c, "/active.txt", "active mode payload\n");

    for (int eprt = 0; eprt < 2; ++eprt) {
        int l = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        assert(::bind(l, (sockaddr *) &sa, sizeof(sa)) == 0 && ::listen(l, 1) == 0);
        socklen_t sl = sizeof(sa);
        ::getsockname(l, (sockaddr *) &sa, &sl);
        unsigned port = ntohs(sa.sin_port);
        char cmd[64];
        if (eprt) std::snprintf(cmd, sizeof(cmd), "EPRT |1|127.0.0.1|%u|", port);
        else std::snprintf(cmd, sizeof(cmd), "PORT 127,0,0,1,%u,%u", port >> 8, port & 255);
        c.send(cmd);
        int pc = c.readCode();
        std::fprintf(stderr, "  %s ->%d\n", cmd, pc);
        assert(pc == 200);
        c.send("RETR /active.txt");
        int dfd = ::accept(l, nullptr, nullptr);
        ::close(l);
        assert(dfd >= 0);
        assert(c.readCode() == 150);
        std::string got = drainData(dfd);
        ::close(dfd);
        assert(c.readCode() == 226);
        assert(got == "active mode payload\n");
    }
    c.send("EPRT |2|::1|5000|");              assert(c.readCode() == 522);
    c.send("EPRT |1|10.1.2.3|5000|");         assert(c.readCode() == 501);
    c.send("PORT 127,0,0,1,0,21");            assert(c.readCode() == 501); // privileged port
}

// PASV, then never connect: 425 after the data timeout, session survives.
// Same for PORT to a port nobody listens on.
void test_data_timeout() {
    FtpDial c;
    assert(c.login() == 230);
    assert(c.pasv() > 0);
    auto t0 = std::chrono::steady_clock::now();
    c.send("LIST");
    int code = c.readCode(5000);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    std::fprintf(stderr, "  LIST without data conn ->%d after %lld ms (expect 425)\n", code, (long long) ms);
    assert(code == 425);
    c.send("NOOP");                           assert(c.readCode() == 200);
    c.send("RETR /upload.bin");               assert(c.readCode() == 425); // no PASV/PORT pending

    int l = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(l, (sockaddr *) &sa, sizeof(sa));
    socklen_t sl = sizeof(sa);
    ::getsockname(l, (sockaddr *) &sa, &sl);
    unsigned port = ntohs(sa.sin_port);
    ::close(l); // refuse connections on that port
    char cmd[64];
    std::snprintf(cmd, sizeof(cmd), "PORT 127,0,0,1,%u,%u", port >> 8, port & 255);
    c.send(cmd);                              assert(c.readCode() == 200);
    c.send("LIST");                           assert(c.readCode() == 425);
    c.send("NOOP");                           assert(c.readCode() == 200);
}

// An upload that is aborted (ABOR, or the control connection drops) must
// leave the previous file content untouched.
void test_aborted_upload_keeps_original() {
    const std::string orig = "vout_max = 28.4\n";
    {
        FtpDial c;
        assert(c.login() == 230);
        put(c, "/keep.conf", orig);

        uint16_t pport = c.pasv();            assert(pport > 0);
        int dfd = openData(pport);            assert(dfd >= 0);
        c.send("STOR /keep.conf");            assert(c.readCode() == 150);
        sendAll(dfd, "vout_max = 9");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        c.send("\xff\xf4\xff\xf2" "ABOR");
        int a1 = c.readCode(), a2 = c.readCode();
        std::fprintf(stderr, "  ABOR ->%d,%d (expect 426,226)\n", a1, a2);
        assert(a1 == 426 && a2 == 226);
        ::close(dfd);
        assert(get(c, "/keep.conf") == orig && "ABOR must not touch the target");

        pport = c.pasv();                     assert(pport > 0);
        dfd = openData(pport);                assert(dfd >= 0);
        c.send("STOR /keep.conf");            assert(c.readCode() == 150);
        sendAll(dfd, "truncated");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ::close(c.fd); // drop control mid-upload
        c.fd = -1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ::close(dfd);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    FtpDial c;
    assert(c.login() == 230);
    assert(get(c, "/keep.conf") == orig && "dropped session must not touch the target");
    c.send("SIZE /keep.conf");                assert(c.readCode() == 213);
}

// Misc commands: FEAT, SYST, TYPE, MLST, SIZE, MDTM, RNFR/RNTO, RMD, 502, 530
void test_misc_commands() {
    FtpDial c;
    assert(c.readCode() == 220);
    c.send("FEAT");                           assert(c.readCode() == 211);
    c.send("SYST");                           assert(c.readCode() == 215);
    c.send("LIST");                           assert(c.readCode() == 530);
    c.send("CWD /");                          assert(c.readCode() == 530);
    c.send("USER nobody");                    assert(c.readCode() == 331);
    c.send("PASS secret");                    assert(c.readCode() == 530);
    c.send(std::string("USER ") + USER);      assert(c.readCode() == 331);
    c.send(std::string("PASS ") + PASS);      assert(c.readCode() == 230);
    c.send("OPTS UTF8 ON");                   assert(c.readCode() == 200);
    c.send("TYPE I");                         assert(c.readCode() == 200);
    c.send("TYPE X");                         assert(c.readCode() == 504);
    c.send("XYZW");                           assert(c.readCode() == 502);
    c.send("SITE CHMOD 777 x");               assert(c.readCode() == 502);
    c.send(std::string(400, 'N'));            assert(c.readCode() == 500); // over-long line
    c.send("NOOP");                           assert(c.readCode() == 200);
    put(c, "/ren.txt", "12345");
    c.send("SIZE /ren.txt");                  assert(c.readCode() == 213 && c.last == "213 5");
    c.send("MDTM /ren.txt");                  assert(c.readCode() == 213 && c.last.size() == 18);
    c.send("MLST /ren.txt");                  assert(c.readCode() == 250);
    c.send("RNTO /x.txt");                    assert(c.readCode() == 503);
    c.send("RNFR /ren.txt");                  assert(c.readCode() == 350);
    c.send("RNTO /renamed.txt");              assert(c.readCode() == 250);
    c.send("SIZE /ren.txt");                  assert(c.readCode() == 550);
    c.send("MKD /d1");                        assert(c.readCode() == 257);
    c.send("CWD d1");                         assert(c.readCode() == 250);
    c.send("CDUP");                           assert(c.readCode() == 250);
    c.send("RMD /d1");                        assert(c.readCode() == 250);
    c.send("CWD /d1");                        assert(c.readCode() == 550);
    c.send("RETR ../../../etc/passwd");       assert(c.readCode() == 425);
    assert(get(c, "../../../../etc/hosts") == "<550>" && ".. must clamp at the FTP root");
    c.send("QUIT");                           assert(c.readCode() == 221);
}

// Not logging in within the auth timeout, or idling after login, closes the session.
void test_auth_timeout() {
    FtpDial c;
    assert(c.readCode() == 220);
    c.send(std::string("USER ") + USER);      assert(c.readCode() == 331);
    int code = c.readCode(4000);
    std::fprintf(stderr, "  idle before login ->%d (expect 421)\n", code);
    assert(code == 421);
    assert(c.readCode() == -2);
}

void test_idle_timeout() {
    FtpDial c;
    assert(c.login() == 230);
    c.send("NOOP");                           assert(c.readCode() == 200);
    int code = c.readCode(4000);
    std::fprintf(stderr, "  idle after login ->%d (expect 421)\n", code);
    assert(code == 421);
    assert(c.readCode() == -2);
}

// ---- subprocess harness ---------------------------------------------------

struct Test {
    const char *name;
    void (*fn)();
    FtpServer::Timeouts timeouts;
};

// Fork-per-test so an assertion in one test is reported as a single failure.
// The server runs in the parent.
bool runForked(const Test &t) {
    pid_t pid = ::fork();
    if (pid < 0) { std::perror("fork"); return false; }
    if (pid == 0) {
        t.fn();
        std::_Exit(0);
    }
    int status = 0;
    ::waitpid(pid, &status, 0);
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        std::fprintf(stderr, "  PASS %s\n\n", t.name);
        return true;
    }
    if (WIFSIGNALED(status)) {
        std::fprintf(stderr, "  FAIL %s — killed by signal %d\n\n", t.name, WTERMSIG(status));
    } else {
        std::fprintf(stderr, "  FAIL %s — exit %d\n\n", t.name, WEXITSTATUS(status));
    }
    return false;
}

}  // namespace

int main() {
    // Writing to a half-closed peer would otherwise SIGPIPE the whole binary.
    std::signal(SIGPIPE, SIG_IGN);

    char root[] = "/tmp/ftp-e2e-XXXXXX";
    if (!::mkdtemp(root)) { std::perror("mkdtemp"); return 1; }
    if (!server.begin(USER, PASS, root, kCmdPort, kPasvPort)) {
        std::fprintf(stderr, "server.begin failed\n");
        return 1;
    }
    std::fprintf(stderr, "root %s\n", root);

    std::thread t(ftpLoop);

    const FtpServer::Timeouts def{};
    const Test tests[] = {
        {"pass_without_user",       test_pass_without_user, def},
        {"malformed_port",          test_malformed_port, def},
        {"port_bounce",             test_port_bounce, def},
        {"cwd_dotdot_clamps",       test_cwd_dotdot_clamps_at_root, def},
        {"upload_download",         test_upload_download, def},
        {"large_file_round_trip",   test_large_file_round_trip, def},
        {"dele_then_retr_fails",    test_dele_then_retr_fails, def},
        {"mlsd_subdir",             test_mlsd_subdir, def},
        {"two_sessions_interleaved", test_two_sessions_interleaved, def},
        {"over_limit_421",          test_over_limit_421, def},
        {"epsv",                    test_epsv, def},
        {"active_port",             test_active_port, def},
        {"data_timeout",            test_data_timeout, {10000, 300000, 500}},
        {"aborted_upload_keeps_original", test_aborted_upload_keeps_original, def},
        {"misc_commands",           test_misc_commands, def},
        {"auth_timeout",            test_auth_timeout, {1000, 300000, 10000}},
        {"idle_timeout",            test_idle_timeout, {10000, 1000, 10000}},
        {"long_cwd_path",           test_long_cwd_path, def},
    };

    int failures = 0, n = 0;
    for (const auto &t : tests) {
        {
            std::lock_guard<std::mutex> lk(serverMu);
            server.timeouts = t.timeouts;
        }
        std::fprintf(stderr, "--- %s ---\n", t.name);
        if (!runForked(t)) ++failures;
        ++n;
        // let the server notice the closed sessions before the next test connects
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running = false;
    t.join();
    server.end();
    std::string rm = std::string("rm -rf ") + root;
    if (std::system(rm.c_str()) != 0) std::fprintf(stderr, "cleanup of %s failed\n", root);

    if (failures) {
        std::fprintf(stderr, "%d of %d test(s) FAILED\n", failures, n);
        return 1;
    }
    std::fprintf(stderr, "ALL %d FTP TESTS PASSED\n", n);
    return 0;
}
