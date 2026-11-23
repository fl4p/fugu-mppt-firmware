#include "tls_console.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>
#include <mbedtls/ssl.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

#include <algorithm>
#include <new>

#include "../console.h"
#include "../logging.h"
#include "../etc/version.h"
#include "telemetry.h" // getHostname

#define TAG "tlscon"

namespace {
constexpr uint16_t kOutCap = 1024;
constexpr uint8_t kLineCap = 128;
constexpr uint32_t kMinHeap = 24000;
constexpr time_ms kHandshakeMs = 10000;

struct Session {
    mbedtls_ssl_context ssl;
    int fd;
    time_ms t0;
    bool up;
    uint16_t pend; // length of a write that returned WANT_WRITE; must be retried unchanged
    uint16_t outLen;
    uint8_t lineLen;
    char line[kLineCap];
    char out[kOutCap];
};

mbedtls_ssl_config *s_conf;
Session *s_sess;
int s_listen = -1;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

const int kSuites[] = {MBEDTLS_TLS_PSK_WITH_AES_128_GCM_SHA256, 0};
}

static void sink(const char *s, uint16_t n) {
    portENTER_CRITICAL(&s_mux);
    if (Session *x = s_sess) {
        uint16_t k = std::min<uint16_t>(n, kOutCap - x->outLen);
        memcpy(x->out + x->outLen, s, k);
        x->outLen += k;
    }
    portEXIT_CRITICAL(&s_mux);
}

static int bioSend(void *ctx, const unsigned char *b, size_t n) {
    int r = send((int) (intptr_t) ctx, b, n, MSG_DONTWAIT);
    if (r >= 0) return r;
    return errno == EAGAIN || errno == EWOULDBLOCK ? MBEDTLS_ERR_SSL_WANT_WRITE : MBEDTLS_ERR_NET_SEND_FAILED;
}

static int bioRecv(void *ctx, unsigned char *b, size_t n) {
    int r = recv((int) (intptr_t) ctx, b, n, MSG_DONTWAIT);
    if (r > 0) return r;
    if (r == 0) return MBEDTLS_ERR_SSL_CONN_EOF;
    return errno == EAGAIN || errno == EWOULDBLOCK ? MBEDTLS_ERR_SSL_WANT_READ : MBEDTLS_ERR_NET_RECV_FAILED;
}

static void closeSession() {
    Session *x = s_sess;
    if (!x) return;
    removeLogCallback(sink);
    portENTER_CRITICAL(&s_mux);
    s_sess = nullptr;
    portEXIT_CRITICAL(&s_mux);
    if (x->up) mbedtls_ssl_close_notify(&x->ssl);
    mbedtls_ssl_free(&x->ssl);
    close(x->fd);
    delete x;
    ESP_LOGI(TAG, "client closed");
}

static void acceptClient() {
    sockaddr_in a{};
    socklen_t al = sizeof(a);
    int fd = accept(s_listen, (sockaddr *) &a, &al);
    if (fd < 0) return;
    const char *why = s_sess ? "busy" : heap_caps_get_free_size(MALLOC_CAP_8BIT) < kMinHeap ? "low heap" : nullptr;
    Session *x = why ? nullptr : new(std::nothrow) Session{};
    if (!x) {
        ESP_LOGW(TAG, "refused %s: %s", inet_ntoa(a.sin_addr), why ? why : "no memory");
        close(fd);
        return;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    mbedtls_ssl_init(&x->ssl);
    x->fd = fd;
    x->t0 = millis();
    if (int r = mbedtls_ssl_setup(&x->ssl, s_conf)) {
        ESP_LOGE(TAG, "ssl setup -0x%04x", -r);
        mbedtls_ssl_free(&x->ssl);
        close(fd);
        delete x;
        return;
    }
    mbedtls_ssl_set_bio(&x->ssl, (void *) (intptr_t) fd, bioSend, bioRecv, nullptr);
    portENTER_CRITICAL(&s_mux);
    s_sess = x;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "client %s", inet_ntoa(a.sin_addr));
}

static bool feed(Session *x, const char *b, int n) {
    for (int i = 0; i < n; ++i) {
        char c = b[i];
        if (c != '\r' && c != '\n') {
            if (x->lineLen < kLineCap) x->line[x->lineLen++] = c;
            continue;
        }
        if (!x->lineLen) continue;
        if (x->lineLen == 4 && !memcmp(x->line, "exit", 4)) return false;
        consoleDispatchAsync(x->line, x->lineLen, nullptr);
        x->lineLen = 0;
    }
    return true;
}

static bool flush(Session *x) {
    uint16_t n = x->pend;
    if (!n) {
        portENTER_CRITICAL(&s_mux);
        n = x->outLen;
        portEXIT_CRITICAL(&s_mux);
        if (!n) return true;
    }
    int r = mbedtls_ssl_write(&x->ssl, (const unsigned char *) x->out, n);
    if (r == MBEDTLS_ERR_SSL_WANT_WRITE || r == MBEDTLS_ERR_SSL_WANT_READ) {
        x->pend = n;
        return true;
    }
    if (r < 0) return false;
    x->pend = 0;
    portENTER_CRITICAL(&s_mux);
    memmove(x->out, x->out + r, x->outLen - r);
    x->outLen -= r;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

std::string TlsConsoleService::statusDetail() const {
    return s_sess ? (s_sess->up ? "connected" : "handshake") : "";
}

bool TlsConsoleService::onStart() {
    if (!WiFi.isConnected()) return false;
    ConfFile c{_confPath, true};
    std::string id = c.getString("psk_id", ""), hex = c.getString("psk", "");
    const auto port = (uint16_t) c.getLong("port", 992);
    uint8_t key[32];
    size_t kl = hex.size() / 2;
    bool ok = !id.empty() && hex.size() % 2 == 0 && kl >= 16 && kl <= sizeof(key);
    for (size_t i = 0; ok && i < kl; ++i) {
        char *e;
        char h[3] = {hex[2 * i], hex[2 * i + 1], 0};
        key[i] = (uint8_t) strtoul(h, &e, 16);
        ok = *e == 0;
    }
    mbedtls_platform_zeroize(hex.data(), hex.size());
    if (!ok) {
        ESP_LOGE(TAG, "tlscon.conf needs psk_id and psk (32-64 hex digits)");
        return false;
    }
    psa_crypto_init();
    s_conf = new(std::nothrow) mbedtls_ssl_config;
    if (!s_conf) return false;
    mbedtls_ssl_config_init(s_conf);
    int r = mbedtls_ssl_config_defaults(s_conf, MBEDTLS_SSL_IS_SERVER, MBEDTLS_SSL_TRANSPORT_STREAM,
                                        MBEDTLS_SSL_PRESET_DEFAULT);
    mbedtls_ssl_conf_min_tls_version(s_conf, MBEDTLS_SSL_VERSION_TLS1_2);
    mbedtls_ssl_conf_max_tls_version(s_conf, MBEDTLS_SSL_VERSION_TLS1_2);
    mbedtls_ssl_conf_ciphersuites(s_conf, kSuites);
    if (!r) r = mbedtls_ssl_conf_psk(s_conf, key, kl, (const unsigned char *) id.data(), id.size());
    mbedtls_platform_zeroize(key, sizeof(key));

    if (!r) {
        s_listen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        int one = 1;
        setsockopt(s_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port);
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        if (s_listen < 0 || bind(s_listen, (sockaddr *) &a, sizeof(a)) || listen(s_listen, 1)) r = -1;
        else fcntl(s_listen, F_SETFL, O_NONBLOCK);
    }
    if (r) {
        ESP_LOGE(TAG, "start failed (%d)", r);
        onStop();
        return false;
    }
    ESP_LOGI(TAG, "listening on port %u, identity '%s'", port, id.c_str());
    return true;
}

void TlsConsoleService::onStop() {
    closeSession();
    if (s_listen >= 0) close(s_listen);
    s_listen = -1;
    if (s_conf) mbedtls_ssl_config_free(s_conf);
    delete s_conf;
    s_conf = nullptr;
}

void TlsConsoleService::onTick() {
    acceptClient();
    Session *x = s_sess;
    if (!x) return;
    if (!x->up) {
        int r = mbedtls_ssl_handshake(&x->ssl);
        if (!r) {
            x->up = true;
            char b[96];
            int n = snprintf(b, sizeof(b), "\nWelcome to %s (%s)\n", getHostname().c_str(),
                             WiFi.localIP().toString().c_str());
            sink(b, std::min<int>(n, sizeof(b) - 1));
            const char *v = format_version();
            sink(v, strlen(v));
            sink("\n(exit to disconnect.)\n", 23);
            addLogCallback(sink, false);
            lastTimeOutUs = 0;
        } else if ((r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE) ||
                   millis() - x->t0 > kHandshakeMs) {
            ESP_LOGW(TAG, "handshake failed -0x%04x", -r);
            closeSession();
        }
        return;
    }
    char b[64];
    int r = 0;
    bool keep = true;
    while (keep && (r = mbedtls_ssl_read(&x->ssl, (unsigned char *) b, sizeof(b))) > 0) keep = feed(x, b, r);
    if (keep && r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE) keep = false;
    if (!keep || !flush(x)) closeSession();
}
