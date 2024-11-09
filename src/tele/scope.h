#pragma once

#include <sstream>
#include <cassert>


#ifdef ARDUINO

#include <WiFi.h>

#else

#define ESP_LOGI(tag, format, ...) printf("[%s] " format "\n", tag, ##__VA_ARGS__)

class TCPClientStub {
public:
    bool _conn = false;

    bool connected() { return _conn; }

    int write(const uint8_t *buf, size_t len) {
        ::write(fileno(stdout), "\nbin data[]|", 13);
        auto l = ::write(fileno(stdout), buf, len);
        ::write(fileno(stdout), "|bin data end\n", 14);
        return l;
    }

    int write(const char *str) {
        printf("> %s\n", str);
        return strlen(str);
    }

    struct String {
        std::string toString() {
            return "stub";
        }
    };

    String remoteIP() { return String(); }
};

#endif

class TCPServerStub {
public:
    TCPServerStub(int port = 22, int maxClients = 1) {}

    void begin() {}

    void setNoDelay(bool noDelay) {}

    TCPClientStub accept() {
        auto c = TCPClientStub();
        c._conn = true;
        return c;
    }

    bool hasClient() { return true; }
};


class Scope {
    typedef TCPClientStub TCPClient;
    typedef TCPServerStub TCPServer;


    struct {
        uint16_t _zero: 1;
        uint16_t channel: 3;
        uint16_t data: 12;
    } Data12Ch4;

    struct {
        uint8_t _one: 1; // always 1
        uint8_t channel: 3; // ch num 0-8
        uint8_t _zero: 1;
        uint8_t fmt: 2; // 0b00 = u16, 0b01 = i16, 0b10= ?, 0b11 = f16
        uint8_t data1;
        uint8_t data2;
    } Data16Ch4;

    TCPServer srv;
    TCPClient client;


public:
    bool begin(uint16_t port = 24) {

#if ARDUINO
        if (WiFi.status() != WL_CONNECTED)
            return false;
#endif

        srv = TCPServerStub(port, 1);
        srv.begin();
        srv.setNoDelay(true);
        return true;
    }

    static constexpr uint8_t TypInt = 'i';
    static constexpr uint8_t TypUInt = 'u';
    static constexpr uint8_t TypF = 'f';

    struct h {
        uint8_t ch;
        uint8_t typ;
        uint8_t len;
        const char *name;
        //const char *fmt; // i8, i12, i16, u8, u12, u16, u32, f32
        void *ptr;
    };

    std::vector<h> channels;
    std::array<uint8_t, 8> chOffset{};

    uint8_t wroteCh = 0;
    uint8_t *buf = nullptr;
    uint16_t frameIdx = 0;
    uint16_t bufFrames = 256;
    uint8_t frameSize = 0;

    void addChannel(uint8_t ch, const char *name, float *ptr) {
        assert(sizeof(*ptr) == 4);
        channels.emplace_back(h{ch, TypF, 32, name, (void *) ptr});
        chOffset[ch] = *std::max_element(chOffset.begin(), chOffset.end()) + 1;
    }

    void addChannel(uint8_t ch, uint8_t typ, uint8_t bitLen, const char *name) {
        assert(buf == nullptr);
        assert(bitLen % 8 == 0);
        uint8_t len = bitLen / 8;

        auto lastCh = std::max_element(chOffset.begin(), chOffset.end()) - chOffset.begin();
        uint8_t offset = 0;
        if (!channels.empty()) {
            if (channels.size() >= 2)
                assert(channels.back().ch == lastCh);
            offset = chOffset[lastCh] + channels.back().len;
        }

        channels.emplace_back(h{ch, typ, len, name, nullptr});

        chOffset[ch] = offset;
        frameSize += len;
    }

    void startStream() {
        assert(buf == nullptr);
        buf = (uint8_t *) malloc(frameSize * bufFrames);
    }

    void addSample(uint8_t ch, const uint16_t &sample) {
        //assert() ch type
        if (wroteCh << ch & 1) {
            // if we already wrote this channel, move to next frame
            ++frameIdx;

            // TODO what about those channels that have not set?
            // set to zero, nan flag, repeat?

            if (frameIdx == bufFrames) {
                // flush
                _flushBuf();
                frameIdx = 0;
            }
        }
        memcpy(&buf[frameIdx * frameSize], &sample, sizeof(sample));
        wroteCh |= 1 << ch;
    }

    /*void addSample(uint8_t ch, const uint16_t &sample) {
        //assert() ch type
        if (wroteCh << ch & 1) {
            // if we already wrote this channel, move to next frame
            ++frameIdx;

            // TODO what about those channels that have not set?
            // set to zero, nan flag, repeat?

            if (frameIdx == bufFrames) {
                // flush
                _flushBuf();
                frameIdx = 0;
            }
        }
        memcpy(&buf[chOffset[ch] + frameIdx * frameSize], &sample, sizeof(sample));
        wroteCh |= 1 << ch;
    }*/

    void _flushBuf() {
        //client.write((uint8_t *)"flushBuf:", 9);
        client.write((uint8_t *) buf, (size_t) (bufFrames * frameSize));
        //client.write("\n");
    }

    void sendHeader(TCPClient &cl) {
        std::stringstream ss;
        ss << "###HEAD:";
        for (auto &ch: channels) ss << ch.name << '=' << ch.typ << (int) ch.len << ',';
        ss << "###ENDHEAD\n";
        cl.write(ss.str().c_str());
    }

    void _updateClient(TCPClient &cl) {
        int any = 0;
        auto us = &any; // &loopWallClockUs();
        //cl.write((uint8_t *) us, sizeof(*us));
    }

    void update() {
        if (!srv.hasClient()) {
            return;
        }

        if (!client.connected()) {
            auto newClient = srv.accept();
            if (newClient.connected()) {
                ESP_LOGI("scope", "New client %s connected", newClient.remoteIP().toString().c_str());
                client = newClient;
                sendHeader(client);
            }
        } else {
            _updateClient(client);
        }
    }
};


static Scope *scope = nullptr;