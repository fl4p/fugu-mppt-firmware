#pragma once

#include <sstream>
#include <cassert>

static uint64_t bytesSent = 0;

#ifdef ARDUINO

#include <WiFi.h>
#include <lwip/sockets.h>

typedef WiFiServer TCPServer;
typedef WiFiClient TCPClient;

#else

#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define ESP_LOGI(tag, format, ...) printf("[%s] " format "\n", tag, ##__VA_ARGS__)

static std::function<void(const uint8_t *buf, size_t len)> stub_sink = nullptr;

class TCPClientStub {
public:
    bool _conn = false;

    bool connected() { return _conn; }


    int write(const uint8_t *buf, size_t len) {

        if (stub_sink) stub_sink(buf, len);

        //::write(fileno(stdout), "\nbin data[]|", 13);
        //auto l = ::write(fileno(stdout), buf, len);
        //::write(fileno(stdout), "|bin data end\n", 14);
        return len;
    }

    int write(const char *str) {
        if (stub_sink) stub_sink((uint8_t *) str, strlen(str));
        //printf("> %s\n", str);
        return strlen(str);
    }

    struct String {
        std::string toString() {
            return "stub";
        }
    };

    String remoteIP() { return String(); }
};


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

typedef TCPClientStub TCPClient;
typedef TCPServerStub TCPServer;
#endif

/**
 *

  auto sc = Scope();

void controlLoop() {
  while(true) {
    uint16_t x0 = adcReadCh(0);
    uint16_t x1 = adcReadCh(1);

    sc.addSample12(0, x0);
    sc.addSample12(1, x1);
  }
}


void networkLoop() {
    sc.update();
}

# Encoding

The data is serialiazed into binary stream.
 * no headers
 * 12bit sample [u12] => 1 length bit (`0`) + 3 chID bits + 12bit payload (2byte)
   * if the length bit is 1, jump to 16bit sample
 * 16bit sample [u16,i16,f16]=> 1 lenght bit (`1`) + 3 chID bits + 1 lenght bit ( `0`) + 2 fmt bits + 16 payload bits (3byte)
    * if 2nd length bit is `1`, jump to 32bit
    * fmt bits sets the datatype 00 => u16, 01 -> i16, 11 -> f16
 * 32bit  u32,i32,f32 => 1 lenght bit (`1`) + 3 chID bits + 1 lenght bit ( `1`) + 2 fmt bits + 32 payload bits (5byte)

 */

class Scope {


public:
    struct Data12Ch4 {
        uint16_t _zero: 1;
        uint16_t channel: 3;
        uint16_t data: 12;
    };

    struct Data16Ch4 { // 3bytes
        uint8_t _one: 1; // always 1
        uint8_t channel: 3; // ch num 0-8
        uint8_t _zero: 1; // 1 => 32bit
        uint8_t fmt: 2; // 0b00 = u16, 0b01 = i16, 0b10= N/A, 0b11 = f16
        uint8_t data1;
        uint8_t data2;
        uint8_t reserved: 1;

        uint16_t u16() {
            return *(uint16_t *) &data1;
        }

        uint16_t i16() {
            return *(int16_t *) &data1;
        }
    };

    struct Data32Ch4 { // 5bytes = 4 payload + 1
        uint8_t _one: 1; // always 1
        uint8_t channel: 3; // ch num 0-8
        uint8_t _one2: 1;
        uint8_t fmt: 2; // 0b00 = u32, 0b01 = i32, 0b10= f32, 0b11 = 64b // todo
        uint8_t data1;
        uint8_t data2;
        uint8_t data3;
        uint8_t data4;
    };

private:

    TCPServer srv;
    TCPClient client;

    std::unordered_map<const void *, uint8_t> ptr2Cid;

public:
    //uint32_t numBytesSent = 0;

    bool begin(uint16_t port = 24) {

#if ARDUINO
        if (WiFi.status() != WL_CONNECTED)
            return false;
#endif

        srv = TCPServer(port, 1);
        srv.begin();
        srv.setNoDelay(true);
        return true;
    }

    void end() {
        srv.end();
    }

    static constexpr uint8_t TypInt = 'i';
    static constexpr uint8_t TypUInt = 'u';
    static constexpr uint8_t TypF = 'f';

    struct h {
        uint8_t cid; // global ch id
        uint8_t ch; // adc local channel
        uint8_t typ;
        uint8_t bitLen;
        const char *name;
        //const char *fmt; // i8, i12, i16, u8, u12, u16, u32, f32
        //void *ptr;
    };

    std::vector<h> channels;

    static constexpr size_t BufSize = 1024 * 2;
    static constexpr size_t BufNum = 2;

    uint8_t buf[BufNum][BufSize];
    volatile uint8_t bufSel = 0;
    volatile uint16_t bufPos[BufNum] = {0};

    bool connected = false;

    void addChannel(const void *adc, uint8_t chNum, uint8_t typ, uint8_t bitLen, const char *name) {
        // assert(buf == nullptr);
        // assert(bitLen % 8 == 0);
        // uint8_t len = bitLen / 8;
        uint8_t cid = channels.empty() ? 0 : (channels.back().cid + 1);
        channels.emplace_back(h{cid, chNum, typ, bitLen, name});
        assert(ptr2Cid.find((char *) adc + chNum) == ptr2Cid.end()); // check for cid collision
        ptr2Cid[((char *) adc) + chNum] = cid; // todo custom mapping
    }

    void startStream() {
        //uint8_t offset = 0;
    }

    TaskNotification notification{};

    bool netLoop() {
        notification.subscribe();
        if (notification.wait(1)) {
            auto bufSelSend = (bufSel - 1) % BufNum;
            auto sent = client.write((uint8_t *) buf[bufSelSend], (size_t) (bufPos[bufSelSend]));
            bufPos[bufSelSend] = 0;
            bytesSent += sent;
            return true;
        }
        return false;
    }

    uint8_t cid(const void *adc, uint8_t chNum) const {
        return ptr2Cid.find((char *) adc + chNum)->second;
    }

    void addSample12(const void *adc, uint8_t ch, const uint16_t &sample) {

        if (!connected)
            return;

        if (bufPos[bufSel] + sizeof(Data12Ch4) > BufSize) {
            static uint8_t dropping = false;
            auto nextBuf = (bufSel + 1) % BufNum;
            if (bufPos[nextBuf]) {
                if (!dropping) {
                    dropping = true;
                    ESP_LOGW("scope", "buffer over-flow, dropping sample");
                }
                return;
            } else if (dropping) dropping = false;
            bufSel = nextBuf;
            notification.notify();
        }

//ESP_LOGI("scope", "new sample %u", sample);

        auto p = (Data12Ch4 *) &(buf[bufSel][bufPos[bufSel]]);
        p->_zero = 0;
        p->channel = cid(adc, ch);
        p->data = sample;

        bufPos[bufSel] = bufPos[bufSel] + sizeof(Data12Ch4);
    }

    /*
    void addSample16(uint8_t ch, const uint16_t &sample) {
        if (bufPos + sizeof(Data16Ch4) > BufSize) {

            bufPos[bufSel] = 0;
        }

        auto p = (Data16Ch4 *) &buf[bufSel][bufPos[bufPos]];
        p->_one = 1;
        p->channel = ch;
        p->_zero = 0;
        p->fmt = 0;
        *((uint16_t *) &p->data1) = sample;
        p->reserved = 0;

        bufPos[bufSel] += sizeof(Data16Ch4);
    }
     */

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


    void sendHeader(TCPClient &cl) {
        std::stringstream ss;
        ss << "###ScopeHead:";
        for (auto &ch: channels)
            ss << int(ch.cid) << '$' << ch.name << "=" << ch.typ << (int) ch.bitLen << ',';
        ss << "###ENDHEAD\n";
        cl.write(ss.str().c_str());
    }

    void _updateClient(TCPClient &cl) {
        //int any = 0;
        //auto us = &any; // &loopWallClockUs();
        //cl.write((uint8_t *) us, sizeof(*us));
    }

    void update() {
        /*if (!srv.hasClient()) {
            if (connected) {
                ESP_LOGI("scope", "Disconnected (!hasClient())");
                connected = false;
            }
            return;
        }*/

        if (!client.connected()) {
            if (connected) ESP_LOGI("scope", "Disconnected (!connected())");
            connected = false;
            if (srv.hasClient()) {
                auto newClient = srv.accept();
                if (newClient.connected()) {
                    connected = true;
                    ESP_LOGI("scope", "New client %s connected", newClient.remoteIP().toString().c_str());
                    client = newClient;
                    sendHeader(client);
                    WiFi.setTxPower(WIFI_POWER_21dBm);
                }
            }
        } else {
            connected = true;
            _updateClient(client);
        }
    }
};

class ScopeDecoder {

    uint8_t buf[128];
    uint16_t bufPos = 0;


    size_t readBuffer(const uint8_t *start, const uint8_t *end) {
        auto s0 = start;
        while (end - start >= sizeof(Scope::Data12Ch4)) {
            const auto &d12 = (Scope::Data12Ch4 *) start;
            if (d12->_zero == 0) {
                if (onInt)
                    onInt(d12->channel, d12->data);
                start += sizeof(Scope::Data12Ch4);
            } else {
                if (end - start < sizeof(Scope::Data16Ch4)) break;
                const auto &d16 = (Scope::Data16Ch4 *) buf;
                if (d16->_zero == 0) {
                    if (d16->fmt == 0b00) onInt(d12->channel, d16->u16());
                    else if (d16->fmt == 0b01) onInt(d12->channel, d16->i16());
                    else {
                        throw std::range_error("not impl");
                    }
                    start += sizeof(Scope::Data16Ch4);
                } else {
                    if (bufPos < sizeof(Scope::Data32Ch4)) break;
                    throw std::range_error("not impl");
                    //const auto &d32 = (Scope::Data32Ch4 *) buf;
                    //start += sizeof(Scope::Data32Ch4);
                }
            }
        }
        return start - s0;
    }

public:
    std::function<void(uint8_t ch, int)> onInt;

    void write(const uint8_t *bytes, size_t len) {
        if (len == 0) return;
        if (bufPos or len < 2) {
            memcpy(&buf[bufPos], bytes, len);
            bufPos += len;

            auto r = readBuffer(&buf[0], &buf[bufPos]);
            if (r < bufPos) {
                // copy any residual data to the beginning
                //  a blocked circular buffer might be more efficient here, but we expect that this does not happen often
                memcpy(&buf[0], &buf[r], bufPos - r);
                bufPos = bufPos - r;
            }
        } else {
            assert(bufPos == 0);
            auto r = readBuffer(bytes, bytes + len);
            if (r != len) {
                memcpy(&buf[bufPos], bytes + r, len - r);
                bufPos += len - r;
            }
        }
    }

    int client_fd = -1;

    bool connect() {
        client_fd = socket(AF_INET, SOCK_STREAM, 0);

        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(24);

        // Convert IPv4 and IPv6 addresses from text to binary
        // form
        if (inet_pton(AF_INET, "192.168.1.147", &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return false;
        }

        int status;

        if ((status = ::connect(client_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))) < 0) {
            printf("\nConnection Failed \n");
            return false;
        }

        printf("Connected!\n");


        return true;
    }

    void update() {
        static uint8_t rbuf[512];
        if (client_fd != -1) {
            auto l = recv(client_fd, rbuf, sizeof(rbuf), 0);
            if (l == 9 and strstr((char *) rbuf, "ScopeHead")) {
                printf("received header");
            } else if (l > 0)
                write(rbuf, l);
        }
    }
};


extern Scope *scope;