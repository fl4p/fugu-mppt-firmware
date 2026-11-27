#include "tele/tele_packet.h"
#include "tele/sym_line_protocol.h"
#include <cassert>
#include <chrono>
#include <cstdio>

int main() {
    TelePacketBatch b;
    assert(!b.due(10000));
    assert(b.append("x", 0));
    assert(!b.due(1799)); assert(b.due(1800)); assert(b.due(999999));
    assert(b.append("y", 1700)); assert(b.due(1800)); // adding a sample never postpones first deadline
    b.clear(); assert(!b.due(999999));
    assert(!b.append(std::string(TelePacketBatch::CAP + 1, 'x'), 0)); assert(b.raw.empty());
    assert(b.append(std::string(TelePacketBatch::CAP, 'x'), 100)); assert(b.due(100)); assert(!b.fits(1));
    b.clear(); assert(b.append("x", 999999)); assert(!b.due(1000000)); assert(!b.due(999998));
    TelePacketEncoder encoder;
    std::vector<std::string> packets;
    assert(!encoder.pack("", 1440, packets));
    assert(!encoder.pack(std::string("\x80", 1), 1440, packets));
    assert(!encoder.pack(std::string("\xff\xff\xff\xff\xff\xff\xff\xff\xff\x7f",10),1440,packets));
    assert(packets.empty());
    SymbolTable table;
    std::string raw;
    for (int i=0;i<100;i++) {
        BinaryLineProtocol p(table,"mppt");p.addTag("device","test");
        p.addField("I",float(i)/7);p.addField("P",float(i)*2);p.setTimeMs();raw+=p.take();
    }
    auto start=std::chrono::steady_clock::now();
    assert(encoder.pack(raw, 1440, packets));
    for(auto &p:packets) assert(!p.empty() && p.size()<=1440 && uint8_t(p[0])<=2);
    printf("native packet encode %lld us, %zu packets\n",(long long)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count(),packets.size());
    packets.clear();assert(encoder.pack(raw,128,packets));
    assert(packets.size()>1);for(auto&p:packets)assert(p.size()<=128);
    packets={"existing"};assert(!encoder.pack(raw,2,packets));assert(packets.size()==1&&packets[0]=="existing");
    // A rejected complete pack must not teach the private dtype cache.
    TelePacketEncoder transaction;
    packets.clear(); assert(!transaction.pack(raw,2,packets));
    std::string dataOnly;
    for(size_t at=0;at<raw.size();) {
        const size_t begin=at;size_t n=0;unsigned shift=0;uint8_t byte;
        do {byte=uint8_t(raw[at++]);n|=size_t(byte&127)<<shift;shift+=7;} while(byte&128);
        if(uint8_t(raw[at])==1)dataOnly.append(raw,begin,at-begin+n);
        at+=n;
    }
    assert(transaction.pack(dataOnly,1440,packets));
    for(const auto &p:packets)assert(uint8_t(p[0])!=2);
    puts("tele-packet tests passed");
}
