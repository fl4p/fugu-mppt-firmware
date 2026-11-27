#include "tele/tele_packet.h"
#include <cstring>
#include <memory>
static std::unique_ptr<TelePacketEncoder> encoder;
extern "C" void reset_packets() { encoder.reset(new TelePacketEncoder()); }
extern "C" size_t encode_packets(const char *raw,size_t n,char *out,size_t cap) {
    if (!encoder) reset_packets();
    std::vector<std::string> packets;
    if(!encoder->pack(std::string(raw,n),1440,packets))return 0;
    size_t pos=0;
    for(auto&p:packets){
        if(p.size()>65535 || pos+2+p.size()>cap)return 0;
        out[pos++]=char(p.size());out[pos++]=char(p.size()>>8);
        memcpy(out+pos,p.data(),p.size());pos+=p.size();
    }
    return pos;
}
