#include <sys/time.h>
#include <memory>
static int64_t now_ms;
static int bench_time(timeval *tv, void *) { tv->tv_sec=now_ms/1000; tv->tv_usec=(now_ms%1000)*1000; return 0; }
#define gettimeofday bench_time
#include "src/tele/sym_line_protocol.h"
#undef gettimeofday
#include "src/tele/compress.h"
static SymbolTable table;
extern "C" {
void reset_table() { table=SymbolTable{}; }
size_t point(int64_t ts, const char *dev, int n, const char **names, const char *types, const double *values, char *out) {
    now_ms=ts;
    BinaryLineProtocol p(table,"mppt"); p.addTag("device",dev);
    for(int i=0;i<n;++i) {
        if(types[i]=='i') p.addField(names[i],int(values[i]));
        else if(types[i]=='B') p.addField(names[i],bool(values[i]));
        else p.addField(names[i],float(values[i]));
    }
    p.setTimeMs(); auto s=p.take(); memcpy(out,s.data(),s.size()); return s.size();
}
size_t compress_packet(const unsigned char *in,size_t n,char *out) {
    TampCompress c; std::string s;
    if(!c.packet(in,n,s)) return 0;
    memcpy(out,s.data(),s.size()); return s.size();
}
}
