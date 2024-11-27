#include <iostream>
#include <unistd.h>

#include "../../src/tele/scope.h"
#include "../../managed_components/brianpugh__tamp/tamp/compressor.h"

int main() {

    auto dec = ScopeDecoder();
    dec.connect();

    std::array<int, 2<<3> chVal{0};

    dec.onInt = [&](uint8_t ch, int v) {
        if(chVal[ch] == v) return;
        chVal[ch] = v;
        if(ch == 2)
            printf("decoded ch%i v=%d\n", ch, v);
    };

    while(true) {
        dec.update();
        usleep(1);
    }

    /*
    static unsigned char window_buffer[1024];

    TampConf conf = {
            .window = 10, // 10 = 1024byte (8min=256, 15max = 32678)
            .literal = 8, // general purpose=8bit, ascii text: 7bit
            / * To improve compression ratios for very short messages, a custom
            buffer initialization could be used.
            For most use-cases, set this to false.* /
            .use_custom_dictionary = false
    };
    TampCompressor compressor;
    tamp_compressor_init(&compressor, &conf, window_buffer);

*/

    auto sc = Scope();
    //tamp_compressor_sink()
    sc.addChannel(1, 'u', 16, "vin");
    sc.addChannel(0, 'u', 16, "vout");
    sc.addChannel(2, 'i', 16, "iin");

    //auto dec = ScopeDecoder();
    auto decLastCh = -1;
    auto decLastVal = -1;
    auto decI = 0;
    dec.onInt = [&](uint8_t ch, int v) {
        decLastCh = ch;
        decLastVal = v;

        if (ch == 0)
            assert((42 + decI) % (2 << (12 - 1)) == v);
        else if (ch == 1)
            assert((420 + decI) % (2 << (12 - 1)) == v);
        else
            assert(false);

        printf("decoded ch%i v=%d\n", ch, v);

        decI++;
    };
    stub_sink = [&](const uint8_t *buf, size_t len) {
        dec.write(buf, len);
    };

    sc.startStream();

    int i = 0;
    while (true) {
        sc.update();

        sc.addSample12(0, 42 + i++);
        sc.addSample12(1, 420 + i++);

        usleep(2000);
    }
}