// Host test for src/tele/vedirect_text.h: block layout, checksum, buffer overflow.
//
// Build & run:
//   clang++ -std=gnu++20 -o /tmp/vedirect-test test/host-stub/vedirect-test.cpp && /tmp/vedirect-test
// `/tmp/vedirect-test dump` writes three blocks to stdout instead (feed to a VE.Direct reader).

#include <cstdio>
#include <cstring>
#include <string>

#include "../../src/tele/vedirect_text.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static VeText sample(uint32_t h20) {
    return {.pid = 0xA056, .fw = 210, .ser = "FG12AB34CD", .v_mV = 26840, .i_mA = -120, .vpv_mV = 61230,
            .ppv_W = 0, .cs = VE_CS_OFF, .mppt = VE_MPPT_OFF, .err = 0, .off = VE_OR_NO_INPUT,
            .h19 = 123456, .h20 = h20, .h21 = 812, .h22 = 310, .h23 = 905, .hsds = 17};
}

int main(int argc, char **argv) {
    char buf[256];
    if (argc > 1 && !strcmp(argv[1], "dump")) {
        for (uint32_t h20: {0u, 7u, 250u}) fwrite(buf, 1, veTextBlock(buf, sizeof buf, sample(h20)), stdout);
        return 0;
    }

    for (uint32_t h20 = 0; h20 < 300; ++h20) { // walks the checksum byte through all values
        size_t n = veTextBlock(buf, sizeof buf, sample(h20));
        CHECK(n > 0);
        uint8_t sum = 0;
        for (size_t i = 0; i < n; ++i) sum += (uint8_t) buf[i];
        CHECK(sum == 0);
    }

    size_t n = veTextBlock(buf, sizeof buf, sample(42));
    std::string s(buf, n - 1);
    CHECK(s.rfind("\r\nPID\t0xA056\r\nFW\t210\r\n", 0) == 0);
    CHECK(s.find("\r\nV\t26840\r\nI\t-120\r\nVPV\t61230\r\nPPV\t0\r\n") != std::string::npos);
    CHECK(s.find("\r\nOR\t0x00000001\r\n") != std::string::npos);
    CHECK(s.find("\r\nH20\t42\r\n") != std::string::npos);
    CHECK(s.size() >= 11 && s.compare(s.size() - 11, 11, "\r\nChecksum\t") == 0);
    CHECK(n < 200);

    CHECK(veTextBlock(buf, n - 1, sample(42)) == 0); // no room for the checksum byte
    CHECK(veTextBlock(buf, n, sample(42)) == 0);     // snprintf's NUL needs the checksum's byte too
    CHECK(veTextBlock(buf, n + 1, sample(42)) == n);

    printf(fails ? "%d FAILED\n" : "vedirect-test: all passed\n", fails);
    return fails != 0;
}
