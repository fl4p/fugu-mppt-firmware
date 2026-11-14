// Host build of src/conf_hash.cpp for test/host_py/test_conf_hash.py, which compares it with
// etc/conf_hash.py.  `conf-hash-test DIR` prints the conf-hash lines; `conf-hash-test -r` redacts stdin.
//   c++ -std=gnu++17 -Wall -I test/host-stub -I src -o /tmp/conf-hash-test test/host-stub/conf-hash-test.cpp

#include "../../src/conf_hash.cpp"

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    if (!strcmp(argv[1], "-r")) {
        std::string in;
        char b[256];
        size_t n;
        while ((n = fread(b, 1, sizeof(b), stdin)) > 0) in.append(b, n);
        const std::string out = confHashRedact(in);
        fwrite(out.data(), 1, out.size(), stdout);
        return 0;
    }
    uint8_t sha[32];
    std::string err;
    const int files = confHash(argv[1], sha, err, [](const char *name, const uint8_t *s) {
        printf("conf-file %s sha256=%s\n", name, confHashHex(s, 8).c_str());
    });
    if (files < 0) {
        printf("conf sha256=unknown error=%s\n", err.c_str());
        return 1;
    }
    printf("conf sha256=%s files=%d\n", confHashHex(sha, 32).c_str(), files);
    return 0;
}
