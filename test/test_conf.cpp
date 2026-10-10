// Tests for src/conf.h ConfFile getters using the in-memory map constructor (no file I/O).
// Covers multi-base integer parsing, float/string getters, defaults and the
// required-key (max-sentinel default) throw contract.

#include <unity.h>
#include <esp_log.h>
#include <Arduino.h>

#include <string>
#include <cstdio>

#include "conf.h"
#include "store.h" // mountLFS

static ConfFile makeConf() {
    return ConfFile{{
        {"dec", "42"},
        {"hex", "0xff"},
        {"bin", "0b101"},
        {"oct", "010"},
        {"flt", "3.5"},
        {"str", "hello"},
    }};
}

void test_conf_getlong_bases() {
    auto c = makeConf();
    TEST_ASSERT_EQUAL_INT(42, c.getLong("dec", 0));
    TEST_ASSERT_EQUAL_INT(255, c.getLong("hex", 0)); // 0x
    TEST_ASSERT_EQUAL_INT(5, c.getLong("bin", 0));   // 0b
    TEST_ASSERT_EQUAL_INT(8, c.getLong("oct", 0));   // leading 0 -> octal
}

void test_conf_getlong_default_when_missing() {
    auto c = makeConf();
    TEST_ASSERT_EQUAL_INT(99, c.getLong("nope", 99));
}

void test_conf_getfloat() {
    auto c = makeConf();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.5f, c.getFloat("flt", 0.f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.25f, c.getFloat("nope", 1.25f)); // default on missing
}

void test_conf_getstring() {
    auto c = makeConf();
    TEST_ASSERT_EQUAL_STRING("hello", c.getString("str").c_str());
    TEST_ASSERT_EQUAL_STRING("def", c.getString("nope", std::string{"def"}).c_str());
}

void test_conf_required_missing_long_throws() {
    auto c = makeConf();
    bool threw = false;
    try {
        c.getLong("nope"); // default == LONG_MAX sentinel -> required
    } catch (const std::runtime_error &) {
        threw = true;
    }
    TEST_ASSERT_TRUE(threw);
}

void test_conf_required_missing_string_throws() {
    auto c = makeConf();
    bool threw = false;
    try {
        c.getString("nope"); // no-default overload throws
    } catch (const std::runtime_error &) {
        threw = true;
    }
    TEST_ASSERT_TRUE(threw);
}

void test_conf_operator_bool() {
    auto c = makeConf();
    TEST_ASSERT_TRUE((bool) c);
    ConfFile empty{};
    TEST_ASSERT_FALSE((bool) empty);
}

// ---- ConfFile::remove() — file-backed, needs a writable littlefs ----

static const char *kRemovePath = "/littlefs_test/remove.conf";

// Seed kRemovePath with keys carrying inline comments, plus comment-only and blank lines.
static void writeRemoveFixture() {
    FILE *f = fopen(kRemovePath, "w");
    TEST_ASSERT_NOT_NULL(f);
    fputs("# header comment\n"
          "\n"
          "hv_v_adc=ads        # inline note on vin\n"
          "lv_v_adc=esp32\n"
          "lv_i_ch=3  # current channel\n"
          "# trailing comment\n"
          "foo=bar\n", f);
    fclose(f);
}

static std::string readFile(const char *path) {
    std::string s;
    FILE *f = fopen(path, "r");
    if (!f) return s;
    char buf[256];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) s.append(buf, n);
    fclose(f);
    return s;
}

static bool contains(const std::string &hay, const char *needle) {
    return hay.find(needle) != std::string::npos;
}

void test_conf_remove_drops_line_and_keeps_rest() {
    TEST_ASSERT_TRUE(mountLFS("littlefs_test", true));
    writeRemoveFixture();

    ConfFile conf{kRemovePath};
    TEST_ASSERT_TRUE(conf.remove("lv_v_adc"));

    auto txt = readFile(kRemovePath);
    // the whole line (key and value) is gone
    TEST_ASSERT_FALSE(contains(txt, "lv_v_adc"));
    TEST_ASSERT_FALSE(contains(txt, "esp32"));
    // everything else is preserved verbatim: comment-only lines, inline comments, other keys
    TEST_ASSERT_TRUE(contains(txt, "# header comment"));
    TEST_ASSERT_TRUE(contains(txt, "hv_v_adc=ads"));
    TEST_ASSERT_TRUE(contains(txt, "# inline note on vin"));
    TEST_ASSERT_TRUE(contains(txt, "# trailing comment"));
    TEST_ASSERT_TRUE(contains(txt, "foo=bar"));
    // in-memory view is consistent: getter falls back to default
    TEST_ASSERT_EQUAL_STRING("gone", conf.getString("lv_v_adc", std::string{"gone"}).c_str());
}

void test_conf_remove_strips_inline_comment_too() {
    TEST_ASSERT_TRUE(mountLFS("littlefs_test", true));
    writeRemoveFixture();

    ConfFile conf{kRemovePath};
    TEST_ASSERT_TRUE(conf.remove("lv_i_ch"));

    auto txt = readFile(kRemovePath);
    TEST_ASSERT_FALSE(contains(txt, "lv_i_ch"));
    TEST_ASSERT_FALSE(contains(txt, "current channel")); // the line's inline comment goes with it
}

void test_conf_remove_missing_key_returns_false() {
    TEST_ASSERT_TRUE(mountLFS("littlefs_test", true));
    writeRemoveFixture();
    auto before = readFile(kRemovePath);

    ConfFile conf{kRemovePath};
    TEST_ASSERT_FALSE(conf.remove("not_a_key"));

    TEST_ASSERT_EQUAL_STRING(before.c_str(), readFile(kRemovePath).c_str()); // file untouched
}
