// Regression tests for three security/robustness fixes:
//   #7  src/util.cpp::strntof()  — out-of-bounds read on an empty (len==0) payload.
//   #8  src/storage/key-value.h::KeyValueStorage::readString() — abort/boot-loop on a value
//        longer than the old fixed 64-byte buffer.
//   #9  src/console.cpp::consoleDispatchAsync — network console commands must be DEFERRED
//        (enqueue_task), not run inline in the transport callback (UAF when the command tears it down).

#include <unity.h>
#include <Arduino.h>

#include <cmath>
#include <string>

#include "util.h"            // strntof
#include "logging.h"         // enqueue_task, process_queued_tasks
#include "storage/key-value.h"

#include "console.h"           // consoleDispatchAsync

// Owned by test/main.cpp (the instrumented handleCommand stub).
extern int g_handleCommandCalls;
extern String g_lastHandleCommand;
extern KeyValueStorage nvs;

// ------------------------------------------------------------------
//  #7 strntof — empty / terminated / unterminated input
// ------------------------------------------------------------------

void test_strntof_empty_returns_nan_no_oob() {
    // len==0 must NOT read dat[-1]; it returns NAN. (Pass a valid pointer so any stray read of
    // dat[-1] would be into readable-but-wrong memory — the assert is the contract, the fix is
    // the guard.)
    const char *p = "x";
    TEST_ASSERT_TRUE(std::isnan(strntof(p, 0)));
    TEST_ASSERT_TRUE(std::isnan(strntof("", 0)));
    TEST_ASSERT_TRUE(std::isnan(strntof(p, -1)));
}

void test_strntof_parses_nul_terminated() {
    // "3.65" is {'3','.','6','5','\0'}; len=5 sees dat[4]=='\0' -> fast strtof path.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.65f, strntof("3.65", 5));
}

void test_strntof_parses_unterminated_slice() {
    // dat[len-1] != '\0' -> strndup path. Slice the first 4 chars of a longer, non-terminated run.
    const char buf[] = "3.65xxxx";
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.65f, strntof(buf, 4));
}

// ------------------------------------------------------------------
//  #8 NVS readString — long value must round-trip, not abort
// ------------------------------------------------------------------

void test_nvs_readstring_roundtrips_long_value() {
    nvs.init();
    nvs.open();
    // 200 chars: well over the old 64-byte buffer that returned ESP_ERR_NVS_INVALID_LENGTH ->
    // ESP_ERROR_CHECK abort. A user-settable key this long (e.g. hostname) used to boot-loop.
    std::string longVal(200, 'h');
    nvs.writeString("sec_long", longVal);
    std::string got = nvs.readString("sec_long", "DEFAULT");
    TEST_ASSERT_EQUAL_INT(200, (int) got.size());
    TEST_ASSERT_EQUAL_STRING(longVal.c_str(), got.c_str());
}

void test_nvs_readstring_short_and_missing() {
    nvs.init();
    nvs.open();
    nvs.writeString("sec_short", "abc");
    TEST_ASSERT_EQUAL_STRING("abc", nvs.readString("sec_short", "X").c_str());
    // Absent key returns the default, not an abort. (NVS keys are <=15 chars.)
    TEST_ASSERT_EQUAL_STRING("DEF", nvs.readString("sec_absent", "DEF").c_str());
}

// ------------------------------------------------------------------
//  #9 telnet command is deferred, not run inline
// ------------------------------------------------------------------

void test_telnet_command_is_deferred_then_runs_on_drain() {
    process_queued_tasks();             // drain anything left by earlier tests
    int before = g_handleCommandCalls;

    consoleDispatchAsync("status", 6);
    // The whole point of the UAF fix: handleCommand must NOT have run inline.
    TEST_ASSERT_EQUAL_INT(before, g_handleCommandCalls);

    // loopNetwork_task drains the queue -> the command runs now, off the transport callback.
    process_queued_tasks();
    TEST_ASSERT_EQUAL_INT(before + 1, g_handleCommandCalls);
    TEST_ASSERT_EQUAL_STRING("status", g_lastHandleCommand.c_str());
}

void test_telnet_blank_command_is_dropped() {
    process_queued_tasks();
    int before = g_handleCommandCalls;
    consoleDispatchAsync("   ", 3);  // trims to empty -> skipped, never reaches handleCommand
    process_queued_tasks();
    TEST_ASSERT_EQUAL_INT(before, g_handleCommandCalls);
}
