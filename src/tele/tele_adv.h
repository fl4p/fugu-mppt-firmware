#pragma once

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif


// Connectionless telemetry broadcast (CONFIG_FUGU_WITH_BLE_ADV): a packed snapshot record in the
// manufacturer-data AD of the legacy advertisement, so any number of passive observers
// receive telemetry without connecting (MAX_CONNECTIONS=1 makes the NUS link exclusive).
// While a client is connected the tick switches to a non-connectable advertisement so
// the broadcast never pauses. tele.conf::adv_ms sets the payload refresh (0 = off).
//
// Record (little-endian, 17 B, mfr AD company id 0xFFFF):
//   u8 magic=0xF7, u8 seq, f16 Ui Uo I P, i8 mcu_temp ntc_temp,
//   u16 pwm_duty, u16 lag_us, u8 state (mppt_state | cv_lim_idx<<4)

#include "../util.h"   // time_ms

#ifdef CONFIG_FUGU_WITH_BLE_ADV

void teleAdvInit();                 // read tele.conf (from bleConsoleBegin)
void teleAdvTick(time_ms nowMs);    // reconcile adv mode + refresh payload (network loop)
bool teleAdvOwnsAdv();              // true while our non-connectable advertisement runs
bool teleAdvEnabled();

#else

inline void teleAdvInit() {}
inline void teleAdvTick(time_ms) {}
inline bool teleAdvOwnsAdv() { return false; }
inline bool teleAdvEnabled() { return false; }

#endif
