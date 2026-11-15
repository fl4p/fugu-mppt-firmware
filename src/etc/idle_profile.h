#pragma once
// Idle (night) power profile: lower CPU clock, slower connectable BLE advert and telemetry point rate
// after 5 min with the converter and every listed activity off; restored on the first busy tick.

#include <cstdint>

constexpr uint32_t kTeleMinPointUs = 20'000;
extern uint32_t g_teleMinPointUs; // min interval between telemetry points
// sparse points (>= 1 s apart) carry every periodic field and leave without batching
inline bool teleSparse() { return g_teleMinPointUs >= 1'000'000; }

void idleProfileSuspend();       // leave and stop the profile until reboot (manual `pwr` knobs)
void idleProfileHold(bool hold); // leave now and stay out while held (blocking HTTP OTA)
bool idleProfileOn();
