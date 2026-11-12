#pragma once

#include <cstdint>

enum class AlertLevel : uint8_t { Ok, Warn, Derate, Trip, Unverified };

/**
 * State of one alert source. On a change (and on the first call per source) logs a line and
 * publishes retained JSON {"lvl","msg","t"} to pv/alert/<hostname>/<source>. A publish that
 * can't go out (MQTT off or down) is retried by alertTick(), which also republishes every
 * source after a reconnect. source must be a string literal. Core 0 only.
 */
void alert(const char *source, AlertLevel lvl, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

void alertTick();
