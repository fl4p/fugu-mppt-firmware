---
name: arduino-ledc-collides-with-gate-ledc
description: Any Arduino ledcAttach/analogWrite (e.g. the `anaw` console cmd) grabs LEDC ch0 + timer0, which the raw-IDF LEDC gate driver owns — retimes/steals the live HS/LS PWM
metadata:
  type: project
---

The LEDC gate driver (src/pwm/ledc.h) configures LEDC timer 0 and channels 0 (Ctrl/HS) and 1 (Rect/LS) through raw IDF
calls, so arduino-esp32's `ledc_handle.used_channels` and periman know nothing about them. The first Arduino
`ledcAttach()` (analogWrite → ledcAttach(pin, 1000 Hz, 8 bit)) therefore picks channel 0 and `find_free_timer()` returns
timer 0: timer 0 is reconfigured to 1 kHz / 8 bit and channel 0 is rerouted. Gate duty counts (~2047 scale) then exceed
the 255 period, so LS sits at 100 % while HS follows the analogWrite duty: shoot-through on a live LEDC-driven converter.
Found auditing the B4 pin-guard fix on 2026-09-29; a free-pin check does not fix it (the conflict is channel/timer, not pin).
Fan code's ledcAttach is commented out (cooling.h), so `anaw` is currently the only Arduino LEDC user.

**Why:** pwm_driver defaults to ledc; only fry/flat run mcpwm.
**How to apply:** refuse Arduino LEDC use while the LEDC gate driver is active, or pin the diagnostic to channel >= 2
AND a timer >= 1 explicitly (ledcAttachChannel alone is not enough; the timer search still returns 0).
