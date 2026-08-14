---
name: pd-control-td-and-dt-traps
description: Degenerate-input traps in PD_Control's Td path and in mppt.cpp's lastUs/dt plumbing — negative Td flips the D sign, Kp=0 kills D, lastUs tracks PWM updates not controller calls
metadata:
  type: project
---

Traps found reviewing the `converter.conf::ctrl_<n>_{kp,kd,td}` refactor of `src/pd_control.h` +
`src/mppt.cpp`. Companion to [[pd-control-loop-gain-anatomy]].

**`Td < 0` is accepted and inverts the D sign.** `PD_Control::update()` gates on
`std::isfinite(Td)`, which is true for negatives. `loadCtrlGains()` logs the value but validates
nothing. Worked example (Vout unit, Kp=1500 Kd=12000, 2 % step): legacy `+270`, `Td=-4.6 ms` gives
`-209.6` — the limiter's verdict flips sign. These are protection limiters on live converters
(fry/flat) and the value comes from an editable `.conf` on littlefs.
**How to apply:** any conf-sourced derivative *time* must be rejected (or treated as unset) when
`< 0`; `isfinite()` alone is not a validity check for a time constant.

**`Td` finite with `Kp == 0` silently deletes the D component** (`D = Kp*Td*de/dt`). The migration
identity `Td = (Kd/Kp)*Ts` is undefined at `Kp=0`, so a D-only controller cannot be expressed in Td
form. No warning is emitted.

**`ctrl_*_kp` / `_kd` accept `nan`/`inf` from the conf file.** `strtof("nan")` parses cleanly, so
`ConfFile::getFloat` returns NaN with no complaint — the converter then trips
`shutdownDcdc("ctrl-nan")` forever with nothing naming the bad key. `getFloat` already has a
`warnIfNan` third parameter; it is not used on this path.

**`ConfFile::getFloat(key, NAN)` correctly preserves NaN as a default.** `getX` throws only when
`def == numeric_limits<T>::max()`, and `NaN == FLT_MAX` is false. So "absent `_td` keeps NaN =
legacy behavior" holds.

**`lastUs` means "time of the last PWM update", not "time of the last controller call".** In
`MpptController::update()` the controller loop runs at ~line 133 but `lastUs = nowUs` is at ~line
293, and there is a `return` in between (the `_sweeping && converter.disabled() && inBackoff()`
path). On that tick `_prevE` advances while `lastUs` does not, so the next `de/dt` is scaled by the
wrong interval. Harmless while `Td` is unset (dt is ignored), and only mis-scales D otherwise — but
any new early return between those two points widens the hole. Prefer a separate `lastCtrlUs`
stamped right after the controller loop.

**dt cannot go negative or stale-read.** `wallClockUs()` returns a *reference* to the global
`loopWallClockUs_`, which is only written at the top of the RT loop (`main.cpp`) and in
`setupSensors()` — never inside `update()`. So computing `dt` at the top of the function is
identical to computing it at the bottom, and the uint64 subtraction cannot wrap.

**Re-associating a constant chain is not bit-exact.** Folding `cv*(1.f/2000.f)*pwmMax*dt*25.f*2.f`
into `cv*kCtrlSlewLimit*pwmMax*dt` changes results by up to ~3.5e-7 relative (~3 ulp of float32);
~43 % of sampled operand sets differ in the last bits. No constant reproduces the original exactly,
because the `*25` originally happened *after* the `*pwmMax` and `*dt`. Physically irrelevant here,
but do not claim "bit-for-bit" for such a refactor.
