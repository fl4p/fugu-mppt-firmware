*this document is an LLM generated placeholder*

# Charge termination: field notes removed from the published page

Moved out of `website/docs/guide/charging/termination.md` on 2026-11-24 because they are
unsourced field observations (no capture or log referenced). Kept here so the record is not lost.

- Premature termination from a current-dependent EOC target: "observed in the field in July 2026
  at 3.40 V/cell". The published page keeps the mechanism (pin lowers current, which lowers
  `v_term`, which lowers the pin) without the anecdote.
- CV hold motivation: "on a 280 Ah LFP pack at 30 A the line can trip at about 90 % SoC".
- Approach overshoot: "at 40 A into a 280 Ah pack the highest cell peaked about 28 mV above
  `cv_eoc`" for a hold started at full current.

Still on the published page and also unsourced: the short-term cell resistance "about 2–3 mΩ
near the top for a 280 Ah cell" in the parallel-charger gain argument (also in a `src/charger.h`
comment as "field 2-3 mOhm").

Corrected figures (2026-11-24): with the firmware defaults (`cv_eoc` 3.5 V, `cv_float`
3.325 V, `tail_c_rate` 0.05) and 280 Ah, r = 0.175 V / 14 A = 12.5 mΩ, and the cell-loop
bound R < r/1.5 is 8.3 mΩ. The page previously said 10.7 mΩ and 7 mΩ. `src/charger.h` comments
still carry wrong values: "~20mΩ" in `Li_ChgTerminationCondition::update` and "7 mOhm at
280 Ah" above `_cvHoldStep`.

Nordkyn (https://nordkyndesign.com/charging-marine-lithium-battery-banks/, fetched 2026-11-24):
manufacturer termination 3.65 V @ 0.033C (older) or 0.05C (newer cells); full cell at rest
~3.37 V OCV (termination current zero there); 3.50 V/cell "a very good conservative charging
voltage"; 3.325 V/cell a practical voltage for supplying loads while preserving reserve.
