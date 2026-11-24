*this document is an LLM generated placeholder*

# fry light-load efficiency check after rect_offset −325 ns

Start on or after 2026-11-26. InfluxDB only, no device commands.

## Background

- fry ran rect_offset_ns −1312 until the 2026-11-23 10:46 UTC OTA. Since then it runs −325 ns,
  measured by `rect-cal` on 2026-11-21 (record: `doc/research/2026-11-21-fry-rectcal-runs.md`).
- −1312 ns closed the LS too early, so the body diode conducted at the end of each DCM cycle. The
  expected effect is a small loss reduction in DCM, i.e. at light load.

## Data

`mppt` at tm.fabi.me / `ha_van` (query recipe in `CLAUDE.md`), `device='fry'`.

- Use `pwm_dcm=true`, `mppt_state` = tracking, `cv_lim_idx` = none (not limited).
- `rect_off` confirms the offset in each window: check its units first; it must change at the OTA from the old
  value to −325 ns (−52 ct).
- There is no input-current field, so Vin·Iin/Vout·Iout can't be computed. Proxies:
  1. **fry/flat output-power ratio:** bin by time of day at matched light, P < ~100 W. flat had
     no rect_offset change, so it controls for weather. Bias: other tracker changes landed in the
     same window (see `doc/research/2026-11-23-dusk-trap-field-eval.md`).
  2. **Heat at a given power:** `ntc_temp` − ambient, or `mcu_temp`, against P in DCM. Weak,
     because ambient isn't logged.

## Windows

- Before: 2026-11-20 .. 2026-11-23 10:46 UTC. Exclude the rect-cal force runs on 11-19..21.
- After: 2026-11-23 10:46 UTC .. now. Exclude OTA restarts and top-balance runs.

## Output

- Short note in `doc/research/` with the before/after ratio, the spread and the sample counts.
- Expect a difference of no more than ~1 % of DCM power. If it is within the spread, record it as
  "not resolvable" and close the item. Keep −325 ns either way, because it is a measured value.
- Update memory `project_rectification` when done.
