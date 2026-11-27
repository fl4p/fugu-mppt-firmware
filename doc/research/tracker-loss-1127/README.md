*this document is an LLM generated placeholder*

# Tracker energy loss, fry + flat, 2026-11-23 13:29 .. 11-27 17:00 UTC

Inputs: havan console log (current + 5 rotated files, `mppt: scan: P`, `Start sweep` lines, saved as
`/tmp/scanlog.txt` by `grep -aE " (fry|flat): .*mppt: (scan: P|Start sweep|sweep (verified|flagged)|scan near)"`),
InfluxDB `ha_van/mppt` (P). Scripts: `wrong_peak_loss.py`, `sweep_cost.py` (need `INFLUX_HA_VAN_USER/_PASS`).

Yield in the window: fry 6548 Wh, flat 8283 Wh.

## Wrong local peak (found by a scan move)

21 scan moves (`, moving`, best ×1.03..×1.42). Loss per move = (x − 1) · P0 · Δt, with Δt from the last scan or
sweep that did not move (lower bound) or from the last sweep / boot (upper bound).

| device | moves | loss | share of yield |
|---|---|---|---|
| fry | 14 | 7.9 .. 64 Wh | 0.12 .. 0.98 % |
| flat | 7 | 9.7 .. 24 Wh | 0.12 .. 0.29 % |

fry's upper bound is dominated by one move (11-26 14:44, ×1.18, 36 min since the sweep) under fast clouds
(P 67 → 359 W around it), so the true value is nearer the low end. The realised P after a move (2–4 min later vs
2 min before) ranges 0.5..5.3× and is dominated by light changes, so it cannot confirm the gains.

Not counted: wrong peaks no scan ever found (each sweep resets them, ≤ 30 min).

## Sweep cost

Energy below the average of P(−40..−5 s) and P(+25..+60 s) during the 20 s after each `Start sweep`
(boot sweeps excluded, they log no start line).

| device | sweeps | loss | per sweep | share of yield |
|---|---|---|---|---|
| fry | 107 | 7.7 Wh | 72 mWh | 0.12 % |
| flat | 116 | 18.7 Wh | 161 mWh | 0.23 % |

## Result

Wrong-peak loss ≈ 0.1–0.4 % of yield (best guess near 0.2 %), the sweeps cost another 0.1–0.2 %. Together well under
1 %. More tracker work (#70, more frequent scans) can win at most a few Wh a day on these strings; more frequent
sweeps would cost about as much as they recover.
