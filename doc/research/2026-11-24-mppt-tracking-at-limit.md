*this document is an LLM generated placeholder*

# MPPT tracking on fry and flat: where the loss is (11-21..11-24)

## Summary

Tracking on fry and flat is close to what the hardware can do. The ~2.5 % "headroom" from the
30-day study (`2026-11-21-mppt-vs-fixed-voltage.md`) does not survive a direct check: its
largest part, flat sitting 1.5–5 V below the sweep-curve Vmpp at ≥100 W, is a sweep-curve
artefact. The remaining identified mechanisms total a few Wh per converter per day (≤0.5 %).
Further tracker tuning is not expected to pay.

## Analyses

| Question | Data | Result |
|---|---|---|
| MPPT vs a fixed voltage | 30 days of sweeps + 50 Hz Influx | fry +~1.5 % (shading), flat ~0 vs fixed 56 V |
| Peak scans miss better peaks (fry, afternoon shading) | 121 scans 11-20/21 | 0.3–1.8 Wh on 11-20, 0 on other days; fixed 3 % move threshold, probes read low on the far peak. Lowering it to 1 % is below re-measurement noise (p90 1.9 %): not done |
| fry tracks on the high-V flank (11-20 16:09–16:19) | dither P(V) fits, 42 windows | Not a fault: the full-string peak moved to 73–75 V as the sun came out; op point 0.23 V below the fitted vertex on average |
| Dawn, dusk, overcast | log events + 1 s Influx 11-14..11-23 | 2–5 Wh/day per converter in total; no-MPP backoff doubling costs nothing measurable; no overcast-specific mechanism |
| flat ≥100 W operating-point offset | `scan near:` probes (scan_near_v, 912/929), 11-24 | At peak: see below |

## Near-point probes (11-24, scans at ≥100 W)

Each peak scan measures power at Vin−2 V and Vin+2 V relative to the operating point
(`tracker.conf::scan_near_v`, log line `scan near:`). A ratio above 1 on one side means the
peak lies that way.

| Converter | Scans | Both sides < 1 | +2 V > 1.01 | −2 V > 1.01 | Median +2 V / −2 V |
|---|---|---|---|---|---|
| flat | 14 | 11 | 1 | 0 | 0.986 / 0.962 |
| fry | 19 | 12 | 0 | 2 | 0.972 / 0.993 |

If flat sat 1.5–5 V below its peak, the +2 V ratio would exceed 1 in most scans. It did once.

## Caveats

- One day of probe data; recheck on a clear day with more scans if the question returns.
- Iin is a virtual sensor (fixed 97 % efficiency), so converter losses are not visible in
  these logs; this note covers tracking only.
- The 30-day study's per-band loss figures above 100 W are biased by the same sweep-curve
  effect (sweep curves read ~2 % high against steady points on flat).

## Remaining energy items outside tracking

- flat `highI/highD-bflowOpen` protection trips near a full pack (11-14, 11-23), estimated
  ~165 Wh by the other-converter proxy; the pack was at 27.4–28.2 V, so much of it may not
  have been absorbable. Under separate review.
- Pre-dawn Vin held within 0.7 V of Vout for 10–24 min every day: unexplained, no measurable cost.
