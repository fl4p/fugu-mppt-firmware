*this document is an LLM generated placeholder*

# MPPT tracking on fry and flat: where the loss is (11-21..11-24)

## Summary

Open for bright conditions. The identified mechanisms below ~300 W total a few Wh per
converter per day (≤0.5 %). The largest item from the 30-day study
(`2026-11-21-mppt-vs-fixed-voltage.md`), flat sitting 1.5–5 V below the sweep-curve Vmpp,
~22 Wh/day above 300 W, is neither confirmed nor disproven: the sweep-curve bias that would
explain it is unproven, and the near-point probes of 11-24 cannot detect an offset of that
size and never ran in the bright regime. Independent review:
`2026-11-24-mppt-tracking-at-limit.codex-review.md`.

## Analyses

| Question | Data | Result |
|---|---|---|
| MPPT vs a fixed voltage | 30 days of sweeps + 50 Hz Influx | fry +~1.5 % (shading), flat ~0 vs fixed 56 V |
| Peak scans miss better peaks (fry, afternoon shading) | 121 scans 11-19..11-21 | 0.3–1.8 Wh on 11-20, 0 on other days; fixed 3 % move threshold, probes read low on the far peak. Lowering it to 1 % is below re-measurement noise (p90 1.9 %): not done |
| fry tracks on the high-V flank (11-20 16:09–16:19) | dither P(V) fits, 42 windows | Not a fault: the full-string peak moved to 73–75 V as the sun came out; op point 0.23 V below the fitted vertex on average |
| Dawn, dusk, overcast | log events + 1 s Influx 11-14..11-23 | 2–5 Wh/day per converter in total; no-MPP backoff doubling costs nothing measurable; no overcast-specific mechanism |
| flat ≥300 W operating-point offset | 30-day per-band loss; `scan near:` probes 11-24 | Unresolved, see below |

## Near-point probes (11-24, scans at ≥100 W)

Each peak scan measures power at Vin−2 V and Vin+2 V relative to the operating point
(`tracker.conf::scan_near_v`, log line `scan near:`). A ratio above 1 on one side means the
peak lies that way.

| Converter | Scans | Both sides < 1 | +2 V > 1.01 | −2 V > 1.01 | Median +2 V / −2 V |
|---|---|---|---|---|---|
| flat | 14 | 11 | 1 | 0 | 0.986 / 0.962 |
| fry | 19 | 12 | 0 | 2 | 0.972 / 0.993 |

flat's +2 V ratio exceeded 1 three times (1.009, 1.002, 1.024). This does not show flat at
its peak:

- Both references (p0, p) are taken after the probes, so changing light shifts both ratios.
  At 11:15:19 flat's +2 V probe read x0.936 while power rose 291→312 W during the scan; against
  the earlier level it was a gain.
- With flat's reference variation (p90 1.5 %, p95 3.5 %) and a broad-peak quadratic, a probe
  reliably reveals an offset only from ~2.7–4.8 V (1.5–5 % loss). A 1.5–3 V offset gives a
  +2 V gain of 0.4–1.8 %, inside the noise.
- Median reference power of flat's 14 scans was 127 W; none started at ≥300 W, where 73 % of
  flat's energy is. A scan, and with it the near probe, is skipped when no other peak could
  beat the operating point (`src/peak_scan.h`, `if (!nc)`), which is the normal case with the
  full string conducting in bright light.

## 30-day study band loss

The ~22 Wh/day above 300 W reproduces at 21.4 Wh/day without sweep bins and 22.1 Wh/day on
fresh non-sweep bins. A uniform sweep-power scaling cancels in the normalisation; a
voltage-dependent bias of ~2.25 % in the right direction would erase it, but none is
established. The earlier evidence for a ~2 % sweep bias came from probes 10–16 V below the
operating point, not from Vmpp.

## Caveats

- One day of probe data.
- Iin is a virtual sensor (fixed 97 % efficiency), so converter losses are not visible in
  these logs; this note covers tracking only.

## Next step

Log-only probe changes so it can answer the question: run the near probe also when the scan
has no distant candidate, and bracket it with a reference measured before and after, dropping
probes where the light changed. Then collect clear days on fry and flat.

## Remaining energy items outside tracking

- flat `highI/highD-bflowOpen` trips near a full pack (11-14, 11-23): ~0 Wh lost, the pack was
  at termination or in CV hold. They are not the accepted join-at-float kind: a duty cut at
  240–640 W (termination latch, `dc 0`, `measure-coil`, selftest, OTA) gives a few negative
  current samples, the reverse-current branch opens the back-flow switch at 4–13 A and the
  next protect tick trips. Fix optional (log noise, switch opened under current).
- Pre-dawn Vin held within 0.7 V of Vout for 10–24 min every day: unexplained, no measurable cost.
