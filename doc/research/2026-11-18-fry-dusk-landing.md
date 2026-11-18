*this document is an LLM generated placeholder*

# fry dusk landing below the MPP

Date: 2026-11-18, 16:45-17:17 UTC, live converter fry (firmware a5f852f6, MCPWM, pwmMax 4103, f = 39 kHz,
C_in = 2x Rubycon ZLH 470 uF, battery ~25.8 V). Data: console replies over MQTT, InfluxDB `mppt` at ~48 Hz
(`P` = Iout_med3 * Vout_med3, the output side; `Ui` = Vin med3), console log on havan. Iin on fry is virtual
(from Iout), so every power the firmware uses is output power.

## Result

The sweep's power peak at low light is not the MPP. It is the input capacitor discharging while Vin collapses at
the knee of the DCM duty-to-Vin map. The excess is P_ex = eta * C_in * Vin * |dVin/dt|, which is 3-3.6 W at
|dVin/dt| = 40-45 V/s against a real 3.9-4.2 W. The swept peak sits at that steepest point, past the true MPP
in duty. Hypothesis B holds and is the main term. A (EWM lag) adds 17-25 counts. C holds after landing: the P&O
cannot see the slope back to the MPP. a5f852f6 did not change any of this: old-firmware dusk sweeps show the same
spike and the same landing duty.

## Static P(D)

Pass A, 16:45:32-16:46:16, descending duty, 4 s dwell, sync rect off (Iout < 0.2 A), steady light:

| duty | Vin [V] | Pout [W] |
|-----:|--------:|---------:|
| 675 | 40.4 | 3.2 |
| 650 | 44.9 | 3.3 |
| 625 | 49.6 | 3.7 |
| 600 | 52.4 | 3.9 |
| 575 | 54.1 | 3.8 |
| 550 | 55.4 | 3.9 |
| 525 | 56.3 | 3.8 |
| 500 | 57.0 | 3.8 |
| 475 | 57.7 | 3.9 |
| 450 | 58.2 | 3.9 |

Flat 3.8-3.9 W plateau from 52 to 58 V (duty 450-600); the MPP is the plateau, Voc ~65 V.

Pass C, 17:12:51-17:14:11, right after the 17:12:29 sweep, 5-7 s per point. Light was falling about 10 %/min, so
the return leg reads lower:

| duty | down: Vin / P | up: Vin / P |
|-----:|--------------:|------------:|
| 650 | 43.9 V / 3.9 W | 39.0 V / 2.8 W |
| 625 | 48.3 V / 4.1 W | 45.3 V / 3.2 W |
| 600 | 52.0 V / 4.2 W | 49.7 V / 3.7 W |
| 575 | 53.7 V / 4.0 W | 52.5 V / 3.7 W |
| 550 | 54.6 V / 3.9 W | - |

Plant step response (pass A, 48 Hz): after a 25-count step Vin settles with tau ~0.25 s and is flat within 1 s; P
follows Vin. A 4 s dwell is > 10 tau. The "jump-then-decay" seen on the dashboard during pass B (16:46-16:48,
ascending duty) was rising irradiance (P 4.3 -> 6.4 W at a constant 42-44 V), not a step transient. Pass B, the
16:49-16:51 pass and pass 3 (16:51-16:56) ran while irradiance rose from ~4 W to 14 W and partly overlap Fab's
`sync 0` and two sweeps (16:52:31-16:53:05); they are excluded from the low-light analysis.

## Sweeps vs static

| sweep | captured (P, duty, Vin) | raw med3 peak | static MPP then | landed |
|---|---|---|---|---|
| 16:44:56 | 6.5 W, 679, 48.1 V | 6.74 W @ 654, 51.2 V, dV/dt -44 V/s | 3.9 W @ 550-600, 52-58 V | ~39-41 V, 2.7-3.2 W |
| 17:12:29 | 7.5 W, 650, 53.1 V | 7.66 W @ 633, 55.1 V, dV/dt -41 V/s | 4.2 W @ 600, 52 V | 43.7 V, 3.9 W |
| 16:51:27 (10 W light) | 10.7 W, 686, 59.5 V | flat 10.0-10.7 W | 11.0-11.4 W @ 650-700, 59-62 V | 57 V, 10.3 W |

- Duty offset, captured minus static MPP: ~80-105 counts (16:45) and ~50 counts (17:12). Of that, the EWM lag
  (captured duty minus raw med3 peak duty) is 25 and 17 counts, i.e. 45-65 ms at the 380 counts/s ramp.
  The rest (raw peak past the static MPP, 33-80 counts) is the capacitor term.
- Excess fit, P_out - P_static(Vin) = a + C_eff * Vin * (-dVin/dt), over the collapse: C_eff = 1.5 mF (16:45) and
  1.0 mF (17:12, offset 1.0 W from the falling light), against 0.94 mF nominal. Output-referred C_eff = eta * C_in
  plus panel capacitance and the +20 % cap tolerance; same order, so the capacitor explains the whole excess.
- Why only at low light: the ramp is in duty, and the knee where Vin collapses is a few tens of counts wide in deep
  DCM, so |dVin/dt| reaches ~40 V/s whatever the light. The excess C*V*dV/dt is then a fixed ~2-3.5 W: +80 % at
  4 W, ~+10 % at 10 W (16:51, |dV/dt| max 24 V/s over a P plateau that hides it), negligible at 100 W.
- The swept curve (`sweep-curve`, ewm Vin) therefore shows a fake 6.5-7.5 W hump at 48-53 V. The earlier dusk study
  ("Scope" in `2026-11-18-dusk-tracker-trap.md`: "sweeps find 4-10 W at 45-60 V" while the tracker sits at 2 W) used
  these sweep peaks; at dusk they overstate the available power by 50-100 %.

## Old firmware

Low-light `Stop sweep` lines in `pv/fugu_console.log.1..5` (2026-11-11..11-18) captured duty 655-725 at 2-7 W
(and a second cluster at 337-388 at ~2 W), with sweeps of 8.7 s. The 2026-11-17 16:51:50 sweep (48 Hz data) shows
the same ramp rate near the knee (~380 counts/s), the same spike (P 1.0-1.3 W before the knee, 2.08 W at duty 652 while
Vin falls 41 -> 35 V), MPP=(2.0W,663,36.6V), and a landing at 33.9 V and ~0.8 W. a5f852f6 only shortened the
tail of the sweep; it did not change the landing.

## P&O after landing

- 16:42:12 landing at 674: the tracker stayed at 40-44 V, duty 660-700, for 2.7 min (P 4.4 -> 3.0 W with the
  light) and never climbed to the 52-58 V plateau.
- Before the 17:12 sweep the fast P&O dithered at duty 583-626, Vin 50.5-55.3 V, period ~1.5 s. The leg length is
  set by the 5 % Vin rule (tracker.h:128), not by power. In that dither, the measured P swings 4.3 <-> 5.4 W in phase
  with dVin/dt (+-6..9 V/s): C*V*dV/dt ~ 0.3-0.5 W. The true P difference across the dither is ~0.1-0.3 W
  (static table), below the cap term and the 1.5 %*(P+5) = 0.14 W threshold region, so the reversals do not
  encode the slope.
- From a landing at 43-44 V the static gain to 52 V is 0.3 W (7 %) over 50 counts, about 6 mW per count. With
  the cap term and the 5 % rule the tracker has no net drift toward it: hypothesis C holds as the reason the
  wrong landing persists until the next sweep.

## Sync rect and backflow

- In every low-light sweep the transient Iout from the capacitor discharge crosses 0.20 A at duty 595-646
  ("Current above threshold 0.20"), which enables backflow switch and sync rect. During the fade back to the
  captured duty the PV current recharges C_in, Iout dips to <= 0.05 A and both turn off again (16:41:28,
  16:45:02). The landed point then runs in diode mode at Iout 0.10-0.15 A, inside the 0.05/0.20 A hysteresis.
- No measurable effect on the plant: at dc 500, 6.5 W at 68.2-68.6 V with LS on (L=585) and off (L=320).
- `sync 0` latches nothing: `enableSyncRect(aboveThres)` (mppt.cpp:419) overrides it every MPPT/sweep tick, so
  any sweep re-enables it. Sync rect is not a cause of the landing error.

## Other firmware findings

- `mppt` after manual `dc` resumes the tracker with stale state (mppt.h:1315: PsuOff -> Mppt, no
  `resetDirection`). At 16:56:29 the tracker took over at duty 700 / 61.9 V / 11.8 W (the MPP) and walked up to
  1098 / 42 V / 9.8 W in 12 s without reversing, then dithered at 44-48 V for 2 min. Likely cause: the stale
  `_lastVin` (~42 V from before the manual session) and `_timeTakeover` make every drop count as "plant"
  (`against > 1 %`), which suppresses both the power reversal and the 5 % rule until Vin is back at the stale
  value. Not traced; a `tracker.resetDirection(.., Vin)` on the PsuOff->Mppt transition would rule it out.
- A sweep aborted by `dc` (16:52:42, 16:52:59) logged no "Stop sweep" and wiped the last good curve:
  `sweep-curve` replied `sc: none`, so the dashboard lost it.

## Fix proposals

1. Pace the ramp on dVin/dt, not duty: keep C*Vin*|dVin/dt| below ~5 % of P. With C = 1 mF, P = 4 W, Vin = 50 V:
   |dVin/dt| <= 4 V/s, ~10x slower only across the knee; at 100 W it never binds. Fits the sweep-pacing work.
2. Compensate the capture: P_cap = P_out + eta * C_in * Vin * dVin/dt (dVin/dt < 0 on the ramp), with C_in a new
   conf key (`converter.conf`). The C uncertainty (+-30 %) leaves ~1 W at 40 V/s, so combine with 1.
3. Land on the captured Vin (Vin-target fade), not on the captured duty. The swept Vin lags the static map the other
   way: captured Vin was 48.1 V (static MPP plateau 52-58 V, ~3.5 W at 48 V) and 53.1 V (static MPP 52 V). Landing on
   Vin would have given ~3.5 and ~4.2 W instead of 3.0 and 3.9 W. Cheapest change, and robust to the duty map.
4. Verify after landing: hold the landing point and a point ~50 counts lower for ~1 s each (> 3 tau), keep the
   better one. Costs ~2 s per sweep.
5. Capture lag: take the duty at the raw (med3) peak or shift by the EWM group delay; worth 17-25 counts.
6. P&O at low power: add the same C*V*dV/dt term to the tracker's power, or step-and-hold so each comparison is
   made with dVin/dt ~ 0.

## Safety record

Commands sent: read-only, `dc` 450-1400 (steps <= 50 counts), `sweep` (16:51:27, 17:12:29), `mppt` after every pass
(16:48:16, 16:51:11, 16:56:29, 17:14:16). No trips, no ADC errors, no backoff in the log. fry left in MPPT.
Two `dc` commands landed inside Fab's sweeps (dc 600 at 16:52:42 with the ramp near duty 94, dc 750 at 16:52:59 with
it at 1147), so those two were larger than 50-count steps (the `dc` fade still ramps them) and aborted the sweeps.
