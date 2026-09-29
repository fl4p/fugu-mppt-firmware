*this document is an LLM generated placeholder*

# Coil inductance: fry vs flat

Internal lab notes, not published. Extracted verbatim from the pre-scrub `doc/Coil Inductance Measurement.md` lines 268–362 (commit 695feee), except that the "Open questions" heading is demoted one level; the generic part is `website/docs/lab/coil-inductance.md`.

## 7. Case study: `fry` vs `flat` (two different coils)

The two field boards do **not** share a coil — assuming they did is what made the 1.57× gap look
like a sensor fault. Their hand-wound inductors:

| board  | core                  | turns | `Al`        | nominal `Al·N²`         | measured (median) |
|--------|-----------------------|-------|-------------|-------------------------|-------------------|
| `flat` | 2× stacked KS130-060A | 20–21 | 122 nH/N²   | 48.8 – 53.8 µH          | **50.9 µH**       |
| `fry`  | 2× KDM KS184-125A     | 10    | 562 nH/N²   | ~56 µH (`fisi.py`)      | **79.8 µH**       |

`flat`'s 50.9 µH lands squarely inside its own computed nominal — which *validates* its sensor and
the DCM method. `fry`'s `~56 µH` was only an unverified documentation figure (the turns/`Al` look
off — 80 µH needs ~12 turns at that `Al`); its measured ~80 µH is the better estimate, and an
independent battery-shunt cross-check finds `fry`'s current gain ≈ 1.0, so its sensor is fine too.
So the 1.57× is simply two different inductors, not a measurement error on either board.

A full step-1 duty sweep on each (`measure_coil.py --steps 600 --i-max 6`, ~550 DCM points apiece,
telnet) gives the real `L` vs `H` (PWM count) below — dashed line is each board's median, points
near the CCM boundary excluded.

```
 FRY  L/µH   (547 DCM pts, median 79.8, IQR 16%; CCM rolloff below 68 clipped)
  98.2 |      o
  96.2 |     ooo
  94.2 |    oooo
  92.2 |   ooo o            ooo
  90.2 |   oo  o            o o             ooo
  88.2 |   o   oo          o   o            o oo             oo             ooo
  86.2 |  oo    o         oo   o           o   o           oo oo           oo o
  84.2 | oo     o         o    o          oo    o         oo   o          oo  o
  82.2 | oo     o       oo     o        oo      o        oo    o         oo   o
  80.2 |-o------o-------o------o-------ooo------o-------oo-----o-------ooo-----o   median 79.8
  78.2 |oo      o      oo       o      o        o      oo       o     oo
  76.2 |o             oo        o    oo         o    oo         o    oo        o
  74.2 |         o   oo         o   oo          o   oo          o  ooo
  72.2 |         o  ooo         ooooo            oooo           oooo           o
  70.2 |         oooo             o               o                            o
  68.2 |          oo
       +------------------------------------------------------------------------
        206                                                                  736   H (PWM ct)
```

```
 FLAT L/µH   (573 DCM pts, median 50.9, IQR 11%)
  60.2 |                                                                      +
  59.0 |
  57.9 |                                             +           ++           +
  56.7 |                                +           +++         + ++          +
  55.5 |                    ++         +++++       +  ++        +  ++        ++
  54.3 |         +++        + +        +   +       +   ++      ++  +++       + +
  53.2 |         +++       ++ ++      ++   +      ++   +++     +    ++       +
  52.0 |        +   +      +   ++     +    ++     +      +     +     +++    ++
  50.8 |--------+---+-----++----+----++-----+----++-------+---+-------++----+---   median 50.9
  49.6 |++     +    +     +      +   +      +++ ++        ++ ++        +  ++
  48.5 |++     +     +   ++      ++ ++       ++++          +++         ++++
  47.3 | +    ++     ++ ++        +++
  46.1 |++   ++      ++++
  44.9 | +  ++
  43.8 |  +++
  42.6 |  ++
       +------------------------------------------------------------------------
        219                                                                  799   H (PWM ct)
```

Findings:

- **The 1.57× gap is two different coils, not a sensor error.** `flat` (KS130, ~20 t) measures
  50.9 µH — inside its own computed `Al·N²` of 48.8–53.8 µH — so its sensor *and* the DCM method are
  validated against a known nameplate. `fry` (KS184, nominally 10 t) measures ~80 µH; its documented
  56 µH disagrees and is the suspect number (turns/`Al` likely off). An independent battery-shunt
  cross-check puts `fry`'s current gain at ≈ 1.0, so its sensor is fine too. **The earlier "fake
  INA226 reading ~1.5× low" story is withdrawn.**
- **Each board's measured value is its true inductance, to a few %.** Take `flat ≈ 51 µH`,
  `fry ≈ 80 µH`. The shunt cross-check hints `flat` under-reports current ~6–7 %, which would inflate
  its `L` by the same factor and put the true value nearer the `N = 20` end (~48 µH) — a small
  correction, not a 1.5× one.
- **Both means are flat across the whole current range** (≈0.3–3 A): no downward trend, so this is
  *not* core saturation. The ±10–16 % point-to-point wiggle is the duty-pinned SR-timing
  reverse-current notch of §6, not noise in `L`.
- **Boundary breakdown (the §5 limit, observed).** Pushing `fry` and `flat` toward `H ≈ M·pwmMax`
  makes the DCM estimate diverge — `fry` rolls *down* (to ~37 µH), `flat` *up* (past ~100 µH) — as
  the waveform enters CCM and `Iout` stops obeying the DCM transfer relation. Both are artifacts of
  measuring outside DCM, not changes in the coil; the median over the clean DCM band is the result.

Practical consequence: set each board's `coil.conf::L0` to its own measured value — `flat ≈ 51e-6`
(its current `56e-6` is ~10 % high, the riskier direction per §4), `fry ≈ 80e-6` (its current
`40e-6` is very conservative, costing body-diode loss near the boundary). No sensor recalibration is
indicated for either.

## Open questions

- Confirm `fry`'s ~80 µH with an LCR meter or scoped CCM-ripple measurement (§2), and reconcile it
  with the documented 10 turns / `Al` (it implies ~12 effective turns).
- The shunt cross-check hints `flat` under-reports current ~6–7 % (see
  `charger-current-calibration-analysis`); if real, trim its `Iout` calibration and its `L0` follows.