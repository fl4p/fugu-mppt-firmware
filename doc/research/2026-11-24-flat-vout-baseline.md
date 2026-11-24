*this document is an LLM generated placeholder*

# flat Vout sense error — before-baseline for the filter-cap fix

Measured 2026-11-24 from InfluxDB `ha_van`/`mppt` (data 09:30–16:30 UTC), before fitting the
missing filter cap at flat's Vout sense divider (47k/47k into the INA226 VBUS pin; fry has it
fitted). Diagnosis and mechanism: `~/dev/ee/hw/Fugu2/HW Stories/flat.md`
"2026-11-10: Vout sense reads low while switching (missing filter cap)".

## Offset against fry, both converting

76 two-minute buckets where **both** converters run >100 W, comparing flat's `Uo` with fry's on
the same bus:

- median **−0.101 V**, mean −0.117 V, range −0.395 … +0.033 V

Not proportional to current — the discriminator against a wire/IR effect:

| flat power | buckets | median ΔV |
|------------|---------|-----------|
| 100–200 W  | 21      | −0.088 V  |
| 200–300 W  | 26      | −0.134 V  |
| 300–450 W  | 20      | −0.098 V  |
| 450–700 W  | 9       | −0.098 V  |

`corr(flat Iout, ΔV) = +0.01` over 3.7–22.7 A. A wire drop would give ≈ +1.0.

## Against the BMS pack voltage

| device | median Uo − BMS | corr(Iout, Uo − BMS) |
|--------|-----------------|----------------------|
| flat   | +0.008 V        | +0.69                |
| fry    | +0.132 V        | +0.97                |

Both carry a wire-drop term, so the BMS reference alone does not separate the sense error; the
fry comparison is the cleaner one.

## The success criterion

flat's sweep fits are rejected, fry's are not (non-null counts, last 12 h):

| device | Ro      | Rc      | Rb      | P         |
|--------|---------|---------|---------|-----------|
| flat   | 0       | 0       | 0       | 1 575 237 |
| fry    | 49 175  | 38 029  | 38 029  | 1 629 854 |

After fitting the cap, flat should start publishing `Ro`/`Rc`/`Rb` **and** the flat−fry offset
should fall to about zero. That is the before/after pair to check.

## Caveat

Today's steady-state offset (−0.10 V) is smaller than the −0.26…−0.39 V recorded on 2026-11-12.
That measurement was taken during sweeps, where the error peaks at the knee (high Vin, steep
current rise), so the two are not directly comparable. The `Ro`/`Rc` rejection is the stable
signal.
