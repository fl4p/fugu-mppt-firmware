*this document is an LLM generated placeholder*

# KDM Sendust (KS) 125u and 60u: permeability vs DC bias

Date 2026-11-24. Scope: the `dc_bias` laws of `KDM_SendustKS_125` and `KDM_SendustKS_60` in
`pwr-mosfet-lib/maglib/materials/__init__.py`.

## Result

KDM prints the law `%mu = 100 / (a + b * H^c)` (H in Oe, %mu = percent of initial
permeability) with a coefficient table inside the Sendust DC-bias panel. The rows for the two
grades we use are:

| grade | a | b | c |
|---|---|---|---|
| 125u | 1 | 1.34E-03 | 1.780 |
| 60u | 1 | 3.57E-04 | 1.748 |

The same digits are printed in three independent renderings (the KS184-125A part sheet, the
KS series sheet `Ljf_KS.pdf`, and catalogue page 014). The maglib "very rough fit" is
**exactly these printed coefficients**. Its comment is wrong on two counts: the numbers are
the vendor's printed values, not an eyeballed fit, and the cited catalogue page 18 holds the
*Normal Magnetization* (B-H) curves. The DC-bias chart is on page **14** (printed page number
014; PDF page 14).

The drawn curves agree with the printed rows to within 0.5 %-pt rms and 0.9 %-pt max across
all three sources, so the printed law describes its own chart. Use the printed coefficients.
The free fits below are only a cross-check.

## Sources

| file (under `ee/datasheets/kdm/`) | URL | sha256 | type |
|---|---|---|---|
| `KS184-125A_semic_Ljf_T184-S-125A_KD.pdf` (existing) | (semic) | 5aa7be7a... | vector page, outlined glyphs; DC-Bias panel bottom right |
| `semic-2026-11-24/k6_Catalogue_Alloy_Powder_Core.pdf` | https://www.semic.cz/!KATEGORIE/6K/k6_Catalogue%20Alloy%20Powder%20Core.pdf | 0debee5b... | 126 pages of 128 ppi JPEG scans, no text layer. p.14 = Sendust + Si-Fe DC bias, p.18 = magnetization curves |
| `semic-2026-11-24/Ljf_KS.pdf` | https://semic.cz/!old/files/pdf_www/Ljf_KS.pdf | ca5b5e13... | 300 dpi JPEG strips; p.1 = KS DC-bias panel |
| `semic-2026-11-24/Ljf_KDM.pdf` | https://semic.cz/!old/files/pdf_www/Ljf_KDM.pdf | 2598d3dd... | Excel text, material overview: Sendust "DC Bias (%u) @100Oe = 47%" |

All three downloads came through on fetch rung 1 (plain curl, Chrome UA), with `%PDF` magic
and the expected byte counts. Nothing was blocked. kdm-mag.com was not needed.

Printed-row differences between sources: only the 26u row differs (`Ljf_KS`: 3.46E-05/1.861;
KS184 sheet and catalogue: 5.59E-05/1.825). The 60u and 125u rows are identical everywhere.
`Ljf_KS` prints 125u c as "1.78" and the other two print "1.780".

## Digitization (dsdig)

There was no dsdig class for this chart type, so I added a new one,
`digitize-powder-dc-bias` (module `powder_dc_bias.py`). It is committed as 5628e00 on
branch `powder-dc-bias` (worktree `/Users/fab/dev/pv/ee/.dsdig-dcbias`), not merged. That
commit also adds reference data, `tests/fixtures/kdm_sendust_dc_bias`, and
`tests/test_powder_dc_bias.py`. **The class is
unvalidated**: every result has status `overlay-review-required`, and the overlays below
need Fab's review before batch use.

- **Vector path:** calibrates from the field rectangle plus the white gridline strokes,
  then flattens the six stroked Bezier curves.
- **Raster path:** calibrates from the gridline gaps in the tinted field, then takes the ink
  crossings along each %mu level. Curve identity comes from left-to-right order, because the
  curves are monotone. A level that does not show all six crossings is skipped.
- **Binding check:** each curve must lie closest to its own printed row.

| run | status | x cal | y cal | binding | overlay |
|---|---|---|---|---|---|
| KS184 sheet, vector | overlay-review-required | 28 lines, 0.16 pt rms | 11 lines, 0.24 pt rms | ok | `dsdig/ks184-vector-dcbias-overlay.webp` |
| Ljf_KS p.1, raster 300 dpi | overlay-review-required (y rms flagged) | 28 lines, 0.96 px | 11 lines, 2.4 px | ok | `dsdig/ljf_ks-raster-dcbias-overlay.webp` |
| catalogue p.14, raster 400 dpi (128 ppi source), ink-max 140 | overlay-review-required (x/y rms flagged) | 21 lines, 2.0 px | 8 lines, 2.7 px | ok | `dsdig/catalog_p14-raster-dcbias-overlay.webp` |

The catalogue run uses only 10 of 19 levels. The remaining levels are lost where legend
markers or JPEG speckle change the crossing count, and those levels are skipped, not guessed.
At the default ink threshold of 110 the run was refused: only 2 levels showed every curve.

**Y-grid irregularity:** the artwork's 10 % gridlines are unevenly spaced. In the vector
source they are 15.09 pt apart, except three gaps of 14.20 pt. `Ljf_KS` shows the same
pattern, so it comes from the drawing, not from the digitizer. Calibrating piecewise through
the gridlines instead of by linear LSQ moves the fits by at most about 0.5 %-pt.
Representative piecewise fits: vector 125u b=1.399e-3, c=1.765; vector 60u b=3.621e-4,
c=1.742. This is the read uncertainty of the chart. Treat agreement below 1 %-pt as
"identical".

### Digitized points (KS184 vector trace, linear y calibration)

These points are the dsdig readings. The full sets for every grade and source are in
`dsdig/*-dcbias.json`.

| %mu | 125u H/Oe | 60u H/Oe |
|---|---|---|
| 95 | 7.64 | 17.14 |
| 90 | 11.82 | 26.18 |
| 85 | 15.55 | 34.78 |
| 80 | 19.08 | 42.76 |
| 75 | 22.46 | 50.40 |
| 70 | 25.81 | 58.19 |
| 65 | 29.23 | 66.36 |
| 60 | 32.91 | 75.10 |
| 55 | 37.00 | 84.56 |
| 50 | 41.53 | 94.94 |
| 45 | 46.51 | 106.46 |
| 40 | 52.15 | 119.41 |
| 35 | 58.55 | 134.32 |
| 30 | 66.45 | 152.57 |
| 25 | 76.77 | 175.95 |
| 20 | 90.84 | 207.61 |
| 15 | 111.38 | 254.19 |
| 10 | 145.16 | 332.39 |
| 5 | 219.52 | 506.82 |

Printed row against these points: 125u rms 0.33 / max 0.50 %-pt, 60u rms 0.29 / max 0.53 %-pt.

### Free fits `100/(1 + b H^c)`

| grade | source | b | c | fit rms %-pt |
|---|---|---|---|---|
| 125u | KS184 vector | 1.360e-3 | 1.7719 | 0.15 |
| 125u | Ljf_KS raster | 1.384e-3 | 1.7650 | 0.22 |
| 125u | catalogue p.14 raster | 1.346e-3 | 1.7741 | 0.19 |
| 60u | KS184 vector | 3.518e-4 | 1.7484 | 0.16 |
| 60u | Ljf_KS raster | 3.664e-4 | 1.7377 | 0.16 |
| 60u | catalogue p.14 raster | 3.460e-4 | 1.7517 | 0.27 |

## Independent anchors on the printed law

- **KS184-125A spec table:** "DC-Bias, φ0.8 mm/57 Ts, 20 kHz/1 V, I_DC = 7.5 A (H_DC = 50 Oe):
  344.4 µH (Min.)". The inductance row gives 913.0 µH ±8 % at 0 A, with l_e = 10.740 cm.
  - The field checks: 0.4π·57·7.5/10.74 = 50.0 Oe.
  - 344.4 / (0.92 · 913.0) = **41.0 %**. The printed law gives 41.4 % at 50 Oe, so the
    minimum is exactly the −8 % L0 times about 41 %.
  - Against nominal L0 the minimum is 37.7 %.
- **Ljf_KDM overview:** Sendust "DC Bias (%u) @100Oe = 47%". The printed 60u row gives
  47.2 % at 100 Oe, which suggests the overview quotes the 60u grade (the overview does
  not say which grade).
- **Measurement basis:** KDM measures at 20 kHz/1 V on the DC bias, a small-signal
  incremental value. This is consistent with `~/dev/kb/magnetics/vendor-perm-curves-are-small-signal.md`.

## Comparison at fixed H (% of initial permeability)

Generated by `compare.py`. Units: H in Oe on every chart and table used here. The KDM and
Magnetics x axes are labelled "oersteds" (KDM adds "1 Oe = .7958 A/cm"), and Micrometals
states "H expressed in Oe". All three use percent of initial permeability. The Magnetics and
Micrometals tables use a = 0.01 with a fractional result, so they are rescaled here to
a = 1, %mu form (b' = b/0.01).

### 125u

| source | 5 Oe | 10 Oe | 20 Oe | 30 Oe | 50 Oe | 100 Oe |
|---|---|---|---|---|---|---|
| dsdig vector trace (drawn curve) | 97.7 | 92.4 | 78.7 | 63.9 | 41.8 | 17.5 |
| dsdig free fit, vector KS184 | 97.7 | 92.6 | 78.4 | 64.0 | 41.8 | 17.4 |
| dsdig free fit, raster Ljf_KS | 97.7 | 92.5 | 78.5 | 64.1 | 42.0 | 17.6 |
| dsdig free fit, raster catalogue p.14 | 97.7 | 92.6 | 78.5 | 64.0 | 41.8 | 17.4 |
| **KDM printed 1.34e-3 / 1.780** | 97.7 | 92.5 | 78.3 | 63.7 | 41.4 | 17.0 |
| maglib current (same numbers) | 97.7 | 92.5 | 78.3 | 63.7 | 41.4 | 17.0 |
| Magnetics Kool Mu 125u (catalogue p.63) | 97.0 | 91.3 | 77.1 | 63.4 | 42.9 | 19.5 |
| Micrometals MS-125 (2021 alloy catalogue p.29) | 98.4 | 94.4 | 82.0 | 68.0 | 44.8 | 18.1 |

### 60u

| source | 5 Oe | 10 Oe | 20 Oe | 30 Oe | 50 Oe | 100 Oe |
|---|---|---|---|---|---|---|
| dsdig vector trace (drawn curve) | 99.2 | 97.9 | 93.5 | 87.8 | 75.3 | 47.7 |
| dsdig free fit, vector KS184 | 99.4 | 98.1 | 93.8 | 88.1 | 75.3 | 47.5 |
| dsdig free fit, raster Ljf_KS | 99.4 | 98.0 | 93.7 | 88.1 | 75.3 | 47.7 |
| dsdig free fit, raster catalogue p.14 | 99.4 | 98.1 | 93.8 | 88.2 | 75.3 | 47.6 |
| **KDM printed 3.57e-4 / 1.748** | 99.4 | 98.0 | 93.7 | 88.0 | 75.0 | 47.2 |
| maglib current (same numbers) | 99.4 | 98.0 | 93.7 | 88.0 | 75.0 | 47.2 |
| Magnetics Kool Mu 60u (catalogue p.63) | 99.6 | 98.5 | 94.7 | 89.5 | 76.7 | 47.7 |
| Micrometals MS-60 (2021 alloy catalogue p.29) | 99.6 | 98.5 | 94.9 | 89.9 | 77.7 | 49.3 |

The other vendors' 125u and 60u sendust grades fall within about ±4 %-pt of KDM over
5–100 Oe. Kool Mu 125u rolls off slightly faster below 30 Oe and slower above 50 Oe.

## Proposed maglib change (not applied)

The numbers stay the same; only the provenance changes, and `dc_bias_coef` is exposed the
way `MagInc_KoolMuMAX_40` does it. Leave `coef_source` alone: it is per material, and
`dc_magnetization` for KS60 is still the rough fit.

```diff
 KDM_SendustKS_60 = MagneticCoreMaterialSpecs(
     'kdm', 'KS60', mu_r=60,
 
     core_loss_density=kdm_sendust_core_loss,
-    dc_bias=lambda H_oe: 1 / (1 + 3.57e-4 * H_oe ** 1.748),
-    # this is a very rough fit from looking at this chart: https://www.semic.cz/!KATEGORIE/6K/k6_Catalogue%20Alloy%20Powder%20Core.pdf#page=18
+    # KDM's printed DC-bias row, %mu = 100/(a + b*H_oe^c), a=1 b=3.57E-04 c=1.748, H in Oe.
+    # Printed identically in the Sendust DC-bias panel of the KS184-125A sheet, Ljf_KS.pdf p.1
+    # and k6_Catalogue_Alloy_Powder_Core.pdf p.14 (NOT p.18, which is the B-H curves).
+    # dsdig (powder_dc_bias, unvalidated class) reads the drawn curve within 0.53 %-pt of it.
+    # Small-signal (20 kHz/1 V on bias). Chart drawn 1..1000 Oe, down to ~1 %.
+    # Evidence: fugu-mppt-firmware/doc/research/coil-dcbias/README.md
+    dc_bias=lambda H_oe: 1 / (1 + 3.57e-4 * H_oe ** 1.748),
+    dc_bias_coef=(0.01, 3.57e-6, 1.748),  # vendor row rescaled to the a=0.01 convention
     dc_magnetization=micrometals_dc_mag_model(60, a=9E-03, b=2E+00, c=2E+09, d=0.000E-08, e=1.6E+02),
     # NOT 'datasheet' -- see KDM_CORE_LOSS_IS_DATASHEET_BUT_COEF_SOURCE_IS_NOT below.
 )
 
 KDM_SendustKS_125 = MagneticCoreMaterialSpecs(
     'kdm', 'KS125', mu_r=125,
     ...
     core_loss_density=kdm_sendust_core_loss,
-    dc_bias=lambda H_oe: 1 / (1 + 1.34e-3 * H_oe ** 1.78),
-    # this is a very rough fit from looking at this chart: https://www.semic.cz/!KATEGORIE/6K/k6_Catalogue%20Alloy%20Powder%20Core.pdf#page=18
+    # KDM's printed DC-bias row, a=1 b=1.34E-03 c=1.780 (same three sources as KS60).
+    # dsdig reads the drawn curve within 0.50 %-pt. Part-sheet anchor (KS184-125A): 344.4 uH min
+    # at 50 Oe vs 913.0 uH +-8 % -> 41.0 % of the -8 % L0; this law gives 41.4 %.
+    dc_bias=lambda H_oe: 1 / (1 + 1.34e-3 * H_oe ** 1.780),
+    dc_bias_coef=(0.01, 1.34e-5, 1.780),
```

`dc_bias_coef` follows the a = 0.01 convention: `MagneticCoreMaterialSpecs.__init__` checks
`0.01/(a + b*H^c)` against `dc_bias` at 10 and 100 Oe. So KDM's a = 1 row is stored divided
by 100, which is the same law. The KDM chart is drawn down to about 1 %, so the default
`dc_bias_floor` of 0.25 stays valid.

## Files

- `dsdig/ks184-vector-dcbias.json`, `dsdig/ks184-vector-dcbias-overlay.webp`: primary
  (vector) reading, all six grades, with raw polylines.
- `dsdig/ljf_ks-raster-dcbias.{json,webp}`, `dsdig/catalog_p14-raster-dcbias.{json,webp}`:
  raster cross-checks.
- `*.png`: lossless copies of each overlay. Each overlay shows the digitized curves, the
  printed law (dashed) and the full calibrated grid in red with value labels.
- `run_dsdig.sh`: re-runs all three readings.
- `compare.py`: produces the comparison tables from the JSON.

Reproduce (from the dsdig worktree, `PYTHONPATH=src`, main venv):

```
ROWS="26:1,5.59e-5,1.825;40:1,1.54e-4,1.769;60:1,3.57e-4,1.748;75:1,9.47e-4,1.641;90:1,1.09e-3,1.682;125:1,1.34e-3,1.780"
python -m datasheet_chart_digitizer.cli digitize-powder-dc-bias KS184-125A_semic_Ljf_T184-S-125A_KD.pdf \
  --clip 295,500,545,690 --perms 125,90,75,60,40,26 --printed "$ROWS" --tag ks184-vector
python -m ... Ljf_KS.pdf --page 1 --clip 40,415,580,700 --dpi 300 --mode raster \
  --printed "<ROWS with 26:1,3.46e-5,1.861>" --blank 1,10,45,85 --blank 1,4,0,45 --tag ljf_ks-raster
python -m ... k6_Catalogue_Alloy_Powder_Core.pdf --page 14 --clip 90,205,520,465 --dpi 400 --mode raster \
  --ink-max 140 --printed "$ROWS" --blank 1,10,45,85 --blank 1,4,0,45 --tag catalog_p14-raster
```
