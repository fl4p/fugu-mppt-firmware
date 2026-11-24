#!/bin/bash
# Re-run dsdig digitize-powder-dc-bias on the three KDM Sendust renderings (dsdig branch powder-dc-bias).
set -e
WT=${DSDIG_WT:-/Users/fab/dev/pv/ee/.dsdig-dcbias}
PY=/Users/fab/dev/pv/ee/datasheet-chart-digitizer/.venv/bin/python
R=$(cd "$(dirname "$0")" && pwd)/dsdig
K=/Users/fab/dev/pv/ee/datasheets/kdm
ROWS="26:1,5.59e-5,1.825;40:1,1.54e-4,1.769;60:1,3.57e-4,1.748;75:1,9.47e-4,1.641;90:1,1.09e-3,1.682;125:1,1.34e-3,1.780"
ROWS_KS="26:1,3.46e-5,1.861;40:1,1.54e-4,1.769;60:1,3.57e-4,1.748;75:1,9.47e-4,1.641;90:1,1.09e-3,1.682;125:1,1.34e-3,1.78"
cd "$WT"; export PYTHONPATH=src
D="$PY -m datasheet_chart_digitizer.cli digitize-powder-dc-bias"
$D $K/KS184-125A_semic_Ljf_T184-S-125A_KD.pdf --clip 295,500,545,690 --perms 125,90,75,60,40,26 --printed "$ROWS" --out $R --tag ks184-vector
$D $K/semic-2026-11-24/Ljf_KS.pdf --page 1 --clip 40,415,580,700 --dpi 300 --mode raster --perms 125,90,75,60,40,26 --printed "$ROWS_KS" --blank 1,10,45,85 --blank 1,4,0,45 --out $R --tag ljf_ks-raster
$D $K/semic-2026-11-24/k6_Catalogue_Alloy_Powder_Core.pdf --page 14 --clip 90,205,520,465 --dpi 400 --mode raster --ink-max 140 --perms 125,90,75,60,40,26 --printed "$ROWS" --blank 1,10,45,85 --blank 1,4,0,45 --out $R --tag catalog_p14-raster
