#!/bin/bash
for sc in wx-dawn wx-cloud wx-cloud-cv wx-sun-fry wx-low wx-mid wx-wob-fry wx-shade2 wx-shade3 wx-shade2-move wx-shade3-move wx-shade-walk wx-shade-cloud wx-shade-learn wx-sweep-rise wx-mp-manual wx-mp-ccm wx-mp-ramp wx-mp-psu; do
for board in fry flat; do for gv in "1 1" "0.975 1"; do
  for m in "clean 0" "gauss 1" "gauss 2" "meas 0" "meas 8"; do
    for b in main v2a; do echo "$b $sc $board $gv $m"; done
  done
done; done; done
