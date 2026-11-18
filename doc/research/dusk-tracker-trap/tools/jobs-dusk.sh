#!/bin/bash
# jobs-dusk.sh BUILD [scenario]
b=$1 sc=${2:-wx-dusk}
for board in fry fry-hq flat flat-hq; do for gv in "1 1" "0.983 1" "0.975 1" "1 1.02"; do
  echo "$b $sc $board $gv clean 0"
  for k in 1 2 3 4 5 6 7 8 9; do echo "$b $sc $board $gv gauss $k"; done
  for k in 0 2 4 6 8 10 12 14 16; do echo "$b $sc $board $gv meas $k"; done
done; done
