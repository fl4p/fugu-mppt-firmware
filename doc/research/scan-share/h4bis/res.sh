#!/bin/bash
# res.sh FILE... -> one result line per output
for f in "$@"; do
  k=$(basename "$f" .txt)
  h=$(sed -n 's/^== TRACKER.*harvest \([0-9.]*\) %.*/\1/p' "$f" | head -1)
  pr=$(sed -n 's/^== PROBES[^|]*wx-shade-gen *\([^|]*\)|.*/\1/p' "$f" | tr -d ' -')
  sc=$(sed -n 's/^== SCAN wx-shade-gen *\([0-9]*\) scans.*/\1/p' "$f")
  kv=$(python3 /tmp/h4bis/vs0.py "$f" | awk '{print $NF}')
  echo "$k h2=$h scans=$sc kvoc=$kv probes=$pr"
done
