#!/bin/bash
# rev_k decision from adcscope noise captures: per capture dir build noise.fnz and noise-slow.fnz (--keep-slow) with
# noise_profile.py, then run wx-low/wx-dusk/wx-mid at three noise offsets against a baseline binary and the
# noise-threshold binary at several rev_k, and print the harvest/reversal deltas per scenario.
#   test/host-stub/noise_kscan.sh CAPDIR...
# CAPDIR: <ch>_<rate>.npz (vin, vout, iout), capture.json (channels.<ch>.rate_hz; else the rate in the file
# name), telemetry.json (InfluxDB /query JSON with P, Ui, Uo, I over the capture). Raw lines go to CAPDIR/kscan.txt.
# Env: SIM_A (baseline, /tmp/mppt-sim-study), SIM_K (/tmp/mppt-sim-nt5), KS ("1.4 2.0 2.5"), OFS ("0 5 10"),
#      SCEN ("wx-low wx-dusk wx-mid"), J (4), PY (python with numpy+scipy).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM_A=${SIM_A:-/tmp/mppt-sim-study}
SIM_K=${SIM_K:-/tmp/mppt-sim-nt5}
KS=${KS:-"1.4 2.0 2.5"}
OFS=${OFS:-"0 5 10"}
SCEN=${SCEN:-"wx-low wx-dusk wx-mid"}
J=${J:-4}
if [ -z "${PY:-}" ]; then
    PY="$ROOT/.venv/bin/python3"
    [ -x "$PY" ] || PY="$(git -C "$ROOT" rev-parse --path-format=absolute --git-common-dir)/../.venv/bin/python3"
fi
[ $# -ge 1 ] || { echo "usage: $0 CAPDIR..." >&2; exit 2; }
for b in "$SIM_A" "$SIM_K"; do [ -x "$b" ] || { echo "missing sim binary $b" >&2; exit 2; }; done
"$PY" -c "import numpy, scipy" || { echo "need numpy+scipy in $PY" >&2; exit 2; }

# one run: out-file fnz exp ofs scenario variant(A|k)
run1() {
    local out=$1 fnz=$2 exp=$3 ofs=$4 sc=$5 var=$6
    local cmd=(env -u SIM_CONF "$SIM_A")
    [ "$var" = A ] || cmd=(env SIM_CONF="tracker.conf:rev_k=$var" "$SIM_K")
    {
        echo "### fnz=$(basename "$fnz") exp=$exp ofs=$ofs scen=$sc var=$var"
        SIM_NOISE=$fnz SIM_NOISE_EXP=$exp SIM_NOISE_OFS=$ofs "${cmd[@]}" "$sc" 2>&1 |
            grep -E "^(== (SUMMARY|TRACKER)|# noise|SIM_NOISE)" || echo "== FAILED"
    } > "$out"
}
export -f run1
export SIM_A SIM_K

t0=$(date +%s)
for cap in "$@"; do
    cap=${cap%/}
    echo "######## $cap"
    [ -f "$cap/telemetry.json" ] || { echo "missing $cap/telemetry.json" >&2; exit 2; }
    "$PY" - "$cap/telemetry.json" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))["results"][0]["series"][0]
col = {c: i for i, c in enumerate(s["columns"])}
def m(c):
    v = [r[col[c]] for r in s["values"] if c in col and r[col[c]] is not None]
    return sum(v) / len(v) if v else float("nan")
print(f"telemetry: {len(s['values'])} points, P {m('P'):.1f} W, I {m('I'):.3f} A, Ui {m('Ui'):.2f} V, Uo {m('Uo'):.2f} V")
EOF
    "$PY" "$ROOT/test/host-stub/noise_profile.py" "$cap/noise.fnz" "$cap:$cap/telemetry.json" | sed 's/^/  noise      /'
    "$PY" "$ROOT/test/host-stub/noise_profile.py" "$cap/noise-slow.fnz" "$cap:$cap/telemetry.json" --keep-slow |
        sed 's/^/  noise-slow /'
    tmp=$(mktemp -d)
    n=0
    {
        for fnz in "$cap/noise.fnz" "$cap/noise-slow.fnz"; do
            for sc in $SCEN; do for o in $OFS; do for v in A $KS; do
                echo "$tmp/$((n += 1)).txt $fnz 0 $o $sc $v"
            done; done; done
            for v in A $KS; do echo "$tmp/$((n += 1)).txt $fnz 1 0 wx-low $v"; done
        done
    } > "$tmp/jobs"
    echo "  $(wc -l < "$tmp/jobs" | tr -d ' ') runs, -P $J"
    xargs -P "$J" -L1 bash -c 'run1 "$@"' _ < "$tmp/jobs"
    for i in $(seq 1 "$n"); do cat "$tmp/$i.txt"; done > "$cap/kscan.txt"
    rm -rf "$tmp"
    "$PY" - "$cap/kscan.txt" "$KS" <<'EOF'
import re, sys
from collections import defaultdict
ks = sys.argv[2].split()
runs = defaultdict(dict)  # (fnz, exp, scen) -> {(var, ofs): metrics}
key = None
fails = 0
for line in open(sys.argv[1]):
    m = re.match(r"### fnz=(\S+) exp=(\S+) ofs=(\S+) scen=(\S+) var=(\S+)", line)
    if m:
        fnz, exp, ofs, sc, var = m.groups()
        key = ((fnz, exp, sc), (var, ofs)); runs[key[0]][key[1]] = {}; continue
    d = runs[key[0]][key[1]]
    if line.startswith("== FAILED") or line.startswith("SIM_NOISE"): fails += 1
    for name, rx in (("h", r"harvest\s+([\d.]+) % of GMPP"), ("rev", r"([\d.]+) reversals/min"),
                     ("thf", r"theta/P fast ([\d.]+)"), ("ths", r"theta/P fast [\d.]+ slow ([\d.]+)")):
        mm = re.search(rx, line)
        if mm and name not in d: d[name] = float(mm.group(1))
nan = float("nan")
mean = lambda v: sum(v) / len(v) if v else nan
for exp in ("0", "1"):
    for fnz in sorted({k[0] for k in runs}, key=lambda f: (len(f), f)):
        rows = [k for k in runs if k[0] == fnz and k[1] == exp]
        if not rows: continue
        print(f"\n{fnz}, SIM_NOISE_EXP={exp} " + ("(Iout noise independent of current)" if exp == "0" else
              "(Iout noise scaled by Iout/iRef, comparison only)"))
        hdr = f"{'scenario':9} {'A %':>6} |" + "".join(f" k={k}: {'dH':>6} {'worst':>6} {'dRev':>6} |" for k in ks)
        hdr += "".join(f" th/P k={k} f/s" for k in ks if k in ("1.4", "2.5"))
        print(hdr)
        for k3 in sorted(rows, key=lambda k: k[2]):
            r = runs[k3]
            ofs = sorted({o for (v, o) in r if v == "A"}, key=float)
            a = {o: r[("A", o)] for o in ofs}
            row = f"{k3[2]:9} {mean([a[o].get('h', nan) for o in ofs]):6.2f} |"
            th = ""
            for k in ks:
                dh = [r.get((k, o), {}).get("h", nan) - a[o].get("h", nan) for o in ofs]
                dr = [r.get((k, o), {}).get("rev", nan) - a[o].get("rev", nan) for o in ofs]
                row += f"   {'':{len(k)}} {mean(dh):+6.2f} {min(dh):+6.2f} {mean(dr):+6.1f} |"
                if k in ("1.4", "2.5"):
                    tf = mean([r.get((k, o), {}).get("thf", nan) for o in ofs])
                    ts = mean([r.get((k, o), {}).get("ths", nan) for o in ofs])
                    th += f"  {tf:6.2f}/{ts:<6.2f}"
            print(row + th + f"   ({len(ofs)} offset{'s' if len(ofs) > 1 else ''})")
if fails: print(f"\n{fails} run(s) FAILED, see {sys.argv[1]}")
EOF
done
echo "runtime $(($(date +%s) - t0)) s"
