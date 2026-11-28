# L(Ipk) bins from measure-coil log lines: boost nights (11-24, 11-26) vs buck mornings (11-25, 11-27)
import re, sys, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
src = sys.argv[1] if len(sys.argv) > 1 else "mc-log-2026-11-24-28.txt"
WIN = [("2026-11-24 20:00", "2026-11-25 06:00"), ("2026-11-25 08:20", "2026-11-25 08:31"),
       ("2026-11-26 22:20", "2026-11-26 22:50"), ("2026-11-27 10:05", "2026-11-27 10:15"), ("2026-11-28 09:35", "2026-11-28 09:55")]
SESS = {"2026-11-24": "boost 11-24", "2026-11-25": "buck 11-25", "2026-11-26": "boost 11-26", "2026-11-27": "buck 11-27", "2026-11-28": "buck 11-28"}
COL = {"boost 11-24": "#f4a582", "boost 11-26": "#b2182b", "buck 11-25": "#92c5de", "buck 11-27": "#2166ac", "buck 11-28": "#1b7837", "buck 11-28 sync": "#5aae61"}
runs, cur = [], {}
BINRE = r"([\d.]+)-([\d.]+)A ([\d.]+)uH\+-([\d.]+)/"
for ln in open(src, errors="replace"):
    ts = ln[:16]
    if not any(a <= ts <= b for a, b in WIN): continue
    m = re.search(r"(fry|flat): measure-coil L0: fsw=(\d+)", ln)
    if m:
        if cur.get(m[1], {}).get("unc"): runs.append(cur[m[1]])
        cur[m[1]] = dict(dev=m[1], f=int(m[2]), sess=SESS[ts[:10]] + (" sync" if "rect=sync" in ln and ts[:10] == "2026-11-28" else ""), ts=ts, ofs=None); continue
    m = re.search(r"(fry|flat): measure-coil: on-time offset ([-+]?\d+) ns .*?: (applied|FAIL)", ln)
    if m and m[1] in cur: cur[m[1]]["ofs"] = (int(m[2]), m[3]); continue
    m = re.search(r"(fry|flat): measure-coil: L\(Ipk\)( uncorrected| at the fitted on-time offset[^,]*)?, \+-noise/offsets: (.*)", ln)
    if m and m[1] in cur:
        key = "cor" if m[2] and "fitted" in m[2] else "unc"
        cur[m[1]][key] = [tuple(map(float, b)) for b in re.findall(BINRE, m[3])]
runs += [r for r in cur.values() if r.get("unc")]

fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))
for ax, dev in zip(axs, ["fry", "flat"]):
    for r in (r for r in runs if r["dev"] == dev):
        ls = "-" if r["f"] > 35000 else ("--" if r["f"] > 25000 else ":")
        ofs = f", {r['ofs'][0]:+d} ns {r['ofs'][1]}" if r["ofs"] else ""
        for key, mfc, tag in (("unc", None, ""), ("cor", "white", " corrected")):
            if key not in r: continue
            x = [(a + b) / 2 for a, b, _, _ in r[key]]; y = [L for _, _, L, _ in r[key]]
            lab = f"{r['sess']} {r['f']/1e3:.1f} kHz{tag}{ofs if key == 'unc' else ''}"
            ax.errorbar(x, y, yerr=[e for *_, e in r[key]], color=COL[r["sess"]], ls=ls, marker="o", ms=5,
                        mfc=mfc or COL[r["sess"]], capsize=2, lw=1.5, label=lab)
            print(dev, lab, [f"{xi:.1f}A {yi:.2f}" for xi, yi in zip(x, y)])
    ax.set_title(f"{dev}: L(Ipk), filled = uncorrected, open = on-time corrected", fontsize=10)
    ax.set_xlabel("peak coil current Ipk [A]"); ax.set_ylabel("L [µH]"); ax.grid(alpha=.3)
    ax.legend(fontsize=7, title="solid 39 kHz · dashed 30 kHz · dotted 20–22 kHz", title_fontsize=7)
fig.tight_layout(); fig.savefig("fry-flat-buck-vs-boost-2026-11-28.png", dpi=130)
