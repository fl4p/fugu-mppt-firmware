#!/usr/bin/env python3
"""Figures for website/docs/internals/lfp-longevity.md.

Run from anywhere: .venv/bin/python3 website/scripts/plots/lfp_longevity.py [--check]
Writes SVGs to website/docs/internals/img/lfp-longevity/; --check renders to a temp dir and fails if they differ.
Every value below is taken from the page (section named per constant); nothing is digitised.
"""
import os
import re
import sys
import tempfile

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../docs/internals/img/lfp-longevity")

# §4.1 Average SoC, table [2]: rank order best -> worst, (lo %, hi %, average SoC %)
WINDOWS = [(0, 25, 12.5), (0, 60, 30), (0, 80, 40), (0, 100, 50), (75, 100, 87.5)]

# §3.1, Keil et al. [1] Fig. 2c: A123 APR18650M1A, relative capacity after about 9 months of storage,
# (storage SoC %, relative capacity) per temperature °C. Source Keil 2016 Fig. 2c, digitized from the PDF vector
# paths with dsdig, overlay approved 2026-11-23. 50 °C has no 0 % or 45 % point in the figure.
KEIL_2C = {
    25: [(0, 0.9990), (5, 0.9935), (10, 0.9846), (20, 0.9786), (30, 0.9717), (40, 0.9686), (45, 0.9707),
         (50, 0.9715), (55, 0.9710), (60, 0.9683), (65, 0.9649), (70, 0.9655), (80, 0.9409), (90, 0.9414),
         (95, 0.9429), (100, 0.9418)],
    40: [(0, 0.9778), (5, 0.9636), (10, 0.9522), (20, 0.9429), (30, 0.9376), (40, 0.9287), (45, 0.9249),
         (50, 0.9321), (55, 0.9267), (60, 0.9279), (65, 0.9282), (70, 0.9229), (80, 0.9021), (90, 0.8946),
         (95, 0.8984), (100, 0.9045)],
    50: [(5, 0.9255), (10, 0.9200), (20, 0.9111), (30, 0.8982), (40, 0.8838), (50, 0.8823), (55, 0.8819),
         (60, 0.8839), (65, 0.8801), (70, 0.8843), (80, 0.8654), (90, 0.8598), (95, 0.8565), (100, 0.8520)],
}

# §3.1 / §8 partial_charge row: calendar step at 73 % SoC in the LFP cell of [1], cell-dependent
STEP_SOC = 73
# §3.3 / §8: manufacturer's recommended operating range [8]
MFR_RANGE = (10, 90)
# §3.1: storage fade hardly differed between 37.5 and 62.5 % SoC at 40 °C [3]
FLAT_RANGE, FLAT_TEMP = (37.5, 62.5), 40
# §8 holiday_charge row: holiday_charge 0.6 (example, default off), holiday_band 0.1 -> about 50-60 % SoC
HOLIDAY_CHARGE, HOLIDAY_BAND = 0.6, 0.1
# §8 settings table: recharge_dod default 0.20; partial_charge defaults to off, 0.8 is an example value
PARTIAL_EXAMPLE, RECHARGE_DOD = 0.8, 0.20

# BLUE, AQUA, ORANGE: dataviz validate_palette.js (--pairs all) passes all checks on #ffffff and #1b1b1d.
# INK: contrast 3.6:1 on #ffffff, 4.8:1 on #1b1b1d. DOT on BLUE 3.9:1, on INK 4.0:1, ringed in INK.
BLUE, AQUA, ORANGE = "#3987e5", "#199e70", "#d95926"
INK = "#898781"
DOT = "#2b2a27"
GRID = (0.537, 0.529, 0.506, 0.28)  # INK at low alpha: a hairline on both surfaces
FONT = "system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
FS = 10  # annotation size
W = 6.0  # figure width [in]

plt.rcParams.update({
    "svg.fonttype": "none",
    "svg.hashsalt": "fugu-lfp-longevity",
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans"],
    "font.size": 11,
    "text.color": INK,
    "axes.labelcolor": INK,
    "axes.edgecolor": GRID,
    "axes.linewidth": 0.8,
    "axes.facecolor": "none",
    "figure.facecolor": "none",
    "savefig.facecolor": "none",
    "savefig.transparent": True,
    "xtick.color": INK,
    "ytick.color": INK,
    "xtick.major.size": 0,
    "ytick.major.size": 0,
    "xtick.minor.size": 0,
    "ytick.minor.size": 0,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.spines.left": False,
    "lines.linewidth": 2,
    "lines.solid_capstyle": "round",
    "lines.solid_joinstyle": "round",
    "legend.frameon": False,
    "legend.fontsize": 10,
    "path.simplify": True,
})


def save(fig, name, out):
    os.makedirs(out, exist_ok=True)
    p = os.path.join(out, name + ".svg")
    fig.savefig(p, format="svg", bbox_inches="tight", pad_inches=0.05, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    s = open(p).read()
    s = re.sub(r"font-family:\s*'?DejaVu Sans'?(, sans-serif)?", "font-family: " + FONT.replace("'", ""), s)
    s = re.sub(r"<!-- Created with matplotlib.*?-->\n", "", s)
    s = re.sub(r"<metadata>.*?</metadata>\n?", "", s, flags=re.S)
    open(p, "w").write(s)
    print(f"{name}.svg  {len(s.encode()) / 1024:.1f} kB")


def bar(ax, y, lo, hi, color, h):
    ax.barh(y, hi - lo, left=lo, height=h, color=color, lw=0, zorder=2)


def soc_axis(ax):
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 20), [f"{v}" for v in range(0, 101, 20)])
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("State of charge [%]")


# ----------------------------------------------------------------------------------------------------------------
# 1. Zsoldos et al. [2]: five cycling windows in rank order, average SoC marked; equal-depth extremes emphasised

def fig_windows(out):
    n = len(WINDOWS)
    fig, ax = plt.subplots(figsize=(W, 3.2))
    ys = list(range(n))[::-1]
    for k, ((lo, hi, avg), y) in enumerate(zip(WINDOWS, ys)):
        assert abs((lo + hi) / 2 - avg) < 1e-9
        edge = k in (0, n - 1)
        bar(ax, y, lo, hi, BLUE if edge else INK, 0.3 if edge else 0.2)
        ax.plot(avg, y, "o", ms=9, mfc=DOT, mec=INK, mew=1.2, zorder=5)
        txt = f"average SoC {avg:g} %" if k == 0 else f"{avg:g} %"
        ax.annotate(txt, (avg, y), xytext=(-6 if k == 0 else 0, 9), textcoords="offset points",
                    ha="left" if k == 0 else "center", va="bottom", fontsize=FS)
        if edge:
            first = k == 0
            ax.text(hi + 1.5 if first else lo - 1.5, y, f"depth {hi - lo} %", va="center",
                    ha="left" if first else "right", fontsize=FS)
    ax.set_yticks(ys, [f"{'best  ' if k == 0 else 'worst  ' if k == n - 1 else ''}{lo}–{hi} %"
                       for k, (lo, hi, _) in enumerate(WINDOWS)])
    ax.set_ylim(-0.55, n - 0.3)
    soc_axis(ax)
    save(fig, "soc-windows", out)


# ----------------------------------------------------------------------------------------------------------------
# 2. SoC strip: evidence ranges vs charger settings, lane names above the bars

def fig_strip(out):
    hol = (100 * (HOLIDAY_CHARGE - HOLIDAY_BAND), 100 * HOLIDAY_CHARGE)
    par = (100 * (PARTIAL_EXAMPLE - RECHARGE_DOD), 100 * PARTIAL_EXAMPLE)
    groups = [
        ("evidence", BLUE, [("recommended range [8]", MFR_RANGE),
                            (f"storage fade hardly differs at {FLAT_TEMP} °C [3]", FLAT_RANGE)]),
        ("charger settings", ORANGE, [(f"holiday_charge {HOLIDAY_CHARGE:g} (example), holiday_band {HOLIDAY_BAND:g}", hol),
                                     (f"partial_charge {PARTIAL_EXAMPLE:g} (example)", par)]),
    ]
    fig, ax = plt.subplots(figsize=(W, 3.9))
    tr = ax.get_yaxis_transform()
    y = 0.0
    y_top = None
    for gname, col, lanes in groups:
        y -= 0.9
        ax.text(0, y, gname.upper(), transform=tr, fontsize=FS - 1, fontweight="bold", va="bottom")
        y_top = y if y_top is None else y_top
        for label, (lo, hi) in lanes:
            y -= 0.7
            ax.text(0, y, label, transform=tr, fontsize=FS, va="bottom")
            y -= 0.36
            bar(ax, y, lo, hi, col, 0.4)
            ax.text(lo, y - 0.3, f"{lo:g}", ha="center", va="top", fontsize=FS - 1)
            ax.text(hi, y - 0.3, f"{hi:g}", ha="center", va="top", fontsize=FS - 1)
            y -= 0.6
        y -= 0.3
    ax.text(par[1] + 1.5, y + 0.9, "stop", va="center", ha="left", fontsize=FS)
    ax.text(par[0] - 1.5, y + 0.9, f"resume after recharge_dod {RECHARGE_DOD:.2f}", va="center", ha="right", fontsize=FS)
    ax.axvline(STEP_SOC, color=INK, lw=1.4, ls=(0, (4, 3)), zorder=3)
    ax.text(STEP_SOC + 1.2, y_top, f"{STEP_SOC} % calendar step [1]", va="bottom", ha="left", fontsize=FS)
    ax.set_yticks([])
    ax.set_ylim(y + 0.3, -0.5)
    soc_axis(ax)
    save(fig, "soc-strip", out)


# ----------------------------------------------------------------------------------------------------------------
# 3. Keil et al. [1] Fig. 2c: capacity fade vs storage SoC for the LFP cell

def fig_keil(out):
    fig, ax = plt.subplots(figsize=(W, 3.6))
    for (t, pts), col, mk in zip(KEIL_2C.items(), (BLUE, AQUA, ORANGE), "os^"):
        x = [p[0] for p in pts]
        f = [(1 - p[1]) * 100 for p in pts]
        ax.plot(x, f, color=col, lw=1.8, marker=mk, ms=5.5, mew=0, zorder=4, clip_on=False)
        ax.text(103.5, f[-1], f"{t} °C", color=INK, va="center", fontsize=FS, clip_on=False)
    ax.axvline(STEP_SOC, color=INK, lw=1.2, ls=(0, (4, 3)), zorder=2)
    ax.text(STEP_SOC - 1.2, 15.6, f"central graphite peak\n(new cell), {STEP_SOC} %", ha="right", va="top", fontsize=FS,
            linespacing=1.15)
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 20), [f"{v}" for v in range(0, 101, 20)])
    ax.set_ylim(0, 16)
    ax.grid(axis="x", visible=False)
    ax.set_xlabel("Storage state of charge [%]")
    ax.set_ylabel("Capacity fade [%]")
    save(fig, "keil-fade-vs-soc", out)


def render(out):
    fig_keil(out)
    fig_windows(out)
    fig_strip(out)


if __name__ == "__main__":
    if "--check" in sys.argv:
        with tempfile.TemporaryDirectory() as tmp:
            render(tmp)
            bad = [f for f in sorted(os.listdir(tmp))
                   if not os.path.exists(os.path.join(OUT, f))
                   or open(os.path.join(tmp, f), "rb").read() != open(os.path.join(OUT, f), "rb").read()]
        if bad:
            sys.exit("out of date: " + ", ".join(bad))
        print("up to date")
    else:
        render(OUT)
