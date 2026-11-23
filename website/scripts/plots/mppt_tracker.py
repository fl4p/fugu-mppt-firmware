#!/usr/bin/env python3
"""Figures for website/docs/internals/mppt-tracker.md.

Run from anywhere: .venv/bin/python3 website/scripts/plots/mppt_tracker.py [--check]
Writes SVGs to website/docs/internals/img/mppt-tracker/. Constants mirror src/tracker.h, src/peak_scan.h and
src/mppt.h (settleMs). The PV curve and the input-capacitor timeline are models.
"""
import os
import re
import sys

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../docs/internals/img/mppt-tracker")

# colours validated (dataviz validate_palette.js) against #ffffff (light) and #1b1b1d (dark): all checks pass
BLUE, ORANGE, AQUA = "#3987e5", "#d95926", "#199e70"
INK = "#898781"  # muted ink, >= 3.5:1 on both surfaces
GRID = (0.537, 0.529, 0.506, 0.28)  # INK at low alpha: a hairline on both surfaces
FONT = "system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"

plt.rcParams.update({
    "svg.fonttype": "none",
    "svg.hashsalt": "fugu-mppt-tracker",
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans"],
    "font.size": 10,
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
    "lines.linewidth": 2,
    "lines.solid_capstyle": "round",
    "lines.solid_joinstyle": "round",
    "legend.frameon": False,
    "legend.fontsize": 9,
    "path.simplify": True,
})


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name + ".svg")
    fig.savefig(p, format="svg", bbox_inches="tight", pad_inches=0.05, metadata={"Date": None, "Creator": None})
    plt.close(fig)
    s = open(p).read()
    s = re.sub(r"font-family:\s*'?DejaVu Sans'?(, sans-serif)?", "font-family: " + FONT.replace("'", ""), s)
    s = re.sub(r"<!-- Created with matplotlib.*?-->\n", "", s)
    s = re.sub(r"<metadata>.*?</metadata>\n?", "", s, flags=re.S)
    open(p, "w").write(s)
    print(f"{name}.svg  {len(s.encode()) / 1024:.1f} kB")


def dot(ax, x, y, color, filled=True, z=5, size=7):
    ax.plot(x, y, "o", ms=size, mfc=color if filled else "none", mec=color, mew=1.6 if not filled else 0, zorder=z)


# ----------------------------------------------------------------------------------------------------------------
# 1. Partially shaded string: single-diode cells, 20-cell substrings with bypass diodes (model)

CELL_ISC, CELL_VOC, CELL_N, VT, CELL_RS = 9.0, 0.62, 1.2, 0.0257, 0.006
CELLS_SUB, N_SUB, VF_BYPASS = 20, 6, 0.4
I0 = CELL_ISC / (np.exp(CELL_VOC / (CELL_N * VT)) - 1)


def sub_v(i, g):
    """Voltage of one substring at string current i [A], irradiance fraction g; bypass diode clamps at -Vf."""
    iph = CELL_ISC * g
    with np.errstate(invalid="ignore", divide="ignore"):
        v = CELLS_SUB * (CELL_N * VT * np.log((iph - i) / I0 + 1) - i * CELL_RS)
    return np.where(i < iph, np.maximum(v, -VF_BYPASS), -VF_BYPASS)


def string_pv(gs, n=6000):
    # dense near each photocurrent, where the cell voltage collapses
    i = np.concatenate([np.linspace(0, CELL_ISC * max(gs), n)] +
                       [CELL_ISC * g * (1 - np.logspace(-9, 0, n)) for g in set(gs)])
    i = np.unique(i)
    v = sum(sub_v(i, g) for g in gs)
    ok = v > 0
    v, i = v[ok][::-1], i[ok][::-1]  # ascending V
    return v, v * i


def sweep_maxima(v, p):
    """PeakScan::sweepSample over a falling-Vin sweep: maxima below 0.93 Voc with an 8 % dip on both sides."""
    voc = v[-1]
    fall, plo, pk, vpk, out = True, np.inf, 0.0, 0.0, []
    for vv, pp in zip(v[::-1], p[::-1]):
        if vv <= 1:
            continue
        if fall:
            if pp < plo:
                plo = pp
            elif pp > 1.08 * plo:
                fall, pk, vpk = False, pp, vv
        elif pp > pk:
            pk, vpk = pp, vv
        elif pp < 0.92 * pk:
            fall, plo = True, pp
            if vpk < 0.93 * voc:
                out.append((vpk, pk))
    return out


def learn_n(maxima, voc, p_mpp):
    """PeakScan::learn(): largest n in 2..6 putting every lower maximum at m - 0.25 .. m + 0.5 shares of 0.82 Voc/n."""
    g = next(k for k, (_, pp) in enumerate(maxima) if pp >= 0.6 * p_mpp)
    lower = [vv for vv, pp in maxima[g + 1:] if pp >= 0.3 * p_mpp]
    best = None
    for c in range(2, 7):
        ok = True
        for vv in lower:
            y = vv / (0.82 * voc) * c
            m = int(y + 0.25)
            ok = ok and 1 <= m < c and y <= m + 0.5
        if ok and lower:
            best = c
    return best


def fig_pv():
    gs_sun, gs_shade = [1.0] * N_SUB, [1.0] * 4 + [0.4] * 2
    v0, p0 = string_pv(gs_sun)
    v1, p1 = string_pv(gs_shade)
    voc = v1[-1]
    kvmp = v0[np.argmax(p0)] / v0[-1]
    maxima = sweep_maxima(v1, p1)
    p_mpp = p1.max()
    n = learn_n(maxima, voc, p_mpp)
    print(f"  model: Voc {voc:.1f} V, unshaded MPP {p0.max():.0f} W at {v0[np.argmax(p0)]:.1f} V "
          f"(Vmp/Voc {kvmp:.3f}), shaded maxima {[(round(a, 1), round(b)) for a, b in maxima]}, learned n={n}")
    assert n == 6 and 0.74 < kvmp < 0.86

    share = 0.82 * voc / N_SUB
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(7.6, 6.6), sharex=True, gridspec_kw={"hspace": 0.32})

    # (a) curves, cut, share grid, recorded maxima
    for m in range(1, N_SUB + 1):
        for a in (ax, bx):
            a.axvline(m * share, color=GRID, lw=0.8, zorder=0)
        ax.text(m * share, 1.0, f"{m}", transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=9)
    ax.plot(v0, p0, color=BLUE, label="unshaded")
    ax.plot(v1, p1, color=ORANGE, label="2 of 6 substrings at 40 %")
    ax.axvspan(0.93 * voc, voc * 1.03, color=INK, lw=0, alpha=0.14, zorder=0)
    ax.text(0.93 * voc - 0.6, p0.max() * 1.2, "0.93·Voc", fontsize=8.5, va="top", ha="right")
    ax.annotate("Voc", (voc, 0), xytext=(-34, 22), textcoords="offset points", fontsize=9, ha="right",
                va="center", arrowprops=dict(arrowstyle="-", color=INK, lw=0.8, shrinkA=2, shrinkB=4))
    ax.plot([voc], [0], "o", ms=5, color=INK, clip_on=False, zorder=6)
    for k, (vv, pp) in enumerate(maxima):
        dot(ax, vv, pp, ORANGE)
        is_mpp = pp == max(q for _, q in maxima)
        lbl = "MPP" if is_mpp else "local maximum"
        ax.annotate(f"{lbl}\n{vv:.0f} V, {pp:.0f} W", (vv, pp), xytext=(0, -10) if is_mpp else (-12, 6),
                    textcoords="offset points", ha="center" if is_mpp else "right", va="top" if is_mpp else "bottom", fontsize=8.5)
    ax.set_ylabel("Power [W]")
    ax.set_ylim(0, p0.max() * 1.25)
    ax.legend(loc="upper left", ncols=2, bbox_to_anchor=(0, 0.99))
    ax.set_title("(a) Sweep: recorded maxima, grid of m·0.82·Voc/6", loc="left", fontsize=10, color=INK, pad=16)

    # (b) partial scan from the higher-voltage local maximum (m0 = 6)
    bx.plot(v1, p1, color=ORANGE)
    v_op, p_op = maxima[0]
    i_env = np.max(p1[v1 > 1] / v1[v1 > 1])
    m0 = int(np.clip(round(v_op / share), 1, N_SUB))
    sh = 0.5 * (v_op / m0 + share)
    best = None
    dot(bx, v_op, p_op, INK)
    bx.annotate("start P₀", (v_op, p_op), xytext=(6, 6), textcoords="offset points", fontsize=8.5)
    for m in range(1, N_SUB + 1):
        vc = 0.97 * m * sh
        if m == m0:
            continue
        if vc > 0.95 * voc:
            continue
        if m < m0 and m * sh * i_env < p_op * 1.03:
            bx.plot([vc], [0], "o", ms=7, mfc="none", mec=INK, mew=1.4, clip_on=False, zorder=6)
            bx.annotate("skipped", (vc, 0), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=8)
            continue
        pc = float(np.interp(vc, v1, p1))
        dot(bx, vc, pc, AQUA)
        bx.annotate(f"m={m}\n{pc:.0f} W", (vc, pc), xytext=(0, -10), textcoords="offset points", ha="center",
                    va="top", fontsize=8.5)
        if best is None or pc > best[1]:
            best = (vc, pc)
    assert best[1] > 1.03 * p_op
    bx.annotate("", xy=best, xytext=(v_op, p_op), arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.2,
                                                                  shrinkA=6, shrinkB=6,
                                                                  connectionstyle="arc3,rad=0.3"))
    bx.text(0.5 * (best[0] + v_op), max(best[1], p_op) * 1.3, "move: best probe > 1.03·P₀", ha="center", fontsize=8.5)
    bx.set_ylim(0, p0.max() * 1.25)
    bx.set_ylabel("Power [W]")
    bx.set_xlabel("Vin [V]")
    bx.set_xlim(0, voc * 1.03)
    bx.set_title("(b) Partial scan: probes at 0.97·m·s, lower ones bounded by m·s·Imax", loc="left", fontsize=10,
                 color=INK)
    save(fig, "pv-shaded")


# ----------------------------------------------------------------------------------------------------------------
# 2. Effective relative threshold vs power (src/tracker.h setSlowMode / update)

MODES = {"fast": (1.0, 0.015, BLUE), "slow": (0.5, 0.0015, ORANGE)}


def fig_theta():
    P = np.logspace(0, 3, 400)
    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    for name, (dpa, rel, col) in MODES.items():
        th = np.minimum(dpa, rel * (P + 5)) / P * 100
        ax.plot(P, th, color=col, label=f"{name}: {dpa:g} W, {rel * 100:g} % of (P + 5 W)")
        px = dpa / rel - 5
        thx = dpa / px * 100
        dot(ax, px, thx, col)
        ax.annotate(f"{px:.0f} W, {thx:.2g} %", (px, thx), xytext=(-7, -6), textcoords="offset points", fontsize=8.5,
                    ha="right", va="top")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(1, 1000)
    ax.set_ylim(0.04, 120)
    ax.set_xticks([1, 2, 5, 10, 20, 50, 100, 200, 500, 1000], ["1", "2", "5", "10", "20", "50", "100", "200", "500",
                                                              "1000"])
    ax.set_yticks([0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100], ["0.1", "0.2", "0.5", "1", "2", "5", "10", "20", "50",
                                                             "100"])
    ax.minorticks_off()
    ax.set_xlabel("Power P [W]")
    ax.set_ylabel("Effective threshold [% of P]")
    ax.legend(loc="upper right")
    ax.text(1.15, 0.13, "left of the dot: relative term, right: absolute term", fontsize=8.5)
    save(fig, "theta-eff")


# ----------------------------------------------------------------------------------------------------------------
# 3. P&O band near the MPP

C_CURV = 9


def fig_band():
    fig, axs = plt.subplots(1, 2, figsize=(7.6, 3.4), gridspec_kw={"wspace": 0.28})
    for ax, (name, (_, rel, col)) in zip(axs, MODES.items()):
        xm = np.sqrt(2 * rel / C_CURV) * 1.35
        x = np.linspace(-xm, xm, 300)
        ax.plot(x * 100, (1 - C_CURV * x ** 2) * 100, color=INK)
        for k, a in ((1, 0.22), (2, 0.10)):
            xb = np.sqrt(k * rel / C_CURV) * 100
            ax.axvspan(-xb, xb, color=col, alpha=a, lw=0, zorder=0)
            ax.axhline((1 - k * rel) * 100, color=col, lw=1.2)
            ax.text(xm * 100, (1 - k * rel) * 100, f"1−{k if k > 1 else ''}θ ", ha="right", va="bottom", fontsize=8.5)
            ax.annotate(f"±{xb:.2g} %", (xb, (1 - k * rel) * 100), xytext=(-5, -4), textcoords="offset points",
                        ha="right", va="top", fontsize=8)
        ax.set_ylim((1 - 2.9 * rel) * 100, 100 + 0.15 * rel * 100)
        ax.set_xlim(-xm * 100, xm * 100)
        ax.set_title(f"{name}: θ = {rel * 100:g} %", loc="left", fontsize=10, color=INK)
        ax.set_xlabel("Vin offset x [% of Vmp]")
    axs[0].set_ylabel("P / Pm [%]")
    save(fig, "po-band")


# ----------------------------------------------------------------------------------------------------------------
# 4. Variance factor g(u) of a period mean over exponentially correlated noise

def settle_ms(span, fs):
    return (3320.0 * span + 2000.0) / fs  # src/mppt.h settleMs()


def g_of(u):
    return 2 / u * (1 - (1 - np.exp(-u)) / u)


def fig_g():
    tau = settle_ms(20, 450) / 6.64
    u = np.logspace(-1.5, 2.3, 300)
    fig, ax = plt.subplots(figsize=(7.6, 3.6))
    ax.plot(u, 2 / u, color=INK, lw=1, alpha=0.7)
    ax.text(12, 2 / 12 * 1.3, "2τ / Td", fontsize=8.5, ha="left", va="bottom")
    ax.plot(u, g_of(u), color=BLUE)
    for name, td in (("fast", 33.0), ("slow", 500.0)):
        uu = td / tau
        gg = g_of(uu)
        dot(ax, uu, gg, BLUE)
        ax.annotate(f"{name}, Td = {td:g} ms\nTd/τ = {uu:.1f}, g = {gg:.2g}", (uu, gg), xytext=(-8, -8),
                    textcoords="offset points", ha="right", va="top", fontsize=8.5)
        print(f"  g: {name} Td/tau={uu:.2f} g={gg:.3f} sqrt={np.sqrt(gg):.2f}")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(u[0], u[-1])
    ax.set_ylim(0.01, 1.5)
    ax.set_xticks([0.1, 0.3, 1, 3, 10, 30, 100], ["0.1", "0.3", "1", "3", "10", "30", "100"])
    ax.set_yticks([0.01, 0.03, 0.1, 0.3, 1], ["0.01", "0.03", "0.1", "0.3", "1"])
    ax.minorticks_off()
    ax.set_xlabel("u = Td / τ")
    ax.set_ylabel("g(u) = σ² / σf²")
    ax.set_title(f"τ = {tau:.0f} ms: INA226 profile, 450 Hz, span 20", loc="left", fontsize=10, color=INK)
    save(fig, "variance-g")


# ----------------------------------------------------------------------------------------------------------------
# 5. Settle window after a reversal (model: P(x) = Pm(1 - c x^2) source, DCM converter as a conductance ~ D^2)

def ewma2(x, span):
    a = 2 / (span + 1)
    y = np.empty_like(x)
    y1 = y2 = x[0]
    for k, v in enumerate(x):
        y1 = (1 - a) * y1 + a * v
        y2 = (1 - a) * y2 + a * y1
        y[k] = y2
    return y


def median5(x):
    xp = np.concatenate([np.full(4, x[0]), x])
    return np.array([np.median(xp[k:k + 5]) for k in range(len(x))])


def fig_settle():
    cin, vm, pm, fs, span = 470e-6, 60.0, 2.0, 450.0, 20
    slew = 0.12  # d ln(G)/dt [1/s] of the converter conductance: illustrative
    gm = pm / vm ** 2
    tau = cin * vm ** 2 / (2 * pm)
    t_f = settle_ms(span, fs) / 1000

    def i_pv(v):
        x = (v - vm) / vm
        return np.maximum(pm * (1 - C_CURV * x * x), 0) / v

    sub = 20
    dt = 1 / (fs * sub)
    t0, t1 = -4.0, 1.2
    t = np.arange(t0, t1, dt)
    lng = np.where(t < 0, slew * t, -slew * t)  # duty rising until t = 0, falling after
    lng0 = 0.04  # equilibrium at t = 0 a little past the MPP (below Vm)
    g = gm * np.exp(lng + lng0)
    v = np.empty_like(t)
    vv = vm
    for k in range(len(t)):
        vv += dt * (i_pv(vv) - g[k] * vv) / cin
        v[k] = vv
    p_pv = i_pv(v) * v
    p_conv = g * v * v
    ts, vs, ps, pcs = t[::sub], v[::sub], p_pv[::sub], p_conv[::sub]
    pf = ewma2(median5(pcs), span)
    k0 = np.searchsorted(ts, 0)
    v_rev = vs[k0]
    kv = k0 + np.argmax(vs[k0 + 1:] >= v_rev) + 1
    t_v = ts[kv]
    t_end = min(max(t_f, t_v), 1.0)
    print(f"  settle: tau={tau * 1000:.0f} ms, t_f={t_f * 1000:.0f} ms, Vin back at reversal after {t_v * 1000:.0f} ms,"
          f" Vin minimum at {ts[k0 + np.argmin(vs[k0:])] * 1000:.0f} ms, window {t_end * 1000:.0f} ms")

    m = ts >= -0.4
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(7.6, 4.8), sharex=True, gridspec_kw={"hspace": 0.12,
                                                                                   "height_ratios": [1, 1.2]})
    for a in (ax, bx):
        a.axvspan(0, t_end, color=INK, alpha=0.14, lw=0, zorder=0)
        a.axvline(0, color=INK, lw=1)
    ax.plot(ts[m], vs[m], color=BLUE)
    ax.axhline(v_rev, color=INK, lw=0.8, alpha=0.7)
    dot(ax, t_v, v_rev, BLUE)
    ax.annotate(f"back at the reversal voltage, {t_v * 1000:.0f} ms", (t_v, v_rev), xytext=(10, -6),
                textcoords="offset points", fontsize=8.5, va="top")
    ax.text(-0.38, v_rev, "duty rising", fontsize=8.5, va="bottom")
    ax.text(0.03, vs[m].max(), "reversal: duty falling", fontsize=8.5, va="top")
    ax.set_ylabel("Vin [V]")
    bx.plot(ts[m], ps[m], color=BLUE, label="string power")
    bx.plot(ts[m], pf[m], color=ORANGE, label="measured, filtered")
    bx.axvline(t_f, color=ORANGE, lw=1)
    bx.text(t_f, 0.02, f" filter settled {t_f * 1000:.0f} ms", transform=bx.get_xaxis_transform(), fontsize=8.5,
            va="bottom")
    bx.text((t_f + t_end) / 2, 0.97, f"settle window {t_end * 1000:.0f} ms", transform=bx.get_xaxis_transform(),
            ha="center", va="top", fontsize=8.5)
    bx.set_ylabel("Power [W]")
    bx.set_xlabel("Time after the reversal [s]")
    bx.set_xlim(-0.4, t1)
    lo, hi = min(pf[m].min(), ps[m].min()), max(pf[m].max(), ps[m].max())
    bx.set_ylim(lo - 0.45 * (hi - lo), hi + 0.08 * (hi - lo))
    bx.legend(loc="lower right")
    save(fig, "settle-window")


if __name__ == "__main__":
    fig_pv()
    fig_theta()
    fig_band()
    fig_g()
    fig_settle()
