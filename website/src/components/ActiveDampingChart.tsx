import React, {useMemo, useRef, useState} from 'react';
import styles from './ActiveDampingChart.module.css';

// DCM ring after LS-off (ideal L–C_sw tank, series loss for Q), with and without one LS pulse.
const VIN = 60, VOUT = 27, VF = 0.7, L = 83e-6, C = 3.0e-9, Q = 10, PW = 200e-9;
const W0 = 1 / Math.sqrt(L * C), Z0 = Math.sqrt(L / C), R = Z0 / Q, TR = 2 * Math.PI / W0;
const T_END = 2.6 * TR, DT = 1e-9;

type Case = {key: string; label: string; t0: number | null; cls: string};
const CASES: Case[] = [
  {key: 'free', label: 'free ring', t0: null, cls: 'free'},
  {key: 'q', label: 'LS pulse at T_ring/4', t0: TR / 4, cls: 'quarter'},
  {key: 'val', label: 'LS pulse at valley (T_ring)', t0: TR, cls: 'valley'},
];

type Trace = {v: Float32Array; i: Float32Array; ampBefore: number; ampAfter: number};

function simulate(t0: number | null): Trace {
  const n = Math.ceil(T_END / DT) + 1, v = new Float32Array(n), i = new Float32Array(n);
  let vs = 0, is = 0, ampBefore = VOUT, ampAfter = VOUT;
  for (let k = 0; k < n; k++) {
    const t = k * DT, on = t0 !== null && t >= t0 && t < t0 + PW;
    if (t0 !== null && Math.abs(t - t0) < DT / 2) ampBefore = Math.hypot(vs - VOUT, Z0 * is);
    if (on) vs = 0;
    v[k] = vs; i[k] = is;
    // i > 0 flows from the switch node into the coil
    is += ((vs - VOUT - R * is) / L) * DT;
    if (!on) vs = Math.min(VIN + VF, Math.max(-VF, vs + (-is / C) * DT)); // body-diode clamps
    if (t0 !== null && Math.abs(t - (t0 + PW)) < DT / 2) ampAfter = Math.hypot(vs - VOUT, Z0 * is);
  }
  return {v, i, ampBefore, ampAfter};
}

const VW = 720, PL = 52, PR = 16, PANEL_H = 150, GAP = 34, TOP = 28;
const V_TOP = TOP, I_TOP = TOP + PANEL_H + GAP, AXIS_Y = I_TOP + PANEL_H, VH = AXIS_Y + 44;
const V_MIN = -5, V_MAX = 70, I_MIN = -0.5, I_MAX = 0.5;
const x = (t: number) => PL + (t / T_END) * (VW - PL - PR);
const yV = (v: number) => V_TOP + PANEL_H * (1 - (v - V_MIN) / (V_MAX - V_MIN));
const yI = (i: number) => I_TOP + PANEL_H * (1 - (Math.max(I_MIN, Math.min(I_MAX, i)) - I_MIN) / (I_MAX - I_MIN));

export default function ActiveDampingChart(): React.JSX.Element {
  const svgRef = useRef<SVGSVGElement>(null);
  const [hoverT, setHoverT] = useState<number | null>(null);

  const traces = useMemo(() => CASES.map(c => simulate(c.t0)), []);
  const paths = useMemo(() => traces.map(tr => {
    const step = 4, pv: string[] = [], pi: string[] = [];
    for (let k = 0; k < tr.v.length; k += step) {
      const t = k * DT, c = k ? 'L' : 'M';
      pv.push(`${c}${x(t).toFixed(1)},${yV(tr.v[k]).toFixed(1)}`);
      pi.push(`${c}${x(t).toFixed(1)},${yI(tr.i[k]).toFixed(1)}`);
    }
    return {v: pv.join(''), i: pi.join('')};
  }), [traces]);

  const onMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const r = svgRef.current!.getBoundingClientRect();
    const t = ((((e.clientX - r.left) / r.width) * VW - PL) / (VW - PL - PR)) * T_END;
    setHoverT(t >= 0 && t <= T_END ? t : null);
  };
  const hk = hoverT !== null ? Math.round(hoverT / DT) : null;

  const vTicks = [0, 27, 54], iTicks = [-0.5, -0.25, 0, 0.25, 0.5];
  const tTicks = [0, 2, 4, 6, 8];

  return (
    <figure className={styles.root}>
      <div className={styles.scroll}>
      <svg ref={svgRef} viewBox={`0 0 ${VW} ${VH}`} className={styles.svg} role="img"
           aria-label="DCM ring after the low side turns off: free ring compared with an LS pulse at a quarter ring period and at the ring valley"
           onPointerMove={onMove} onPointerLeave={() => setHoverT(null)}>
        {CASES.filter(c => c.t0 !== null).map(c => (
          <g key={c.key}>
            <rect x={x(c.t0!)} y={V_TOP} width={Math.max(2, x(c.t0! + PW) - x(c.t0!))} height={AXIS_Y - V_TOP}
                  className={styles[`band_${c.cls}`]}/>
            <text x={x(c.t0!) + 4} y={V_TOP - 10} className={styles.phase}>LS on</text>
          </g>
        ))}
        {vTicks.map(v => (
          <g key={`v${v}`}>
            <line x1={PL} x2={VW - PR} y1={yV(v)} y2={yV(v)} className={v === VOUT ? styles.ref : styles.grid}/>
            <text x={PL - 6} y={yV(v)} className={styles.tick} textAnchor="end" dominantBaseline="middle">{v}</text>
          </g>
        ))}
        {iTicks.map(i => (
          <g key={`i${i}`}>
            <line x1={PL} x2={VW - PR} y1={yI(i)} y2={yI(i)} className={i === 0 ? styles.zero : styles.grid}/>
            <text x={PL - 6} y={yI(i)} className={styles.tick} textAnchor="end" dominantBaseline="middle">{i}</text>
          </g>
        ))}
        {tTicks.map(t => (
          <text key={`t${t}`} x={x(t * 1e-6)} y={AXIS_Y + 16} className={styles.tick} textAnchor="middle">{t}</text>
        ))}
        <text x={(PL + VW - PR) / 2} y={AXIS_Y + 36} className={styles.axisLabel} textAnchor="middle">time after LS turn-off (µs)</text>
        <text x={PL} y={V_TOP + 12} dx={6} className={styles.panelTitle}>switch node v_sw (V)</text>
        <text x={PL} y={I_TOP + 12} dx={6} className={styles.panelTitle}>inductor current i_L (A)</text>
        <text x={PL + 6} y={yV(VOUT) - 4} className={styles.refLabel}>V_out</text>

        {paths.map((p, k) => (
          <g key={CASES[k].key}>
            <path d={p.v} className={`${styles.line} ${styles[CASES[k].cls]}`}/>
            <path d={p.i} className={`${styles.line} ${styles[CASES[k].cls]}`}/>
          </g>
        ))}

        {hk !== null && (
          <g pointerEvents="none">
            <line x1={x(hoverT!)} x2={x(hoverT!)} y1={V_TOP} y2={AXIS_Y} className={styles.cross}/>
            {traces.map((tr, k) => (
              <circle key={k} cx={x(hoverT!)} cy={yV(tr.v[hk])} r={3.5} className={`${styles.dot} ${styles[`dot_${CASES[k].cls}`]}`}/>
            ))}
          </g>
        )}
      </svg>
      </div>
      <div className={styles.legend}>
        {CASES.map(c => <span key={c.key}><span className={`${styles.key} ${styles[`key_${c.cls}`]}`}/>{c.label}</span>)}
      </div>
      <div className={styles.readout} aria-live="polite">
        {hk !== null
          ? <>t = {(hoverT! * 1e6).toFixed(2)} µs · {CASES.map((c, k) =>
              <span key={c.key}>{k ? ' · ' : ''}<span className={`${styles.key} ${styles[`key_${c.cls}`]}`}/>{traces[k].v[hk].toFixed(1)} V</span>)}</>
          : null}
      </div>
      <figcaption className={styles.caption}>
        Illustrative, computed from the ideal L–C_sw tank with body-diode clamps at −{VF} V and V_in + {VF} V, not a measurement: V_in = {VIN} V, V_out = {VOUT} V, L = {L * 1e6} µH,
        C_sw = {C * 1e9} nF (f_r = {(1 / TR / 1e3).toFixed(0)} kHz, T_ring = {(TR * 1e6).toFixed(2)} µs,
        √(L/C_sw) = {Z0.toFixed(0)} Ω), damping Q ≈ {Q} assumed, LS pulse {PW * 1e9} ns. While the LS FET is on, the
        node is held at 0 V and the coil current ramps by −V_out/L. The free ring starts at 0 V with zero current;
        the next HS edge is not shown.
      </figcaption>
      <details className={styles.table}>
        <summary>Ring amplitude around V_out before and after the pulse (table)</summary>
        <table>
          <thead><tr><th>case</th><th>pulse at (µs)</th><th>amplitude before (V)</th><th>amplitude after (V)</th></tr></thead>
          <tbody>
            <tr><td>free ring</td><td>none</td><td>{VOUT} at LS turn-off</td><td>decays</td></tr>
            {CASES.slice(1).map((c, k) => (
              <tr key={c.key}><td>{c.label}</td><td>{(c.t0! * 1e6).toFixed(2)}</td><td>{traces[k + 1].ampBefore.toFixed(1)}</td><td>{traces[k + 1].ampAfter.toFixed(1)}</td></tr>
            ))}
          </tbody>
        </table>
      </details>
    </figure>
  );
}
