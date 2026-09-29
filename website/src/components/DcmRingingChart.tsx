import React, {useMemo, useRef, useState} from 'react';
import styles from './DcmRingingChart.module.css';

// Ideal DCM buck period, then the L–C_sw ring after the LS turns off at i_L = 0.
const VIN = 60, VOUT = 27, L = 83e-6, C = 3.0e-9, FSW = 39e3, IPK = 1.5, Q = 10;
const W0 = 1 / Math.sqrt(L * C), Z0 = Math.sqrt(L / C), ALPHA = W0 / (2 * Q);
const T = 1 / FSW, T_ON = L * IPK / (VIN - VOUT), T_LS = T_ON + L * IPK / VOUT, T_END = T + 2.5e-6;

type Phase = 'HS on' | 'LS on' | 'both off (ring)';

function sample(t: number): {v: number; i: number; phase: Phase} {
  if (t < T_ON) return {v: VIN, i: IPK * t / T_ON, phase: 'HS on'};
  if (t < T_LS) return {v: 0, i: IPK * (1 - (t - T_ON) / (T_LS - T_ON)), phase: 'LS on'};
  if (t < T) {
    const tr = t - T_LS, e = Math.exp(-ALPHA * tr);
    return {v: VOUT * (1 - e * Math.cos(W0 * tr)), i: -(VOUT / Z0) * e * Math.sin(W0 * tr), phase: 'both off (ring)'};
  }
  const t2 = t - T;
  return {v: VIN, i: Math.max(0, IPK * t2 / T_ON), phase: 'HS on'};
}

const VW = 720, PL = 52, PR = 16, PANEL_H = 150, GAP = 34, TOP = 28;
const V_TOP = TOP, I_TOP = TOP + PANEL_H + GAP, AXIS_Y = I_TOP + PANEL_H, VH = AXIS_Y + 44;
const V_MAX = 70, I_MIN = -0.5, I_MAX = 2;
const us = (t: number) => t * 1e6;
const x = (t: number) => PL + (t / T_END) * (VW - PL - PR);
const yV = (v: number) => V_TOP + PANEL_H * (1 - v / V_MAX);
const yI = (i: number) => I_TOP + PANEL_H * (1 - (i - I_MIN) / (I_MAX - I_MIN));

const PHASES: [number, number, Phase][] = [[0, T_ON, 'HS on'], [T_ON, T_LS, 'LS on'], [T_LS, T, 'both off (ring)'], [T, T_END, 'HS on']];

export default function DcmRingingChart(): React.JSX.Element {
  const svgRef = useRef<SVGSVGElement>(null);
  const [hoverT, setHoverT] = useState<number | null>(null);

  const {vPath, iPath} = useMemo(() => {
    const n = 1400, pts = Array.from({length: n + 1}, (_, k) => (k / n) * T_END);
    const edges = [T_ON, T_LS, T]; // draw the switching edges vertically
    const all = [...pts, ...edges.flatMap(e => [e - 1e-12, e])].sort((a, b) => a - b);
    const path = (f: (t: number) => number) => all.map((t, k) => `${k ? 'L' : 'M'}${x(t).toFixed(1)},${f(t).toFixed(1)}`).join('');
    return {vPath: path(t => yV(sample(t).v)), iPath: path(t => yI(sample(t).i))};
  }, []);

  const onMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const r = svgRef.current!.getBoundingClientRect();
    const px = ((e.clientX - r.left) / r.width) * VW;
    const t = ((px - PL) / (VW - PL - PR)) * T_END;
    setHoverT(t >= 0 && t <= T_END ? t : null);
  };

  const hv = hoverT !== null ? sample(hoverT) : null;
  const vTicks = [0, 20, 40, 60], iTicks = [-0.5, 0, 0.5, 1, 1.5, 2], tTicks = [0, 5, 10, 15, 20, 25];

  return (
    <figure className={styles.root}>
      <div className={styles.scroll}>
      <svg ref={svgRef} viewBox={`0 0 ${VW} ${VH}`} className={styles.svg} role="img"
           aria-label="Switch-node voltage and inductor current over one DCM period, with the LC ring after the low side turns off"
           onPointerMove={onMove} onPointerLeave={() => setHoverT(null)}>
        {PHASES.map(([a, b, p], k) => (
          <g key={k}>
            {p === 'both off (ring)' && <rect x={x(a)} y={V_TOP} width={x(b) - x(a)} height={AXIS_Y - V_TOP} className={styles.band}/>}
            <text x={(x(a) + x(b)) / 2} y={V_TOP - 10} className={styles.phase} textAnchor="middle">{p}</text>
          </g>
        ))}

        {vTicks.map(v => (
          <g key={`v${v}`}>
            <line x1={PL} x2={VW - PR} y1={yV(v)} y2={yV(v)} className={styles.grid}/>
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
        <text x={(PL + VW - PR) / 2} y={AXIS_Y + 36} className={styles.axisLabel} textAnchor="middle">time (µs)</text>
        <text x={PL} y={V_TOP + 12} dx={6} className={styles.panelTitle}>switch node v_sw (V)</text>
        <text x={PL} y={I_TOP + 12} dx={6} className={styles.panelTitle}>inductor current i_L (A)</text>

        <line x1={PL} x2={VW - PR} y1={yV(VOUT)} y2={yV(VOUT)} className={styles.ref}/>
        <text x={x(T_LS) - 6} y={yV(VOUT) - 4} className={styles.refLabel} textAnchor="end">ring centre = V_out = {VOUT} V</text>
        <text x={x(T_LS) + 4} y={yI(IPK)} className={styles.refLabel}>LS off at i_L = 0</text>
        <line x1={x(T_LS)} x2={x(T_LS)} y1={yI(IPK) + 4} y2={yI(0)} className={styles.ref}/>

        <path d={vPath} className={styles.v}/>
        <path d={iPath} className={styles.i}/>

        {hoverT !== null && hv && (
          <g pointerEvents="none">
            <line x1={x(hoverT)} x2={x(hoverT)} y1={V_TOP} y2={AXIS_Y} className={styles.cross}/>
            <circle cx={x(hoverT)} cy={yV(hv.v)} r={4} className={styles.dotV}/>
            <circle cx={x(hoverT)} cy={yI(hv.i)} r={4} className={styles.dotI}/>
          </g>
        )}
      </svg>
      </div>
      <div className={styles.readout} aria-live="polite">
        {hv && hoverT !== null
          ? <>t = {us(hoverT).toFixed(2)} µs · <span className={styles.keyV}/>v_sw = {hv.v.toFixed(1)} V · <span className={styles.keyI}/>i_L = {hv.i.toFixed(3)} A · {hv.phase}</>
          : <>Hover or tap the chart to read values.</>}
      </div>
      <figcaption className={styles.caption}>
        Illustrative, computed from the ideal DCM model, not a measurement: V_in = {VIN} V, V_out = {VOUT} V,
        L = {L * 1e6} µH, C_sw = {C * 1e9} nF (f_r = {(W0 / 2 / Math.PI / 1e3).toFixed(0)} kHz,
        √(L/C_sw) = {Z0.toFixed(0)} Ω), f_sw = {FSW / 1e3} kHz, 1.5 A peak, damping Q ≈ {Q} assumed. The ring
        starts at 0 V and oscillates around V_out, peaking below 2·V_out as it decays; its current amplitude is at most
        V_out/√(L/C_sw).
      </figcaption>
      <details className={styles.table}>
        <summary>Key points (table)</summary>
        <table>
          <thead><tr><th>event</th><th>t (µs)</th><th>v_sw (V)</th><th>i_L (A)</th></tr></thead>
          <tbody>
            <tr><td>HS turns off</td><td>{us(T_ON).toFixed(2)}</td><td>{VIN} → 0</td><td>{IPK.toFixed(2)}</td></tr>
            <tr><td>LS turns off (i_L = 0), ring starts</td><td>{us(T_LS).toFixed(2)}</td><td>0</td><td>0</td></tr>
            <tr><td>first ring peak (T_r/2)</td><td>{us(T_LS + Math.PI / W0).toFixed(2)}</td>
              <td>{sample(T_LS + Math.PI / W0).v.toFixed(1)}</td><td>0</td></tr>
            <tr><td>next HS turn-on</td><td>{us(T).toFixed(2)}</td><td>{sample(T - 1e-12).v.toFixed(1)} → {VIN}</td>
              <td>{sample(T - 1e-12).i.toFixed(3)}</td></tr>
          </tbody>
        </table>
      </details>
    </figure>
  );
}
