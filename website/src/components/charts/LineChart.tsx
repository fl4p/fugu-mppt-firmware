import React, {useId, useRef, useState} from 'react';
import styles from './LineChart.module.css';

export type Pt = [number, number];
export type Series = {name: string; data: Pt[]; slot: 1 | 2 | 3};
export type Marker = {x: number; y: number; slot: 1 | 2 | 3; label?: string; anchor?: 'start' | 'middle' | 'end'; dy?: number};
export type Panel = {
  title: string; unit: string; yMin: number; yMax: number; yTicks: number[];
  series: Series[]; markers?: Marker[]; refY?: {y: number; label?: string}[]; height?: number;
};
export type Band = {from: number; to: number; label?: string};

type Props = {
  ariaLabel: string; panels: Panel[]; xMin: number; xMax: number; xTicks: number[]; xLabel: string; xUnit: string;
  bands?: Band[]; caption?: React.ReactNode; table?: {head: string[]; rows: (string | number)[][]};
  digits?: {x?: number; y?: number};
};

const VW = 720, PL = 52, PR = 16, TOP = 28, GAP = 34;

// Linear interpolation on x-sorted samples; vertical edges are encoded as duplicate x values.
function at(data: Pt[], x: number): number | null {
  if (!data.length || x < data[0][0] || x > data[data.length - 1][0]) return null;
  let lo = 0, hi = data.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (data[mid][0] <= x) lo = mid; else hi = mid;
  }
  const [x0, y0] = data[lo], [x1, y1] = data[hi];
  return x1 === x0 ? y1 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

export default function LineChart(p: Props): React.JSX.Element {
  const svgRef = useRef<SVGSVGElement>(null);
  const [hx, setHx] = useState<number | null>(null);
  const cid = `lc${useId().replace(/[^a-zA-Z0-9]/g, '')}`;
  const tops: number[] = [];
  let y = TOP;
  for (const pn of p.panels) { tops.push(y); y += (pn.height ?? 150) + GAP; }
  const axisY = y - GAP, VH = axisY + 44;
  const sx = (v: number) => PL + (v - p.xMin) / (p.xMax - p.xMin) * (VW - PL - PR);
  const sy = (k: number, v: number) => {
    const pn = p.panels[k], h = pn.height ?? 150;
    return tops[k] + h * (1 - (v - pn.yMin) / (pn.yMax - pn.yMin));
  };
  const dx = p.digits?.x ?? 2, dy = p.digits?.y ?? 3;

  const onMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const r = svgRef.current!.getBoundingClientRect();
    const v = p.xMin + ((e.clientX - r.left) / r.width * VW - PL) / (VW - PL - PR) * (p.xMax - p.xMin);
    setHx(v >= p.xMin && v <= p.xMax ? v : null);
  };
  // Keyboard: arrows step 1 % of the range (Shift: 10 %), Home/End jump, Escape clears.
  const onKey = (e: React.KeyboardEvent<SVGSVGElement>) => {
    const span = p.xMax - p.xMin, cur = hx ?? (p.xMin + p.xMax) / 2;
    const step = span / (e.shiftKey ? 10 : 100);
    const next = e.key === 'ArrowRight' ? cur + step : e.key === 'ArrowLeft' ? cur - step
      : e.key === 'Home' ? p.xMin : e.key === 'End' ? p.xMax : e.key === 'Escape' ? null : undefined;
    if (next === undefined) return;
    e.preventDefault();
    setHx(next === null ? null : Math.min(p.xMax, Math.max(p.xMin, next)));
  };
  const legend = p.panels.flatMap(pn => pn.series).filter((s, i, a) => a.findIndex(o => o.name === s.name) === i);

  return (
    <figure className={styles.root}>
      {legend.length >= 2 && (
        <div className={styles.legend}>
          {legend.map(s => <span key={s.name}><span className={styles[`key${s.slot}`]}/>{s.name}</span>)}
        </div>
      )}
      <div className={styles.scroll}>
      <svg ref={svgRef} viewBox={`0 0 ${VW} ${VH}`} className={styles.svg} role="img" tabIndex={0}
           aria-label={`${p.ariaLabel}. Focus and use the arrow keys to read values.`}
           onPointerMove={onMove} onPointerLeave={() => setHx(null)} onKeyDown={onKey}
           onFocus={() => setHx(h => h ?? (p.xMin + p.xMax) / 2)} onBlur={() => setHx(null)}>
        <defs>
          {p.panels.map((pn, k) => (
            <clipPath key={k} id={`${cid}-${k}`}>
              <rect x={PL} y={tops[k]} width={VW - PL - PR} height={pn.height ?? 150}/>
            </clipPath>
          ))}
        </defs>
        {(p.bands ?? []).map((b, k) => (
          <g key={`b${k}`}>
            <rect x={sx(b.from)} y={TOP} width={sx(b.to) - sx(b.from)} height={axisY - TOP} className={styles.band}/>
            {b.label && <text x={(sx(b.from) + sx(b.to)) / 2} y={TOP - 10} className={styles.note} textAnchor="middle">{b.label}</text>}
          </g>
        ))}
        {p.panels.map((pn, k) => (
          <g key={`p${k}`}>
            {pn.yTicks.map(t => (
              <g key={t}>
                <line x1={PL} x2={VW - PR} y1={sy(k, t)} y2={sy(k, t)} className={t === 0 ? styles.zero : styles.grid}/>
                <text x={PL - 6} y={sy(k, t)} className={styles.tick} textAnchor="end" dominantBaseline="middle">{t}</text>
              </g>
            ))}
            <text x={PL + 6} y={tops[k] + 12} className={styles.title}>{pn.title} ({pn.unit})</text>
            {(pn.refY ?? []).map((r, j) => (
              <g key={`r${j}`}>
                <line x1={PL} x2={VW - PR} y1={sy(k, r.y)} y2={sy(k, r.y)} className={styles.ref}/>
                {r.label && <text x={VW - PR - 4} y={sy(k, r.y) - 4} className={styles.note} textAnchor="end">{r.label}</text>}
              </g>
            ))}
            {pn.series.map(s => (
              <path key={s.name} className={styles[`s${s.slot}`]} clipPath={`url(#${cid}-${k})`}
                    d={s.data.map(([a, b], j) => `${j ? 'L' : 'M'}${sx(a).toFixed(1)},${sy(k, b).toFixed(1)}`).join('')}/>
            ))}
            {(pn.markers ?? []).map((m, j) => (
              <g key={`m${j}`}>
                <circle cx={sx(m.x)} cy={sy(k, m.y)} r={4} className={styles[`d${m.slot}`]}/>
                {m.label && <text x={sx(m.x) + (m.anchor === 'end' ? -8 : m.anchor === 'middle' ? 0 : 8)}
                                  y={sy(k, m.y) + (m.dy ?? -8)} className={styles.note}
                                  textAnchor={m.anchor ?? 'start'}>{m.label}</text>}
              </g>
            ))}
          </g>
        ))}
        {p.xTicks.map(t => <text key={`x${t}`} x={sx(t)} y={axisY + 16} className={styles.tick} textAnchor="middle">{t}</text>)}
        <text x={(PL + VW - PR) / 2} y={axisY + 36} className={styles.axis} textAnchor="middle">{p.xLabel}{p.xUnit ? ` (${p.xUnit})` : ''}</text>
        {hx !== null && (
          <g pointerEvents="none">
            <line x1={sx(hx)} x2={sx(hx)} y1={TOP} y2={axisY} className={styles.cross}/>
            {p.panels.map((pn, k) => pn.series.map(s => {
              const v = at(s.data, hx);
              return v === null ? null : <circle key={`h${k}${s.name}`} cx={sx(hx)} cy={sy(k, v)} r={4} className={styles[`d${s.slot}`]}/>;
            }))}
          </g>
        )}
      </svg>
      </div>
      <div className={styles.readout} aria-live="polite">
        {hx === null ? 'Hover or tap the chart, or focus it and use the arrow keys, to read values.' : <>
          {p.xLabel} = {hx.toFixed(dx)}{p.xUnit ? ` ${p.xUnit}` : ''}
          {p.panels.map(pn => pn.series.map(s => {
            const v = at(s.data, hx);
            return v === null ? null : <span key={`${pn.title}${s.name}`}> · <span className={styles[`key${s.slot}`]}/>
              {pn.series.length > 1 || p.panels.length === 1 ? s.name : pn.title} = {v.toFixed(dy)} {pn.unit}</span>;
          }))}
        </>}
      </div>
      {p.caption && <figcaption className={styles.caption}>{p.caption}</figcaption>}
      {p.table && (
        <details className={styles.table}>
          <summary>Key points (table)</summary>
          <table>
            <thead><tr>{p.table.head.map(h => <th key={h}>{h}</th>)}</tr></thead>
            <tbody>{p.table.rows.map((r, k) => <tr key={k}>{r.map((c, j) => <td key={j}>{c}</td>)}</tr>)}</tbody>
          </table>
        </details>
      )}
    </figure>
  );
}
