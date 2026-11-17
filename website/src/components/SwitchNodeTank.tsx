import React from 'react';
import styles from './SwitchNodeTank.module.css';

// Synchronous buck power stage during the DCM ring: both FETs off, L rings with the node capacitance.

const Sub = ({b, s, ...p}: {b: string; s?: string} & React.SVGProps<SVGTextElement>) => (
  <text {...p}>
    {b}
    {s && <tspan baselineShift="sub" fontSize="0.75em">{s}</tspan>}
  </text>
);

// Vertical capacitor between y1 and y2, plates at the midpoint.
function Cap({x, y1, y2, cls}: {x: number; y1: number; y2: number; cls?: string}) {
  const m = (y1 + y2) / 2;
  return (
    <g className={cls}>
      <path d={`M${x},${y1}V${m - 5}M${x},${m + 5}V${y2}`} />
      <path className={styles.plate} d={`M${x - 13},${m - 5}h26M${x - 13},${m + 5}h26`} />
    </g>
  );
}

// Vertical diode, anode at y2 (bottom), cathode at y1 (top).
function Diode({x, y1, y2}: {x: number; y1: number; y2: number}) {
  const m = (y1 + y2) / 2;
  return (
    <g>
      <path d={`M${x},${y1}V${m - 8}M${x},${m + 8}V${y2}`} />
      <path className={styles.fill} d={`M${x - 9},${m + 8}h18L${x},${m - 8}Z`} />
      <path d={`M${x - 9},${m - 8}h18`} />
    </g>
  );
}

// n-channel MOSFET, drain/source lead on x = cx, centre y = c; body diode and C_oss to the right.
function Fet({cx, c, name, top, bot}: {cx: number; c: number; name: string; top: number; bot: number}) {
  const g = cx - 30, ch = g + 8;
  const yd = c - 55, ys = c + 55;
  return (
    <g>
      <path d={`M${g},${c - 20}V${c + 20}`} />
      <path d={`M${ch},${c - 22}V${c - 10}M${ch},${c - 6}V${c + 6}M${ch},${c + 10}V${c + 22}`} />
      <path d={`M${ch},${c - 16}H${cx}V${top}M${ch},${c + 16}H${cx}V${bot}M${ch},${c}H${cx}V${c + 16}`} />
      <path className={styles.fill} d={`M${ch + 2},${c}l9,-5v10Z`} />
      <path d={`M${g},${c}H${g - 45}`} />
      <path d={`M${cx},${yd}H${cx + 110}M${cx},${ys}H${cx + 110}`} />
      <Diode x={cx + 50} y1={yd} y2={ys} />
      <Cap x={cx + 110} y1={yd} y2={ys} cls={styles.par} />
      <circle className={styles.dot} cx={cx} cy={yd} r={3} />
      <circle className={styles.dot} cx={cx} cy={ys} r={3} />
      <Sub b="Q" s={name} x={g - 8} y={c - 30} textAnchor="end" />
      <text x={g - 50} y={c + 4} textAnchor="end" className={styles.muted}>off</text>
      <Sub b="D" s="body" x={cx + 62} y={c + 4} />
      <Sub b="C" s={`oss,${name}`} x={cx + 128} y={c + 4} className={styles.parText} />
    </g>
  );
}

export default function SwitchNodeTank() {
  const yVin = 50, ySw = 230, yGnd = 410, xQ = 320, xL = 600, xOut = 730;
  return (
    <figure className={styles.root}>
      <div className={styles.scroll}>
        <svg viewBox="0 0 820 500" className={styles.svg} role="img"
             aria-label="Synchronous buck switch node with HS and LS MOSFETs, their body diodes and output capacitances, PCB capacitance, inductor L and output capacitor">
          <g className={styles.wire}>
            {/* rails */}
            <path d={`M70,${yVin}H${xQ}M70,${yGnd}H${xOut}`} />
            {/* input capacitor */}
            <Cap x={110} y1={yVin} y2={yGnd} />
            <Sub b="C" s="in" x={128} y={(yVin + yGnd) / 2 + 4} />
            <circle className={styles.dot} cx={110} cy={yVin} r={3} />
            <circle className={styles.dot} cx={110} cy={yGnd} r={3} />
            <Sub b="V" s="in" x={66} y={yVin + 4} textAnchor="end" />

            <Fet cx={xQ} c={140} name="HS" top={yVin} bot={ySw} />
            <Fet cx={xQ} c={320} name="LS" top={ySw} bot={yGnd} />

            {/* switch node */}
            <path className={styles.node} d={`M${xQ},${ySw}H${xL}`} />
            <circle className={styles.dot} cx={xQ} cy={ySw} r={3.5} />
            <text x={xQ + 8} y={ySw - 8} className={styles.nodeText}>SW</text>
            <Cap x={540} y1={ySw} y2={yGnd} cls={styles.par} />
            <Sub b="C" s="pcb" x={558} y={(ySw + yGnd) / 2 + 4} className={styles.parText} />
            <circle className={styles.dot} cx={540} cy={ySw} r={3} />
            <circle className={styles.dot} cx={540} cy={yGnd} r={3} />

            {/* inductor */}
            <path className={styles.tank}
                  d={`M${xL},${ySw}a12.5,12.5 0 0 1 25,0a12.5,12.5 0 0 1 25,0a12.5,12.5 0 0 1 25,0a12.5,12.5 0 0 1 25,0`} />
            <text x={xL + 50} y={ySw - 22} textAnchor="middle" className={styles.tankText}>L</text>
            <path d={`M${xL + 100},${ySw}H${xOut}`} />

            {/* output */}
            <Cap x={xOut} y1={ySw} y2={yGnd} />
            <Sub b="C" s="out" x={xOut + 18} y={(ySw + yGnd) / 2 + 4} />
            <circle className={styles.dot} cx={xOut} cy={ySw} r={3} />
            <Sub b="V" s="out" x={xOut + 8} y={ySw - 10} />
          </g>

          <g className={styles.note}>
            <Sub b="C" s="sw" x={70} y={452} />
            <text x={96} y={452}>
              = n<tspan baselineShift="sub" fontSize="0.75em">HS</tspan>·C<tspan baselineShift="sub" fontSize="0.75em">oss,HS</tspan>
              {' '}+ n<tspan baselineShift="sub" fontSize="0.75em">LS</tspan>·C<tspan baselineShift="sub" fontSize="0.75em">oss,LS</tspan>
              {' '}+ C<tspan baselineShift="sub" fontSize="0.75em">pcb</tspan>,
              {'   '}f<tspan baselineShift="sub" fontSize="0.75em">r</tspan> = 1 / (2π√(L·C<tspan baselineShift="sub" fontSize="0.75em">sw</tspan>))
            </text>
            <text x={70} y={478} className={styles.muted}>
              C<tspan baselineShift="sub" fontSize="0.75em">in</tspan> and C<tspan baselineShift="sub" fontSize="0.75em">out</tspan> are
              AC ground for the ring; n = paralleled FETs per position; C<tspan baselineShift="sub" fontSize="0.75em">oss</tspan> taken
              at V<tspan baselineShift="sub" fontSize="0.75em">DS</tspan> near 0 V.
            </text>
          </g>
        </svg>
      </div>
    </figure>
  );
}
