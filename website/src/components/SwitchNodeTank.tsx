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

// Dashed bypass marking a capacitor that is a short at the ring frequency.
function AcShort({x, y1, y2, tx, ty}: {x: number; y1: number; y2: number; tx: number; ty: number}) {
  return (
    <g>
      <path className={styles.short} d={`M${x},${y1}V${y2}`} />
      <text x={tx} y={ty} className={styles.muted}>≈ short</text>
      <text x={tx} y={ty + 15} className={styles.muted}>
        at f<tspan baselineShift="sub" fontSize="0.75em">r</tspan>
      </text>
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
      <g className={styles.off}>
        <path d={`M${g},${c - 20}V${c + 20}`} />
        <path d={`M${ch},${c - 22}V${c - 10}M${ch},${c - 6}V${c + 6}M${ch},${c + 10}V${c + 22}`} />
        <path d={`M${ch},${c - 16}H${cx}M${ch},${c + 16}H${cx}M${ch},${c}H${cx}V${c + 16}`} />
        <path className={styles.fill} d={`M${ch + 2},${c}l9,-5v10Z`} />
        <path d={`M${g},${c}H${g - 45}`} />
      </g>
      <path d={`M${cx},${c - 16}V${top}M${cx},${c + 16}V${bot}`} />
      <path d={`M${cx},${yd}H${cx + 110}M${cx},${ys}H${cx + 110}`} />
      <Diode x={cx + 50} y1={yd} y2={ys} />
      <Cap x={cx + 110} y1={yd} y2={ys} cls={styles.par} />
      <circle className={styles.dot} cx={cx} cy={yd} r={3} />
      <circle className={styles.dot} cx={cx} cy={ys} r={3} />
      <Sub b="Q" s={name} x={g - 8} y={c - 30} textAnchor="end" className={styles.offText} />
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
        <svg viewBox="0 0 820 430" className={styles.svg} role="img"
             aria-label="Synchronous buck switch node with HS and LS MOSFETs, their body diodes and output capacitances, PCB capacitance, inductor L and output capacitor">
          <g className={styles.wire}>
            {/* rails */}
            <path d={`M70,${yVin}H${xQ}M70,${yGnd}H${xOut}`} />
            {/* input capacitor */}
            <Cap x={110} y1={yVin} y2={yGnd} />
            <Sub b="C" s="in" x={128} y={(yVin + yGnd) / 2 + 4} />
            <AcShort x={84} y1={yVin} y2={yGnd} tx={128} ty={(yVin + yGnd) / 2 + 22} />
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
            <AcShort x={xOut - 22} y1={ySw} y2={yGnd} tx={xOut + 18} ty={(ySw + yGnd) / 2 + 22} />
            <circle className={styles.dot} cx={xOut} cy={ySw} r={3} />
            <Sub b="V" s="out" x={xOut + 8} y={ySw - 10} />
          </g>

        </svg>
      </div>
    </figure>
  );
}

// Reduced tank: C_in/C_out shorted, FETs off, so all node capacitances sit in parallel with L to AC ground.
export function SwitchNodeTankSimple() {
  const yTop = 50, yBot = 210, xL = 400, caps: [number, string][] = [[90, 'oss,HS'], [190, 'oss,LS'], [290, 'pcb']];
  return (
    <figure className={styles.root}>
      <div className={styles.scroll}>
        <svg viewBox="0 0 520 250" className={`${styles.svg} ${styles.svgSmall}`} role="img"
             aria-label="Simplified LC tank: C_oss of the HS and LS FETs and the PCB capacitance in parallel with the inductor L">
          <g className={styles.wire}>
            <path className={styles.node} d={`M${caps[0][0]},${yTop}H${xL}`} />
            <path d={`M${caps[0][0]},${yBot}H${xL}`} />
            {caps.map(([x, s]) => (
              <g key={s}>
                <Cap x={x} y1={yTop} y2={yBot} cls={styles.par} />
                <Sub b="C" s={s} x={x + 18} y={(yTop + yBot) / 2 + 4} className={styles.parText} />
                <circle className={styles.dot} cx={x} cy={yTop} r={3} />
                <circle className={styles.dot} cx={x} cy={yBot} r={3} />
              </g>
            ))}
            <path d={`M${xL},${yTop}V${yTop + 30}M${xL},${yBot - 30}V${yBot}`} />
            <path className={styles.tank}
                  d={`M${xL},${yTop + 30}a12.5,12.5 0 0 1 0,25a12.5,12.5 0 0 1 0,25a12.5,12.5 0 0 1 0,25a12.5,12.5 0 0 1 0,25`} />
            <text x={xL + 22} y={(yTop + yBot) / 2 + 4} className={styles.tankText}>L</text>
            <text x={caps[0][0]} y={yTop - 10} className={styles.nodeText}>SW</text>
            <path d={`M245,${yBot}V${yBot + 12}M231,${yBot + 12}h28M236,${yBot + 17}h18M241,${yBot + 22}h8`} />
          </g>
          <g className={styles.note}>
            <text x={262} y={yBot + 20} className={styles.muted}>AC ground (V<tspan baselineShift="sub" fontSize="0.75em">in</tspan>, V<tspan baselineShift="sub" fontSize="0.75em">out</tspan>, GND)</text>
          </g>
        </svg>
      </div>
    </figure>
  );
}
