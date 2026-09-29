import React from 'react';
import LineChart, {Pt} from './LineChart';

// Ideal buck in DCM at fixed HS on-time; FET drops and ringing ignored.
const VIN = 60, VO = 27, L = 83e-6, FSW = 39e3, IPK = 1.5, VF = 0.8;
const T_ON = L * IPK / (VIN - VO);   // HS on-time
const T_ID = L * IPK / VO;           // ideal LS on-time: i_L reaches 0
const US = 1e6;

const params = <>V_in = {VIN} V, V_out = {VO} V, L = {L * 1e6} µH, f_sw = {FSW / 1e3} kHz, HS on-time
  {' '}{(T_ON * US).toFixed(2)} µs (1.5 A peak), body-diode V_f = {VF} V</>;

// Charge delivered to the output per period for a given LS on-time.
function ioutAt(tls: number): number {
  const q1 = 0.5 * IPK * T_ON;
  if (tls <= T_ID) {
    const i1 = IPK - VO / L * tls; // remainder carried by the body diode, falling at (V_o + V_f)/L
    return (q1 + IPK * tls - 0.5 * VO / L * tls * tls + 0.5 * i1 * i1 * L / (VO + VF)) * FSW;
  }
  const tau = tls - T_ID, iNeg = VO / L * tau; // reverse current, returned through the HS body diode
  return (q1 + 0.5 * IPK * T_ID - 0.5 * iNeg * tau - 0.5 * iNeg * iNeg * L / (VIN + VF - VO)) * FSW;
}

export function LsSweepChart(): React.JSX.Element {
  const data: Pt[] = Array.from({length: 281}, (_, k) => {
    const t = k * 7e-6 / 280;
    return [t * US, ioutAt(t)];
  });
  const i0 = ioutAt(0), ipk = ioutAt(T_ID), i6 = ioutAt(6e-6);
  return (
    <LineChart
      ariaLabel="Average output current versus LS on-time: a shallow rise from the body-diode-only value to a peak where LS turns off at the zero crossing, then a steep drop as reverse current starts"
      xMin={0} xMax={7} xTicks={[0, 1, 2, 3, 4, 5, 6, 7]} xLabel="LS on-time" xUnit="µs"
      panels={[{
        title: 'average output current I_out', unit: 'A', yMin: 0.18, yMax: 0.26, yTicks: [0.18, 0.2, 0.22, 0.24, 0.26],
        series: [{name: 'I_out', data, slot: 1}],
        markers: [
          {x: 0, y: i0, slot: 1, label: 'LS never on: body diode only', anchor: 'start', dy: 18},
          {x: T_ID * US, y: ipk, slot: 1, label: 'peak: LS off at i_L = 0', anchor: 'middle', dy: -10},
          {x: 6, y: i6, slot: 1, label: 'reverse current', anchor: 'end', dy: 4},
        ],
      }]}
      caption={<>Illustrative, computed from the ideal DCM model, not a measurement: {params}. Left of the peak the
        body diode carries the rest of the fall at (V_out + V_f)/L, a small loss; right of it the current reverses
        and the charge returned to the input grows with the square of the overshoot. Real sweeps add the FET
        drops and the hardware offset <code>rect_offset_ns</code>, which shifts the peak.</>}
      table={{
        head: ['LS on-time (µs)', 'I_out (A)'],
        rows: [[0, i0.toFixed(4)], [(T_ID * US).toFixed(2), ipk.toFixed(4)], [6, i6.toFixed(4)], [7, ioutAt(7e-6).toFixed(4)]],
      }}
      digits={{x: 2, y: 4}}
    />
  );
}

// i_L(t) with the LS switched off after `tls`.
function iL(t: number, tls: number): number {
  if (t < T_ON) return IPK * t / T_ON;
  const tf = t - T_ON;
  if (tls <= T_ID) {
    if (tf < tls) return IPK - VO / L * tf;
    const i1 = IPK - VO / L * tls;
    return Math.max(0, i1 - (VO + VF) / L * (tf - tls));
  }
  if (tf < tls) return IPK - VO / L * tf;
  const iNeg = IPK - VO / L * tls; // negative
  return Math.min(0, iNeg + (VIN + VF - VO) / L * (tf - tls));
}

const EARLY = 3.0e-6, LATE = 6.2e-6, T_END = 13e-6;

function trace(tls: number): Pt[] {
  const edges = [T_ON, T_ON + tls];
  const pts = Array.from({length: 521}, (_, k) => k * T_END / 520).concat(edges).sort((a, b) => a - b);
  return pts.map(t => [t * US, iL(t, tls)]);
}

export function LsTimingChart(): React.JSX.Element {
  const iLate = iL(T_ON + LATE, LATE), iEarly = iL(T_ON + EARLY, EARLY);
  const diodeEnd = T_ON + EARLY + iEarly * L / (VO + VF);
  const lateEnd = T_ON + LATE - iLate * L / (VIN + VF - VO);
  return (
    <LineChart
      ariaLabel="Inductor current for three LS turn-off times: early, at the zero crossing, and late with reverse current"
      xMin={0} xMax={13} xTicks={[0, 2, 4, 6, 8, 10, 12]} xLabel="time" xUnit="µs"
      panels={[
        {
          title: 'i_L, LS off at the zero crossing (ideal)', unit: 'A', yMin: -0.75, yMax: 2.25, yTicks: [-0.5, 0, 0.5, 1, 1.5], height: 120,
          series: [{name: 'ideal: LS off at the zero crossing', data: trace(T_ID), slot: 1}],
          markers: [{x: (T_ON + T_ID) * US, y: 0, slot: 1, label: 'LS off at 0 A', anchor: 'start', dy: -8}],
        },
        {
          title: 'i_L, LS off early', unit: 'A', yMin: -0.75, yMax: 2.25, yTicks: [-0.5, 0, 0.5, 1, 1.5], height: 120,
          series: [{name: 'early: body diode takes the rest', data: trace(EARLY), slot: 3}],
          markers: [{x: (T_ON + EARLY) * US, y: iEarly, slot: 3, label: 'LS opens; body diode conducts the rest', anchor: 'start', dy: -8}],
        },
        {
          title: 'i_L, LS off late', unit: 'A', yMin: -0.75, yMax: 2.25, yTicks: [-0.5, 0, 0.5, 1, 1.5], height: 120,
          series: [{name: 'late: reverse current', data: trace(LATE), slot: 2}],
          markers: [{x: (T_ON + LATE) * US, y: iLate, slot: 2, label: `LS opens at ${iLate.toFixed(2)} A`, anchor: 'start', dy: 14}],
        },
      ]}
      caption={<>Illustrative, computed from the ideal DCM model: {params}; LS on-time {EARLY * US} µs (early),
        {' '}{(T_ID * US).toFixed(2)} µs (ideal) and {LATE * US} µs (late). Early: the diode ends the fall slightly
        faster, at (V_out + V_f)/L, and the cost is only V_f·I while it conducts. Late: the current keeps falling
        at −V_out/L through zero, reaches {iLate.toFixed(2)} A and flows back to the input until{' '}
        {(lateEnd * US).toFixed(2)} µs, which is why the controller biases toward early.</>}
      table={{
        head: ['case', 'LS off (µs)', 'i_L at LS off (A)', 'i_L = 0 again (µs)'],
        rows: [
          ['early', ((T_ON + EARLY) * US).toFixed(2), iEarly.toFixed(3), (diodeEnd * US).toFixed(2)],
          ['ideal', ((T_ON + T_ID) * US).toFixed(2), '0', ((T_ON + T_ID) * US).toFixed(2)],
          ['late', ((T_ON + LATE) * US).toFixed(2), iLate.toFixed(3), (lateEnd * US).toFixed(2)],
        ],
      }}
    />
  );
}

export function MSensitivityChart(): React.JSX.Element {
  const data: Pt[] = Array.from({length: 201}, (_, k) => {
    const m = 0.3 + k * 0.67 / 200;
    return [m, 2 / (1 - m)];
  });
  const pts = [0.5, 0.8, 0.9, 0.95];
  return (
    <LineChart
      ariaLabel="LS on-time error for a 2 percent error in M, rising steeply as M approaches 1"
      xMin={0.3} xMax={1} xTicks={[0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]} xLabel="M = V_o/V_i" xUnit=""
      panels={[{
        title: 'LS on-time error for a 2 % error in M', unit: '%', yMin: 0, yMax: 70, yTicks: [0, 20, 40, 60],
        series: [{name: 'error', data, slot: 1}],
        markers: pts.map(m => ({x: m, y: 2 / (1 - m), slot: 1 as const, label: `${(2 / (1 - m)).toFixed(0)} %`, anchor: 'end' as const, dy: -6})),
      }]}
      caption={<>|Δt_on,LS / t_on,LS| = 2 % / (1 − M), from the sensitivity formula above; the markers are the rows of the
        table.</>}
      digits={{x: 3, y: 1}}
    />
  );
}
