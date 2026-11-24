*this document is an LLM generated placeholder*

# MPP tracker: simulation records, model derivation and tuning history

Text moved out of the published page `website/docs/internals/mppt-tracker.md` on 2026-11-24, because the
firmware source cannot confirm it (simulation results, field timings, tuning history) or because it is a
derivation rather than reference. The figures referenced below stay in
`website/docs/internals/img/mppt-tracker/`.

## Sweep duration (field logs)

The published page used to say "a sweep takes 20–60 s, depending on the loop rate". After the `sweep_pace`
speed-up and the change that ends the sweep at the Vin floor instead of slowing it, field logs showed about 6–9 s,
8.5 s above 200 W. A sweep throttled by the CV limiter (battery terminated) took about 55 s.

## Input capacitance setting (dusk test, simulation)

The sweep landing is not monotonic in `converter.conf::c_in`: with a 1.2 mF plant, 0.5, 1 and 2 mF harvested 89.5,
91.2 and 95.8 % in a dusk test at 4 W, but 97.6, 97.5 and 95.8 % at 10 W.

## Settle window (simulation)

- A 1.5 s settle-window cap harvested less than a fixed 150 ms window, because the excursion it allows costs more
  than the filter lag does. Hence the 1 s cap.
- A settle window in slow mode: it skipped evaluations when changing light kept Vin from coming back to the reversal
  voltage, and with power filters that settle in more than 500 ms (filter length 60 at a low sample rate). Under an
  irradiance wobble of 0.5 % at 0.2 Hz it cost 3 to 4 % of harvest without sensor noise and 0.2 to 1.7 % with
  measured sensor noise.
- Without the window in slow mode, the trade-off was at dusk: 0.1 to 0.5 % of harvest on average, up to 0.8 % in a
  single run with a slow power filter, and up to 1.4 % for single noise realisations. The tracker also reverses more
  often (with a slow filter, 21.5 to 28 reversals per minute at dusk).
- Vin condition deadband: ending the window 0.1 % of Vin early lost about 1 % of harvest at 2 to 20 W; ending it
  0.1 % late made no measurable difference.

## Slow-mode exit time

A slow-mode pass across its power band near the MPP takes about 35 s, so a limit of 15 s without a reversal dropped
back to fast mode every 20 to 60 s. Hence 45 s.

## Plant moving against the step (simulation)

Simulated board model with a 233 kHz switch-node ring and a battery output, at dusk light (about 23 W, MPP at 48 V):

- Over about 12 % of the PWM period below $D = M$, Vin stays within 0.3 V. The coil current at turn-on goes from
  +0.1 A to -0.3 A, which cancels the extra on-time.
- Across that span $P(D)$ is flat with a slight rise toward higher duty, a local maximum at 37 V. The dip on the way
  back to the MPP is 0.5 % of the power, below both thresholds.
- At an unchanged duty, Vin can step down by 3 to 5 V within 0.3 s.

With the 5 % rule counting moves in either direction, the tracker ended on the local maximum, up to 18 V below the
MPP, until the next sweep. Counting only moves in the steered direction removed the trap: in simulated trapped runs
the old rule had made 36 to 47 reversals per run.

The trap occurs at both PWM resolutions (2047 and 4103 counts per period) with the low side turning off at the
modelled zero crossing (`rect_offset_ns` 0). With the low side turning off 1.3 µs early (`rect_offset_ns` -1312), it
did not occur in 144 runs, with or without the rule.

A power drop coinciding with such a step reversed the tracker about once per trapped run. A guard without the 1 %
margin blocked every reversal of a leg toward higher Vin and none toward lower Vin, so the tracker ratcheted up the
steep side of the P-V curve: under an irradiance wobble of 0.5 % at 0.2 Hz (Vin moving about 0.4 %) it settled 4 V
above the MPP and lost 4 % of harvest on one board model.

## Steady-state efficiency and noise (model)

A simplified model shows how measurement noise limits P&O in steady light. It assumes a fixed curve (constant
irradiance and temperature) and independent noise in each period's power reading. Near the MPP the power curve is
a parabola in the relative voltage offset $x$:

$$
P(x) = P_m \left(1 - c\,x^2\right), \qquad x = \frac{V - V_m}{V_m}
$$

The single-diode model gives the curvature per cell, independent of the number of cells in series:

$$
c \approx 1 + \frac{V_m}{2\,n\,V_t}
$$

which is about 9 for crystalline silicon. The tracking efficiency is the mean power relative to the MPP:

$$
\eta = \frac{\overline{P}}{P_m} = 1 - c\,\overline{x^2}
$$

### Without noise

P&O with step $\delta = \Delta V / V_m$ oscillates around the MPP with

$$
\eta = 1 - \frac{c\,\delta^2}{2}
$$

For steps of about one PWM count this loss is negligible.

### Noise larger than the step

Let $\sigma$ be the relative noise of each period-averaged power reading. When $\sigma$ exceeds the per-step power
change, the operating point performs a random walk with a drift toward the MPP. Its stationary variance is

$$
\overline{x^2} = \frac{\sqrt{\pi}\,\sigma}{4\,c}
$$

so step size and curvature cancel:

$$
\eta \approx 1 - \frac{\sqrt{\pi}}{4}\,\sigma \approx 1 - 0.44\,\sigma
$$

The approximation ignores the direction persistence of P&O and holds to within about a factor of 2.

### Decision threshold

With a relative decision threshold $\theta$ and a reference that follows every reading, the point wanders within
$P > P_m (1 - \theta)$, and

$$
\eta \approx 1 - \frac{\theta}{3}
$$

The tracker holds its reference until the power changes by more than $\theta$, so on the way up the reference can
sit up to $\theta$ below the peak, and the reversal on the way down comes another $\theta$ below the reference. The
band then reaches $P_m(1 - 2\theta)$ and the loss lies between $\theta/3$ and $2\theta/3$.

Figure: `img/mppt-tracker/po-band.svg` (parabola model with $c = 9$; inner band $P > P_m(1 - \theta)$, outer band
$P > P_m(1 - 2\theta)$, with the half-width in Vin offset).

The band model holds only while $\theta$ is well above the noise. In simulation the loss was 0.5 to 0.8 $\theta$
without noise and for $\theta \gtrsim 3\sigma$. Below $\theta \approx \sigma$ the tracker reverses mostly on noise,
and the loss levels off near $1\,\sigma$ instead of falling with $\theta$. A threshold above the noise then costs
more than it saves, unless the light changes quickly: a larger $\theta$ also delays the reaction to rising light,
and in heavy noise with moving clouds it prevents reversals on noise that walk the tracker off the MPP.

The difference of two readings has noise $\sqrt{2}\,\sigma$. Writing $\theta = k\sqrt{2}\,\sigma$, false
reversals become rare from $k \approx 2 \ldots 3$, which costs

$$
\eta \approx 1 - (0.9 \ldots 1.4)\,\sigma
$$

The fixed thresholds of the tracker sit below that: in simulation with measured sensor noise they correspond to
$k \approx 0.6$ in fast mode and $0.2 \ldots 0.3$ in slow mode, so the tracker accepts noise-driven reversals in
steady light. Thresholds derived from a running noise estimate (fixed $k$ from 1.4 to 2.5) gained at most a few
tenths of a percent in simulation and lost up to several percent at low light, where the current-sensor noise
floor does not scale with current; the tracker therefore keeps fixed thresholds.

### Averaging and low power

The decision period $T_d$ is the tracker period, not the sensor sample period: 33 ms in fast mode and 500 ms in
slow mode. The tracker averages the filtered power samples of one period. The sensor filters already average the
raw samples and correlate successive filtered samples, so the period mean gains less than $\sqrt{f_s T_d}$.
Modelling the filtered power as noise $\sigma_f$ with exponential correlation and time constant $\tau$, the
variance of a mean over $T_d$ is

$$
\sigma^2 = \sigma_f^2\, g\!\left(\frac{T_d}{\tau}\right), \qquad
g(u) = \frac{2}{u}\left(1 - \frac{1 - e^{-u}}{u}\right)
$$

with $g \to 1$ for $T_d \ll \tau$ and $g \to 2\tau/T_d$ for $T_d \gg \tau$. The filter's settling time gives
$\tau \approx t_f / 6.64$. For the INA226 profile at 450 Hz ($t_f$ = 152 ms, $\tau \approx$ 23 ms):

| Mode | $T_d$  | $T_d/\tau$ | $g$   | $\sigma / \sigma_f$ |
|------|--------|------------|-------|---------------------|
| Fast | 33 ms  | 1.4        | 0.65  | 0.81                |
| Slow | 500 ms | 22         | 0.087 | 0.30                |

Figure: `img/mppt-tracker/variance-g.svg` ($g(u)$ with its asymptote $2\tau/T_d$, for $\tau$ = 23 ms; fast mode
sits on the flat part, slow mode on the asymptote).

In fast mode the period mean is barely quieter than one filtered sample. The two-pass EWMA is not a single
exponential, so these values are approximate. At low power $\sigma$ grows as $1/P$, so dawn, dusk and low
light dominate the steady-state loss. A longer decision period lowers $\sigma$ but follows changing irradiance more
slowly, which is a separate loss.

### Combined estimate

Writing the noise as the absolute power noise $s_f$ of one filtered sample, so that
$\sigma = s_f \sqrt{g} / P$, and taking the larger of the threshold and noise-walk losses gives

$$
\eta \approx 1 - \frac{c\,\delta^2}{2} - \max\!\left(\kappa\,\frac{\theta_\text{eff}}{3},\; \frac{\sqrt{\pi}}{4}\,\frac{s_f\sqrt{g(T_d/\tau)}}{P}\right)
$$

where $\kappa$ = 1 to 2 accounts for the held reference. The three terms come from separate limiting cases and are
not exact in combination, so the result is an estimate to within about a factor of 2. Both loss terms grow as $1/P$
at low power. With $\theta \approx k\sqrt{2}\,\sigma$ and $k \ge 2$ the threshold term is the larger one; at the
tracker's fixed thresholds ($k < 1$) the two terms are of similar size.

## Partial peak scan cost (estimate)

The published page used to say: a scan takes 1.5 to 4 s and costs 10 to 150 J at 200 to 450 W, depending on the
number of probes (the power given up at the probes plus the travel between them). A scan that moves to a better
maximum pays this back within seconds. The code bounds each probe at 1.5 s travel + at most 1 s settle + 100 ms
averaging, and the scan at 30 s (`src/peak_scan.h`).
