The VirtualConverter (vconv) models a DC-DC synchronous buck or boost converter.
It models the half-bridge MOSFETs (ideal, zero loss), the input and output electrolytic caps and
the coil (ideal).

Input power source: a current source with clamped voltage to model a simplified solar panel.
Output power sink: a constant voltage source as a simplified model for a battery.

The converter model simulates a mock converter for converter firmware tests.
The signal inputs from the firmware are:
binary PWM Gate drive signals for LS and HS
-or-
PWM counts (resolution, LS counts, HS counts).

This still needs to be discussed. binary PWM gate drive signals seems more natural, whereas PWM counts are directly
available in the firmware.

The outputs are mock ADC values:
Vin, Vout, Iout.

Model details:
* DCM/CCM mode
* Reverse coil current
* Waveform of coil current
* Voltage ripple
* Solar model: Isc/Voc

For Later:
- parasitics for passive components
- mosfet power loss
- other power sources / sinks

Do you think its better to first stick to the buck case and then later do boost or both together?
Is this feasible? Where do you see difficulties? Are there relaxations/simplifications to make it easier to implement?
Can we use existing software?


* ignore: backflow switch, out of scope


* don't use params from :
* dont' extract any mode constants from the firmware (`buck.h`). the firmware is the DuT, fitting the converter model to the firmware is counter objective.
* we are trying to model physical laws here, not the firmware. 
* Where does vconv run, and how does it couple to the firmware? → On-target, replace ADC_Fake
* How should vconv read the PWM state from the converter? → Read PWM counts directly
*  What time-step strategy should the model use? Driven by your stated need for DCM, reverse current, ripple,
   and waveform. → Analytic per-cycle (Recommended)
* · What PV input model do you want? (Battery output is going to be a stiff voltage source either way — a
  separate Rint can come later.) → Exponential / single-diode (Recommended)





Design sketch — VirtualConverter

1. File / module layout

Three new files, one edited:

src/sim/vconv.h           ~150 LOC  pure-C++ converter model (no Arduino/IDF deps)
src/sim/vconv.cpp         ~50 LOC   non-inline math helpers
src/adc/vconv.h           ~100 LOC  ADC_VConv : AsyncADC<float> backend, glues vconv + scheduler
src/main.cpp::setupSensors()        new case `*_adc=vconv` selecting ADC_VConv

src/sim/vconv.* keeps the model free of Arduino/IDF so it stays trivially host-testable in test/host-stub/.
src/adc/vconv.h (which DOES depend on the project's timer/notification infra) holds the on-target shell. Mirrors
the existing split between buck.h (logic) and pwm/{ledc,mcpwm,mock}.h (HW shells).

Does that boundary look right?

2. Core model — state and step

State variables (3 scalars):
- V_in — input cap voltage
- V_out — output cap voltage
- I_L_end — coil current at the end of the previous PWM cycle (carries DCM/CCM state)

Per step(pwm_counts), run analytic PWM-cycle solution:

t_HS = pwmCtrl / pwmMax / fsw     // HS on-time (buck)
t_LS = pwmRect / pwmMax / fsw     // LS on-time

I_L(t) during HS = I_L_end + (V_in - V_out)/L * t
I_L(t) during LS = I_L(t_HS) + (-V_out)/L * (t - t_HS)

DCM: if during LS phase I_L would cross zero AND we're emulating a diode (sync rect off below the zero crossing),
clamp I_L = 0 for the rest of the period. Reverse current: if LS is held longer than the zero crossing (forced
PWM, or buck.h's reverse-current regime), I_L goes negative — model lets that happen and reports it back.

Averaged over the cycle:
I_in_avg = ∫I_L during HS / T_period       // PV current draw
I_out_avg = ∫I_L during HS+LS / T_period   // load delivery (matches Iout sensor)

Cap dynamics, forward-Euler at PWM rate (which is the model step):
V_in  += (I_pv(V_in)  - I_in_avg)  * T_period / C_in
V_out += (I_out_avg   - I_bat)     * T_period / C_out  (battery sinks excess; if V_out > V_bat use stiff clamp)

PV (exponential, single-diode-ish):
I_pv(V) = Isc * (1 - exp((V - Voc) / k))
with k tuned so MPP lands near ~0.8 * Voc. Goes negative above Voc — clamp to 0.

Battery: stiff V_bat. The output cap voltage tracks V_bat once the converter pushes current; cap models the
high-frequency ripple. The simplest version is V_out := V_bat + ripple_from_I_L_minus_I_bat, with I_bat = (V_out -
V_bat) / R_bat_small (small R for stiffness; lets us avoid a hard clamp that creates discontinuities).

Make sense? Anything you'd model differently?

3. Coupling to the firmware

ADC_VConv is selected via sensor.conf:
vin_adc=vconv    vin_ch=0
iin_adc=vconv    iin_ch=1
iout_adc=vconv   iout_ch=2
vout_adc=vconv   vout_ch=3
ntc_adc=vconv    ntc_ch=4    # returns a constant temp

ADC_VConv::getSample() reads the channel field, returns the current model state for that channel. The sampling
tick comes from the existing PeriodicTimer pattern in ADC_Fake. On each tick:

1. Read pwmCtrl, pwmRect, pwmMax, pwmFrequency from a SynchronousConverter* injected at init.
2. Advance the model by N = pwm_freq / adc_freq PWM cycles (typically 10-15).
3. Notify the task waiter — the RT loop picks up the sample.

ADC_VConv owns a VirtualConverter instance and exposes it via extern for test access. init() takes a reference to
the SynchronousConverter so it can read PWM counts each tick.

4. Configuration

New vconv.conf (lives in board configs alongside coil.conf etc.):
# PV source
pv_isc=8.0
pv_voc=40.0
pv_k=2.0          # diode ideality * Vt, controls MPP sharpness

# Battery sink
v_bat=28.0
r_bat=0.05        # small R for stiffness

# Passives (L comes from coil.conf::L0)
plant_c_in=470e-6
plant_c_out=470e-6

Plus a vconv console command for runtime tweaks (mirrors set-config style):
vconv pv <isc> <voc> [k]
vconv bat <v>
vconv state         # dump V_in, V_out, I_L, MPP

This is the test-control surface — unit tests use it via the MQTT/UART command path; bench debugging uses it
interactively.

5. Existing software question

For the model itself: don't pull in external libraries. SPICE/PLECS/ngspice would all bring more weight than the
~200 LOC analytic model and they don't fit on-target. Wokwi's existing custom-chip API could eventually host this
same model out-of-MCU but that's a much bigger integration.

For reference / cross-check: keep the existing etc/virtual-converter/ directory and consider adding a host-side
Python copy of the same equations to validate the on-target math (the way etc/measure_coil.py was the reference
before it was ported on-device).

6. Buck vs boost recommendation — concrete

Buck first. Boost reuses identical structure with sides swapped (boost: V_out is the high side, I_L ramps during
LS-on, off during HS-on). The model class can be templated on topo from day one (isBoost flag mirrors buck.h), but
I'd leave the boost branch as a stub that asserts until buck is solid.
  
---
Difficulties to flag

1. State quantization at scale changes. When pwmCtrl jumps (e.g. +50), the model needs to converge over multiple
   PWM cycles, not instantly. The 10-15 cycles per ADC sample handles this naturally.
2. DCM/CCM boundary stiffness. At the transition, I_L_end ≈ 0. Small numerical errors can flip mode every cycle.
   Use the same hysteresis as buck.h::DcmEnterRippleRatio / DcmExitRippleRatio so the model and the firmware agree.
3. Vout clamp vs stiff battery. Stiff voltage clamp on V_out creates δ-functions in I_bat. The small R_bat
   workaround is the standard fix.
4. Numerical drift. The exponential PV term can push the cap dynamics unstable at large Δt. Bound check on V_in ∈
   [0, Voc + margin] each step is cheap insurance.
5. The current firmware tests assume specific ADC behaviors (sinusoidal ramp etc.). Switching wokwi_mock from
   ADC_Fake → ADC_VConv will fail tests that depend on those specific signals. Plan a flag day or keep both backends
   selectable.

  ---
That's the sketch. Three things to confirm before I write it up as a spec doc:

1. Module split src/sim/ (pure model) vs src/adc/vconv.h (firmware glue) OK?
2. Coupling via SynchronousConverter& (read PWM counts directly) OK?
3. vconv console command + vconv.conf as the parameterization surface OK?

❯ * wouldn't it be better to wrap the pwmDriver instead of samplint the values from the memory address?  


# Test
write a spec for vconv-tests:

## Test "FPWM Sweep"
In forced PWM Mode `pwmCtrl + pwmRect == pwmMax` holds true, so `t_off` is zero, either one switch conducts.
The convert is in DCM and the voltage ratio equals HS duty cycle: `Vout/Vin == pwmCtrl/pwmMax` .

Test procedure: Sweep `pwmCtrl` from 0 to `pwmMax`, setting `pwmCtrl = pwmMax - pwmRect`. The voltage ratio `pwmCtrl/pwmMax`.



# Tests
Your job ist to find flaws in an mppt charge controller.
Before looking at the firmware code, think about what load stress solar charger are exposed to.

* First OV,OC,OP, OT (over-temperature) protection and power-derating.
* Short circuit (Vbat=0 ) => charge should shutdown after less than 3 seconds, current stays below `iout_short`*1.05
* bat impedance suddenly jumps to 1e9 (disconnect) => voltage transient contraint: 2 ms no higher than `vmax_` ...
