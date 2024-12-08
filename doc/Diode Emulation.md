# Diode Emulation


We can leave the Low-Side switch (LS, aka *sync-FET*, *synchronous
rectifier*) off and the coil discharge current will flow through the LS
MOSFET´s body diode. The buck converter then operates in non-synchronous
mode, which is much easier to implement with the cost of lower
conversion efficiency.

In CCM (Continuous conduction mode), switching the LS is trivial. DC
bias current is higher than half the ripple current, so we just keep LS
on while HS is off. Notice that the LS body diode conducts during driver
dead times. The inductor is permanently energized and the duty cycle D equals conversion ratio M (=Vo/Vi).

In DCM, e.g. during light load conditions, we must take care about LS
switching times. Conversion ratio is:

$$M_{DCM} = \frac{2}{ 1 + \sqrt{1+4R_e/R} }$$

Where R is the load resistance and Re is the effective input resistance
of the converter:

$$R_e = \frac{2L\cdot f_{sw}}{ D^2 }$$

For more details refer to Fundamentals of Power Electronics, Third
Edition, pages 145, 597. Here it is sufficient to understand that in DCM
conversion ratio M does not equal duty cycle D.

## Inductor Current Zero Cross Detection

We can use a current sensor with zero cross detection (ZCD) to disable
the LS as soon as coil current becomes zero. A digital ZCD
implementation requires an ADC sampling rate much higher than the
switching frequency for accurate timing.

An analog ZCD works with a fast comparator, whose output can be fed into
the half bridge driver to `DIS` (or `SD`, `EN*`) input.

The advantage of ZCD is that it implements peak current limiting. This
prevent excessive currents when inductor core starts to saturate and the
ac ripple current waveform becomes spiky. Once the current threshold is
reached, we shut-down the ctrl switch and after the chosen dead-time
enable the sync switch. The analog signal path garantues very fast
shut-down in over-load and short-circuit conditions.

<https://www.monolithicpower.com/en/learning/resources/power-losses-in-buck-converters-and-how-to-increase-efficiency>

## Sensor-less approach

In a sensor-less approach we model coil current over time and shut the
LS off when we expect the coil current to be near zero. Turning the LS
off too early will increase power loss of the LS body diode. Turning off
to late puts the converter into Forced-PWM mode with reverse current
flow, which decreases efficiency as well.

### DCM or CCM

First, we need to check if converter operating condition requires DCM.
The converter is in DCM if half the ripple current is larger than dc
output current:

$$\frac{\Delta I_L}{2} > I_o$$

We compute the inductor ripple current:

$$\Delta I_L = \frac{V_o}{f_{sw} \cdot L(I_o)} \cdot (1 - V_o/V_i)$$

Notice that inductivity L here depends on I_o. For powder core
materials, permeability drops with increasing dc bias current. We
neglect frequency and temperature dependency, as it is usually low. With
dc coil current, number of turns N and magnetic path length l_e we
compute the dc magnetization force:

$$H_{dc} = \frac{N}{l_e} \cdot I_o$$

With the value of the H-field we can compute the permeability and
inductivity drop with the model from the materials\'s datasheet
( $\\%\mu _i( H )$ ).

$$L(I_o) = \frac{\\%\mu _i(H_dc)}{\mu _i} \cdot L_0$$

With the DC-biased inductivity value we compute ripple current and
decide if the converter is in DCM.

Besides inductivity value L, this approach needs the number of winding
turns, the magnet path length of the core and the dc bias model of the
core material. Simulations show that there is a rather small
operating range where the converter would operate in DCM with L(I_o),
and in CCM with L0. For reduced complexity of the implementation, we can
just assume a fixed inductivity drop of 5%. A simplified model would assume a linear L(I_o), which is expected to work
with well designed converters where ripple current is around 0.3 of full load dc current.
An analytic inference still needs to be done.

If we find the converter to be in DCM, we compute LS on-time as follows.

### DCM Rectifier timing

During HS on-time ($0<t<t_{on,HS}$), coil current rises:

$$I_L(t) = \frac{1}{L} \int_{0}^{t} V_i-V_o \,dt$$

We calculate the peak inductor current:

$$I_{L,max} = I_L(t_{on,HS}) = \frac{1}{L} (V_i-V_o) \cdot t_{on,HS}$$

During LS conduction ($t_{on,HS}<t<t_{on,HS}+t_{on,LS}$), the inductor
current falls:

$$I_L(t) = I_{L,max} - \frac{1}{L} (V_o) \cdot (t- t_{on,HS})$$

Now we want to find $t_{on,LS}$ when the current becomes zero for given
$t_{on,HS}, V_i, V_o$:

$$0 \stackrel{!}{=} I_{L,max} - \frac{1}{L} (V_o) \cdot t_{on,LS}$$

Which results:

$$t_{on,LS} = t_{on,HS} \cdot (\frac{V_i}{V_o} - 1) = t_{on,HS} \cdot (\frac{1}{M} - 1)$$

$$t_{on,LS} = \frac{D}{f_sw} \cdot (\frac{1}{M} - 1)$$

Notice that if we set M = D, as in CCM, this equation becomes equal to
the CCM case:

$$t_{on,LS,CCM} = \frac{1-D}{f_sw}$$

Takeaways

- In CCM low-side switching time is simply (1-D)/f_sw
- whether converter operates in CCM / DCM depends on load conditions
  and (dc-biased) inductivity
- In DCM low-side switching time depends on conversion ratio M and
  duty cycle D
- Switching LS too long causes reverse coil current and might turn the
  buck converter into a (reversed) boost converter

### Error Considerations

The converter measures V_in and V_out with an ADC. Noise, temperature
drift and non-linearity cause voltage errors. This affects the value for
M and finally the rectification on time.

Let assume two extreme cases for the voltage measurements: V_in is +1%
of the actual value, V_out -1%: we will get an M which is around -2%
below the actual value. Since rectification time is reciprocal to M,
this will cause a +4% error rectification on time at D=0.5. If we double
the voltage error, we get approximately double the error for
rectification time. Longer rectification time will cause reverse current
flow and additional loss (it can reduce ripple voltage, refer to forced
PWM or FPWM)

If we measure V_out with -1% error and V_out +1%, the rectification time
will be 4% too short.

# Boost Converter

$$M_{CCM} = \frac{1}{1-D}$$

$$t_{on,HS} = t_{on,LS} \cdot \frac{1}{M - 1}$$

$$t_{on,HS} = \frac{D}{f_{sw}} \cdot \frac{1}{M - 1}$$

References

- Fundamentals of Power Electronics, chapters 5 and 15
