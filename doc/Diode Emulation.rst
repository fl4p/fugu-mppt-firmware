===============
Diode Emulation
===============

We can leave the Low-Side switch (LS, aka *sync-FET*, *synchronous rectifier*) off and the coil discharge current will
flow through the LS MOSFET´s body diode.
The buck converter then operates in non-synchronous mode, which is much easier to implement with the cost of lower conversion efficiency.

In CCM (Continuous conduction mode), switching the LS is trivial. DC bias current is higher than half the ripple current, so we just keep LS on while
HS is off. Notice that the LS body diode conducts during driver dead times.
The duty cycle D equals conversion ratio M (Vo/Vi).

In DCM, e.g. during light load conditions, we must take care about LS switching times.
Conversion ratio is:

.. math::
    M_DCM = \frac{2}{ 1 + \sqrt{1+4R_e/R} }

Where R is the load resistance and Re is the effective input resistance of the converter:

.. math::
    R_e = \frac{2L\cdot f_sw}{ D^2 }

For more details refer to Fundamentals of Power Electronics, Third Edition, pages 145, 597.
Here it is sufficient to understand that in DCM conversion ratio M does not equal duty cycle D.

-------------------------------------
Inductor Current Zero Cross Detection
-------------------------------------

We can use a current sensor with zero cross detection (ZCD) to disable the LS as soon as coil current becomes zero.
A digital ZCD implementation requires an ADC sampling rate much higher than the switching frequency for accurate timing.

An analog ZCD works with a fast comparator, whose output can be fed into the half bridge driver disable input.

--------------------
Sensor-less approach
--------------------
In a sensor-less approach we model coil current over time and shut the LS off when we expect the coil current to be near zero.
Turning the LS off too early will increase power loss of the LS body diode. Turning off to late puts the converter into
Forced-PWM mode with reverse current flow, which decreases efficiency as well.

First, we need to check if converter operating condition requires DCM.
The converter is in DCM if half the ripple current is larger than dc output current:

.. math::
    \frac{\Delta I_L}{2} > I_o

We compute the inductor ripple current:

.. math::
    \Delta I_L = \frac{V_o}{f_sw \cdot L} \cdot (1 - V_o/V_i)


For powder core materials, permeability drops with increasing dc bias current. We neglect frequency and temperature dependency, as it is usually low.
With dc coil current, number of turns N and magnetic path length l_e we compute the dc magnetization force:

.. math::
    H_dc =  \frac{N}{l_e} \cdot I_o

With the value of the H-field we can compute the permeability and inductivity drop with the model from the materials's
datasheet. With the DC-saturated inductivity value we compute ripple current and decide if the converter is in DCM and compute
LS on-time as follows.


During HS on-time (:math:`0<t<t_{on,HS}`), coil current rises:

.. math::
    I_L(t) = \frac{1}{L} \int_{0}^{t} V_i-V_o \,dt

We calculate the peak inductor current:

.. math::
    I_{L,max} = I_L(t_{on,HS}) = \frac{1}{L} (V_i-V_o) \cdot t_{on,HS}

During LS conduction  (:math:`t_{on,HS}<t<t_{on,HS}+t_{on,LS}`), the inductor current falls:

.. math::
    I_L(t) = I_{L,max} - \frac{1}{L} (V_o) \cdot (t- t_{on,HS})

Now we want to find :math:`t_{on,LS}` when the current becomes zero for given :math:`t_{on,HS}, V_i, V_o`:

.. math::
     0 \stackrel{!}{=}  I_{L,max} - \frac{1}{L} (V_o) \cdot t_{on,LS}

Which results:

.. math::
    t_{on,LS} = t_{on,HS} \cdot (\frac{V_i}{V_o} - 1) = t_{on,HS} \cdot (\frac{1}{M} - 1)

.. math::
    t_{on,LS} = \frac{D}{f_sw} \cdot (\frac{1}{M} - 1)

Notice that if we set M = D, as in CCM, this equation becomes equal to the CCM case:

.. math::
    t_{on,LS,CCM} = \frac{1-D}{f_sw}


Takeaways

* In CCM low-side switching time is simply (1-D)/f_sw
* whether converter operates in CCM / DCM depends on load conditions and (dc-biased) inductivity
* In DCM low-side switching time depends on conversion ratio M and duty cycle D
* Switching LS too long causes reverse coil current and might turn the buck converter into a (reversed) boost converter


-------
Boost Converter
-------


.. math::
    M_CCM = \frac{1}{1-D}

.. math::
    t_{on,HS} = t_{on,LS} \cdot \frac{1}{M - 1}

.. math::
    t_{on,HS} = \frac{D}{f_sw} \cdot \frac{1}{M - 1}



References

* Fundamentals of Power Electronics, chapters 5 and 15