FPWM (forced PWM, inductor current crosses zero)

- less ripple but lower eff. when light load
- constant frequency
- increased loss during low-load
- also referred as CCM (CCM / DCM as a control scheme vs CCM/DCM operating condition)

DCM (discontinous conduction mode)
- reduce on-time to avoid negative inductor currents during light load conditions (low D)

![img_1.png](img_1.png)
https://www.ti.com/lit/an/slyt358/slyt358.pdf

PFM (pulse frequency modulation )

- pulse skipping, burts
- variable frequency
- higher ripple
![img.png](img.png)
- https://www.ti.com/lit/an/slva236a/slva236a.pdf?ts=1730240504335


COT

- constant on-time
- nearly constant frequency
- very good load transient response
  https://www.monolithicpower.com/en/learning/resources/advantages-of-constant-on-time-control-in-dc-dc-converters
![img_2.png](img_2.png)