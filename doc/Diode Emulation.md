A non-synchronous buck converter looks like this:

Real diodes always have a voltage drop of a couple of 100mV up to more than 1V.
We can reduce this loss by replacing the diode with a switch:

SW1 is controlled by our PWM signal. SW1 and SW2 must never be on at the same time.
If SW2 is turned on for too long, and there is a battery connected at the output, reverse current will flow
and the buck will turn into a reverse boost.
TI refers this as forced PWM (TODO pdf link).

If we re-arrange the components, we can see that this is similar to boost converter:

Timing SW2.

To approach this problem, lets have a look at the coil current.


