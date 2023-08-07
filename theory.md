# *RFID LOCALISATION SYSTEM*

## Rangefinding of an EM wave
Given a EM signal with frequency $f$, the phase of this signal in time is computed as:
$$
\phi = \omega \cdot t
$$

where $\omega = 2 \pi f$.

Supposing the signal travel from reader(EM source) to the tag and back to th reader the distance of between them will be:

$$
d = \frac{1}{2} \,c \cdot t_f
$$

with $c$ the speed of light and $t_f$ the time of flight.

So:

$$
d = \frac{1}{2} \,  c \,  \frac{\phi}{\omega} = \frac{1}{2} \,  \frac{c \, \phi}{2 \pi f} = \frac{1}{2} \, \frac{\lambda}{2 \pi} \, (2 n \pi + \Delta \phi) = \frac{1}{2} \, \lambda \,(n + \Delta n)
$$

So if we know $\phi$ whe can compute $\Delta n$:

$$
\Delta n = \phi \,\, mod \,\, 2 \pi
$$



