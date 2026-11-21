# Simulation

## drone_v1.m: 1-D hover with PID control

Simulates only the drone's height, so no tilting, no wind and no drama. The controller holds it
at 1 m using hover thrust (m*g) plus three terms:

- **Kp (push):** the further it is from the target, the harder it pushes
- **Kd (braking):** if it is climbing fast, it eases off so it does not overshoot
- **Ki (memory):** the longer it has been slightly wrong, the harder it pushes. This one holds grudges

### What we found

- With Ki = 0 and a perfect model, the drone settles exactly on 1 m.
- With Ki = 1 the grudge builds up while the motors are flat out on the way up (this is called
  integral windup), so it takes a while to forgive and settle.
- When the controller believes the drone is 90 g but it is really 110 g, Ki = 0 leaves it
  about 2.5 cm low forever. With Ki = 1 it works out the missing thrust by itself and reaches 1 m.
  This is the entire reason flight controllers have an integral term.

### Next

Rebuild it in Simulink, then add motor lag (real motors are not instant) and a tilt axis.
