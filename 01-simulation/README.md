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

## Physics model (Python and MATLAB)

Run `python3 wasp_sim.py` (altitude) or `python3 wasp_attitude.py` (roll and pitch). The `.m` files are
line-for-line MATLAB versions (R2018b or newer) and give the same plots.

- `wasp_sim`: weight budget, motor and propeller model, altitude controller and step tests.
  Result: 164.9 g (189.6 g with contingency), thrust-to-weight 3.3 to 5.6, hover about 4.2 min,
  a 1 m step settles inside +-2 cm in about 1.4 to 1.6 s with a laser sensor, a barometer alone cannot.
- `wasp_attitude`: roll and pitch with four tests (nominal, battery off-centre, and so on).
  Lesson: keep the centre of gravity within 5 mm. 20 mm off-centre starts at 48 degrees of tilt.

Weakest assumptions, to be replaced with measurements: the thrust coefficient (thrust stand),
the moment of inertia (Fusion 360, Inspect, then Properties) and the accelerometer seeing tilt.

## Archived: 165 g brushless study (22 Nov 2026)

`wasp_sim.py/.m` and `wasp_attitude.py/.m` model the first design (3S, 1505 motors, 165 g). We changed to a 50 g brushed micro quad, see `docs/decisions/0001-micro-brushed-airframe.md`. The method carries over; the numbers do not. `wasp_budget.py` is the weight, thrust and centre-of-gravity budget for the micro design. The next model uses measured thrust from the bench test.
