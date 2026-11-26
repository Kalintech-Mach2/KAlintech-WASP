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

## Micro quad indoor simulation (current, 26 Nov 2026)

Python stand-in for the Simulink model, for the 1S brushed micro build (about 57 g with a 5 g custom PCB).
Run from this folder:

    python3 run_tests.py      # all tests, writes results.json and wasp_sim_fig1.png / wasp_sim_fig2.png
    python3 tune_gains.py     # retunes the attitude loop gains

- `wasp_micro_sim.py`: 6-DOF rigid body, brushed coreless motor model, 1S LiHV battery with sag,
  MPU-6050 complementary filter, VL53L1X altitude hold, angle and rate PID, 500 Hz loop. No wind.
- `run_tests.py`: hover and drift, takeoff, leans, yaw, recovery from tilt and bumps,
  thrust-per-motor against battery-charge sweep, flight time, 600-run Monte Carlo.
- `wasp_budget.py`: weight, thrust and centre-of-gravity budget.

### What we found

- Thrust per motor decides everything. With the 56.6 g build, 25 g or more passes every test
  down to 10 % battery. 18 g only passes on a fresh pack.
- Flight time is about 6 min at 25 g and 56.6 g, but about 3 min at 65 g. Roughly 20 s lost per extra gram.
- Keep the angle limit at 12 to 15 degrees indoors. Real tilt overshoots the command by 20 to 25 %
  because an accelerometer cannot see tilt in flight.
- With no position sensor the drone drifts a median 3.5 m in 20 s. An optical-flow sensor would fix that.

Weakest assumptions, to be replaced with measurements: thrust per motor (25 g assumed, use the kitchen
scale test), motor resistance, battery resistance, prop torque ratio, inertia (Fusion 360), drag.

## Archived: 165 g brushless study (22 Nov 2026)

The first design (3S, 1505 motors, 165 g) had its own simulation files. We changed to a 50 g brushed micro quad,
see `docs/decisions/0001-micro-brushed-airframe.md`. Those files were removed from the current tree but are
still in the git history from before this change.
