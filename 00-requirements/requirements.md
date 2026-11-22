# Phase 1 requirements

Starting targets. We will refine these as the simulation and the real parts tell us more.

| ID | Requirement | Why |
|---|---|---|
| R1 | Total mass shall not exceed 250 g (design target 165 g) | Under 250 g, with no camera, an Operator ID is only optional. Every extra gram needs more thrust, which needs more battery, which weighs more. Physics is a loop |
| R2 | Thrust-to-weight ratio shall be at least 3:1 | Enough thrust to hover and still have some to spare for fixing our mistakes |
| R3 | Hover endurance shall be at least 4 minutes | Long enough to prove it works, short enough that we do not get bored |
| R4 | Gesture-to-command delay shall not exceed 200 ms | Wave, drone reacts. Not wave, wait, wonder |
| R5 | Motors shall disarm within 0.5 s of losing the command signal | A drone that does not know when to stop is a ceiling fan with ambition |
| R6 | The drone shall carry no camera | Keeps it light, keeps the paperwork light, and the tracker does the watching |
| R7 | From rest, a 1 m altitude command shall be reached and held within +-2 cm | Our altitude target. The model settles in about 1.4 to 1.6 s with a laser height sensor |
| R8 | Battery current shall not exceed 48.75 A and each motor shall not exceed 18 A | Battery rating (75C x 0.65 Ah) and motor limit. The model peaks at 14.8 to 32.6 A in total |
| R9 | Altitude sensing shall have 1 cm noise or better at 50 Hz or faster | A barometer alone only holds about +-10 to 25 cm, so we need a laser time-of-flight sensor |
| R10 | The centre of gravity shall be within 5 mm of the middle of the four motors | In the model a battery 20 mm off-centre starts at 48 degrees of tilt, and 40 mm is out of control |
| R11 | A gesture command shall only be sent if its match confidence is at least its minimum (kill 40%, takeoff 70%, others 60%) and it has been held for its hold time | Stops a resting hand or a stray wave becoming a command. See the command table in `04-tracker/gesture-studio/` |

The formal version of these is `wasp-requirements.sysml` in this folder.
