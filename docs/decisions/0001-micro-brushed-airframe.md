# 0001: Change from a 165 g brushless quad to a ~50 g brushed micro quad

Date: 22 Nov 2026. Status: accepted.

## Context
The first design was a 165 g quad: 4x 1505 brushless motors, a 4-in-1 ESC, 3S 650 mAh battery, 3-inch props.
A pre-purchase check found:
- The ESC has no 5 V output, so the flight controller needed an extra buck regulator.
- A 3S charger, LiPo bag, smoke stopper and more were needed before the first test.
- Nudge and follow commands need horizontal position sensing, which the planned sensors do not give.
- A 0.5 s "no heartbeat = motors off" rule would drop the drone on an ordinary WiFi gap.
- It is a heavy 3S drone to fly indoors in front of a laptop camera, and over 100 g it needs a Flyer ID.

## Options
A. Keep the 165 g brushless design. Fast (thrust-to-weight about 5:1), but the most expensive and the riskiest first build.
B. A ~50 g brushed micro quad: 1S battery, 4x 8520 coreless motors on 65 mm props, one MOSFET per motor, ESP32-S3.
   The same recipe is used by ESP-FLY, LiteWing and ESP-Drone. About 77 GBP for everything.
C. Buy the ESP-FLY kit (about 47 GBP plus shipping). Fewest parts, but little design work for us.

## Decision
Option B.

## Consequences
- Under 100 g: no CAA Flyer ID or Operator ID needed. The Drone Code still applies.
- Thrust-to-weight is only about 1.5 to 2.3 (estimate), the main technical risk. Measure thrust early.
- Agile indoors, not fast. Flight time about 4.5 to 7.5 min (estimate).
- No horizontal position hold, so nudge and follow wait for a later optical-flow upgrade.
- Heartbeat failsafe changed to hold (0.3 s), descend (1.0 s), motors off (3.0 s). Requirements R5 and R12 and R13.
- The gesture tracker, command table and simulated drone carry over unchanged.
- The 165 g design stays on the roadmap as version 2.

## How we will check it
1. Weigh every part (budget in 01-simulation/wasp_budget.py).
2. Measure thrust and current per motor on a kitchen scale.
3. Fly existing ESP-FC firmware first to prove the hardware, then our own firmware.
4. Test the failsafe stages on the bench with the props off.
