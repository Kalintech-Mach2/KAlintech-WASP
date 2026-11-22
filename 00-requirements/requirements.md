# Phase 1 requirements

Starting targets. We will refine these as the simulation and the real parts tell us more.

| ID | Requirement | Why |
|---|---|---|
| R1 | Total mass shall not exceed 100 g (design target 52 g) | Under 100 g no CAA Flyer ID or Operator ID is needed. Every gram costs thrust and flight time |
| R2 | Thrust-to-weight ratio shall be at least 2:1 | Estimate is 1.55 to 2.33, so this is the main risk. Measure it on a scale |
| R3 | Hover endurance shall be at least 3 minutes | Estimate 4.5 to 7.5 min on a 450 mAh pack |
| R4 | Gesture-to-command delay shall not exceed 200 ms | Wave, drone reacts. Not wave, wait, wonder |
| R5 | If the heartbeat stops the drone shall hold after 0.3 s, descend after 1.0 s, and switch the motors off after 3.0 s or on touchdown | WiFi gaps of 100 to 400 ms are normal, so cutting the motors at 0.5 s would drop the drone on a hiccup |
| R6 | The drone shall carry no camera | Keeps it light, keeps the paperwork light, and the tracker does the watching |
| R7 | The 0.5 m take-off and the 0.10 m hover shall each be held within +-3 cm | Target. Needs the distance sensor and measured thrust |
| R8 | Battery current shall not exceed 13.5 A, each motor shall stay under 3 A in flight, and each motor switch shall be rated above the stalled-motor current | 8520 draws about 1.7 A at full load and stalls above 5 A. AO3400A is rated 5.7 A |
| R9 | Altitude sensing shall have 1 cm noise or better at 25 Hz or faster | VL53L1X. A barometer is useless at 10 cm |
| R10 | The centre of gravity shall be within 3 mm of the middle of the four motors | At 70 mm motor spacing, 3 mm off means a 9 percent thrust difference just to hover level |
| R11 | A gesture command shall only be sent if its match confidence is at least its minimum (kill 40%, takeoff 70%, others 60%) and it has been held for its hold time | Stops a resting hand or a stray wave becoming a command. See the command table in `04-tracker/gesture-studio/` |
| R12 | KILL shall switch the motors off within 50 ms of the drone receiving it | Handled in the receive routine, not the control loop |
| R13 | The drone shall only arm with a heartbeat present, the drone level and the battery at 3.7 V or more, and take-off shall only be accepted when armed | Stops it arming on a bad link or a flat battery |

The formal version of these is `wasp-requirements.sysml` in this folder.
