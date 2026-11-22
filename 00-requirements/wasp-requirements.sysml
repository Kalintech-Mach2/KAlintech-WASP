// WASP requirements, written in SysML v2 textual notation.
// Lines starting with // are comments.
// Updated 22 Nov 2026: design changed from a 165 g brushless quad to a ~50 g brushed micro quad
// (1S, ESP32-S3, 8520 coreless motors). See docs/decisions/0001-micro-brushed-airframe.md.
// Numbers marked "estimate" come from wasp_budget.py and must be replaced by bench measurements.

package WASP {

    // ---------- PART 1: requirements ----------

    requirement <R1> totalMass {
        doc /* Total mass shall not exceed 100 g. Design target 52 g, up to 60 g with 15% contingency. */
        // Estimate: 51.6 g budget, 59.3 g with contingency, margin about 40 g.
        // Under 100 g no UK Flyer ID or Operator ID is needed (the Drone Code still applies).
    }

    requirement <R2> thrustToWeight {
        doc /* Thrust-to-weight ratio shall be at least 2:1. */
        // Estimate: 1.55 (20 g per motor) to 2.33 (30 g per motor), 1.94 expected. THE MAIN RISK.
        // Hover throttle about 66 to 80 percent, so there is little spare thrust. Measure on the kitchen scale.
        // If it fails: 260 mAh battery (saves about 4 g) or smaller props, or lighter frame.
    }

    requirement <R3> hoverEndurance {
        doc /* Hover endurance shall be at least 3 minutes. */
        // Estimate: 4.5 to 7.5 min on a 450 mAh pack (80 percent usable).
    }

    requirement <R4> gestureToCommand {
        doc /* Gesture-to-command delay shall not exceed 200 ms. */
        // Measured after the deliberate hold time (0.3 to 0.8 s per command, see R11), not including it.
    }

    requirement <R5> failsafe {
        doc /* If the heartbeat stops the drone shall hold level and altitude after 0.3 s, descend at 0.2 m/s
               after 1.0 s, and switch the motors off after 3.0 s or on touchdown. */
        // Replaces the old "disarm within 0.5 s" rule. WiFi gaps of 100 to 400 ms are normal on an ESP32,
        // so cutting the motors at 0.5 s would drop the drone out of the air on a hiccup.
        // Tested in wasp_link.py (simulated drone) and test_link.py.
    }

    requirement <R6> noCamera {
        doc /* The drone shall carry no camera. */
        // The CAA does not define "camera"; an optical-flow sensor is probably not one, but it is not confirmed.
    }

    requirement <R7> altitudeStep {
        doc /* From rest on the ground, the 0.5 m take-off command shall be reached and held within +-3 cm,
               and the 0.10 m hover command held within +-3 cm. */
        // Target only. Needs the distance sensor (R9) and a re-run of the model with measured thrust.
        // Ground effect below one prop diameter (65 mm) makes 0.10 m harder than 0.5 m.
    }

    requirement <R8> currentLimits {
        doc /* Battery current shall not exceed 13.5 A (450 mAh at 30C). Each motor shall stay under 3 A in flight,
               and each motor switch shall be rated for more than the stalled-motor current. */
        // 8520: about 1.6 to 1.8 A at full load, stall above 5 A. Total about 7 A.
        // Use AO3400A (5.7 A) not SI2302 (about 2.6 to 2.9 A) so a jammed prop does not kill the switch.
    }

    requirement <R9> altitudeSensing {
        doc /* The altitude measurement shall have noise of 1 cm or better (1 sigma) at 25 Hz or faster. */
        // VL53L1X, short mode, up to 50 Hz, minimum range 4 cm. A barometer is useless at 10 cm.
    }

    requirement <R10> centreOfGravity {
        doc /* The centre of gravity shall lie within 3 mm (horizontally) of the geometric centre of the four motors. */
        // 70 mm motor spacing: 3 mm off means the front and back motor pairs differ by about 9 percent
        // of thrust just to hover level, and 10 mm means 29 percent, which uses up most of the spare thrust.
        // Hold the battery in a printed cradle so it cannot slide.
    }

    requirement <R11> gestureConfidence {
        doc /* A gesture command shall only be sent when its match confidence is at least that command's
               minimum (kill 40%, takeoff 70%, all others 60%) for its whole hold time
               (kill 0.3 s, takeoff 0.8 s, others 0.4 to 0.8 s). Body gestures override hand gestures,
               and kill overrides everything. */
        // Test data: 9 gestures, zero wrong-gesture triggers across all tolerance settings,
        // and body idle never came near kill or hover. Hand idle not recorded.
        // Values live in 04-tracker/gesture-studio/wasp-command-table.json.
    }

    requirement <R12> killLatency {
        doc /* KILL shall switch all motors off within 50 ms of the drone receiving it, handled in the
               receive routine and not in the control loop. */
        // The laptop sends KILL five times. Use ESP-NOW for the link if WiFi jitter makes this unreliable.
    }

    requirement <R13> armInterlock {
        doc /* The drone shall only arm when the heartbeat is present, the motors are idle, the drone is level
               (within 10 degrees) and the battery is at least 3.7 V. Take-off shall only be accepted when armed. */
        // Heartbeat, mode and take-off rules are tested in the simulated drone. Level and battery checks need the real hardware.
    }


    // ---------- PART 2: the parts ----------

    part def Drone;
    part def Tracker;

    part wasp : Drone {
        part flightController;   // ESP32-S3 SuperMini
        part imu;                // MPU-6050 (GY-521)
        part altitudeSensor;     // VL53L1X
        part motorDrivers;       // 4x AO3400A MOSFET + 4x SS14 diode
        part motors;             // 4x 8520 coreless brushed
        part propellers;         // 4x 65 mm
        part battery;            // 1S 450 mAh LiHV
        part frame;              // 3D printed, with battery cradle
    }

    // The separate unit that watches your hand and sends commands.
    part tracker : Tracker;      // laptop webcam, wasp_link.py relay, optional ESP-NOW USB dongle


    // ---------- PART 3: which part satisfies which requirement ----------

    satisfy R1 by wasp;
    satisfy R2 by wasp.motors;            // propellers, battery mass and frame mass matter too
    satisfy R3 by wasp.battery;
    satisfy R4 by tracker;
    satisfy R5 by wasp.flightController;
    satisfy R6 by wasp;
    satisfy R7 by wasp.flightController;  // the controller, using the altitude sensor and IMU
    satisfy R8 by wasp.motorDrivers;
    satisfy R9 by wasp.altitudeSensor;
    satisfy R10 by wasp.frame;            // frame design and battery cradle
    satisfy R11 by tracker;
    satisfy R12 by wasp.flightController;
    satisfy R13 by wasp.flightController;
}
