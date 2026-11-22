// WASP Phase 1 requirements, written in SysML v2 textual notation.
// Lines starting with // are comments.
// Updated 22 Nov 2026: matches the real parts list and the physics model, adds R10 (centre of gravity)
// and R11 (gesture confidence).

package WASP {

    // ---------- PART 1: requirements ----------

    requirement <R1> totalMass {
        doc /* Total mass shall not exceed 250 g. Design target 165 g, with contingency up to 190 g. */
        // Model: budget 164.9 g, 189.6 g with 15% contingency. Margin to limit: about 60 g.
        // Under 250 g with no camera, a UK Operator ID is optional (a Flyer ID is still required from 100 g).
    }

    requirement <R2> thrustToWeight {
        doc /* Thrust-to-weight ratio shall be at least 3:1. */
        // Model: 3.3 (pessimistic) to 5.6 (optimistic), 4.9 expected. Confirm on thrust stand.
    }

    requirement <R3> hoverEndurance {
        doc /* Hover endurance shall be at least 4 minutes. */
        // Model: 3.9 to 4.9 min. Borderline at the high-thrust end, so verify by test.
    }

    requirement <R4> gestureToCommand {
        doc /* Gesture-to-command delay shall not exceed 200 ms. */
        // Measured after the deliberate hold time (0.3 to 0.8 s per command, see R11), not including it.
    }

    requirement <R5> failsafe {
        doc /* Motors shall disarm within 0.5 s of losing the command signal. */
    }

    requirement <R6> noCamera {
        doc /* The drone shall carry no camera. */
        // Check with the CAA before adding any optical-flow or depth sensor.
    }

    requirement <R7> altitudeStep {
        doc /* From rest on the ground, a 1 m altitude command shall be reached and held
               within +-2 cm (0.02 m) with no overshoot above +-2 cm. */
        // Model: 0.8 s to 90% of target, settles inside +-2 cm in about 1.4 to 1.6 s,
        // 100% of 300 randomised runs hold +-2 cm. Needs a laser ranging sensor (see R9).
        // Physical limit for 0 to 1 m and stop is 0.51 s, so this is not the limit.
    }

    requirement <R8> currentLimits {
        doc /* Total battery current shall not exceed 48.75 A (75C x 0.65 Ah),
               and each motor shall not exceed 18 A. */
        // Model: 14.8 to 32.6 A total at full throttle, under 9 A per motor.
        // Still cap maximum throttle in firmware as a safety margin.
    }

    requirement <R9> altitudeSensing {
        doc /* The altitude measurement shall have noise of 1 cm or better (1 sigma) at 50 Hz or faster,
               because a barometer alone cannot meet R7. */
        // Model: barometer-only holds about +-10 to 25 cm. Laser ToF holds +-2 cm.
    }

    requirement <R10> centreOfGravity {
        doc /* The centre of gravity shall lie within 5 mm (horizontally) of the geometric centre of the four motors. */
        // Model (wasp_attitude): a battery 20 mm off-centre starts the drone at about 48 degrees of tilt,
        // and 40 mm leaves it at about 160 degrees and out of control authority.
        // Off-centre mass makes the opposite motors run unequal and one hits 0% throttle.
        // Enclose the battery so it cannot slide.
    }

    requirement <R11> gestureConfidence {
        doc /* A gesture command shall only be sent when its match confidence is at least that command's
               minimum (kill 40%, takeoff 70%, all others 60%) for its whole hold time
               (kill 0.3 s, takeoff 0.8 s, others 0.4 to 0.8 s). Body gestures override hand gestures,
               and kill overrides everything. */
        // Test data: 9 gestures, zero wrong-gesture triggers across all tolerance settings,
        // and 10 s of body idle never came near kill or hover. Hand idle still to be recorded.
        // Values live in 04-tracker/gesture-studio/wasp-command-table.json.
    }


    // ---------- PART 2: the parts ----------

    part def Drone;
    part def Tracker;

    part wasp : Drone {
        part flightController;   // ESP32-S3 SuperMini
        part imu;                // MPU-6050 (GY-521), I2C address 0x68
        part altitudeSensor;     // laser ToF (VL53L1X class), I2C address 0x29. NOT BOUGHT YET.
        part esc;                // JHEMCU EM40A 4-in-1, DShot, Bluejay firmware
        part motors;             // 4x YSIDO 1505 3750KV, 11 g each
        part propellers;         // 4x Gemfan 3016, 3-inch, 3-blade, 1.5 mm shaft, M2 screws
        part battery;            // Tattu 650 mAh 3S 75C, XT30, 59 g
        part buckRegulator;      // 5 V supply for the flight controller
        part frame;              // design target 22 g
    }

    // The separate unit that watches your hand and body and sends commands.
    part tracker : Tracker;      // laptop webcam first (Gesture Studio and Gesture Link), Pi Zero 2 W later


    // ---------- PART 3: which part satisfies which requirement ----------

    satisfy R1 by wasp;
    satisfy R2 by wasp.motors;            // propellers matter too, see note below
    satisfy R3 by wasp.battery;
    satisfy R4 by tracker;
    satisfy R5 by wasp.flightController;
    satisfy R6 by wasp;
    satisfy R7 by wasp.flightController;  // the controller, using the altitude sensor and IMU
    satisfy R8 by wasp.esc;               // throttle cap lives in the flight controller too
    satisfy R9 by wasp.altitudeSensor;
    satisfy R10 by wasp.frame;            // frame design and battery mounting
    satisfy R11 by tracker;

    // Note: real requirements often have more than one part involved.
    // R2 depends on motors AND propellers, and R3 depends on the battery AND the weight.
    // Later we can split these into separate lines to show that.
}
