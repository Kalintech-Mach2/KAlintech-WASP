// WASP Phase 1 requirements, written in SysML v2 textual notation.
// Lines starting with // are comments.

package WASP {

    // ---------- PART 1: requirements ----------

    requirement <R1> totalMass {
        doc /* Total mass shall not exceed 100 g. */
    }

    requirement <R2> thrustToWeight {
        doc /* Thrust-to-weight ratio shall be at least 3:1. */
    }

    requirement <R3> hoverEndurance {
        doc /* Hover endurance shall be at least 4 minutes. */
    }

    requirement <R4> gestureToCommand {
        doc /* Gesture-to-command delay shall not exceed 200 ms. */
    }

    requirement <R5> failsafe {
        doc /* Motors shall disarm within 0.5 s of losing the command signal. */
    }

    requirement <R6> noCamera {
        doc /* The drone shall carry no camera. */
    }


    // ---------- PART 2: the parts ----------

    part def Drone;
    part def Tracker;

    part wasp : Drone {
        part flightController;   // ESP32-S3
        part imu;                // MPU-6050
        part esc;                // 4-in-1 DShot ESC
        part motors;             // 4x 1104 7500KV
        part propellers;         // 3-inch, 4x
        part battery;            // 2S LiPo
        part buckRegulator;      // 5 V supply for the flight controller
    }

    // The separate unit that watches your hand and sends commands.
    part tracker : Tracker;      // laptop webcam first, Pi Zero 2 W later


    // ---------- PART 3: which part satisfies which requirement ----------

    satisfy R1 by wasp;
    satisfy R2 by wasp.motors;            // propellers matter too, see note below
    satisfy R3 by wasp.battery;
    satisfy R4 by tracker;
    satisfy R5 by wasp.flightController;
    satisfy R6 by wasp;

    // Note: real requirements often have more than one part involved.
    // R2 depends on motors AND propellers, and R3 depends on the battery AND the weight.
    // Later we can split these into separate lines to show that.
}
