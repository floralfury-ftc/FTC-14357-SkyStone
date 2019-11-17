package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous OpMode
 */
@Autonomous(name="Bridge Park", group="Autonomous")
public class BridgeParkOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        // Initial autonomous config/movement
        super.runOpMode();

        moveForward(1, .5);
    }
}
