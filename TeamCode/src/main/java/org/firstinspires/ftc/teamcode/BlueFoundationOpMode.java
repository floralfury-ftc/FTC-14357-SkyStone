package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous OpMode
 */
@Autonomous(name="Blue Foundation", group="Autonomous")
public class BlueFoundationOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        // Initial autonomous config/movement
        super.runOpMode();

        strafeRight(1.0, .5);
        moveForward(.38, .5);
        robot.armServo.setPosition(0);
        sleep(2000);
        strafeLeft(3, .5);
        robot.armServo.setPosition(1);
        sleep(200);
        moveBackward(1.2, .5);
    }
}
