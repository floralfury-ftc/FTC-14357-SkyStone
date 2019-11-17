package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous OpMode
 */
@Autonomous(name="Auton", group="Autonomous")
public class AutonDriveOpMode extends AutonOpMode {

    /**
     * Run custom movements for this mode
     */
    @Override
    public void runOpMode() {
        // Initial autonomous config/movement
        super.runOpMode();

        moveForward(1, .5);
    }

    @Override
    void moveForward(double time, double maxPower) {
        double endTime = getRuntime() + time;

        while (getRuntime() < endTime) {
            robot.setLeftPower(maxPower);
            robot.setRightPower(maxPower * 1.05);
        }
        robot.setDrivePower(0);
        sleep(100);
    }

    @Override
    void moveBackward(double time, double maxPower) {
        double endTime = getRuntime() + time;

        while (getRuntime() < endTime) {
            robot.setLeftPower(-maxPower);
            robot.setRightPower(-maxPower * 1.05);
        }
        robot.setDrivePower(0);
        sleep(100);
    }

    @Override
    void strafeLeft(double time, double maxPower) {
        double endTime = getRuntime() + time;

        robot.toggleLeftStrafe();

        while (getRuntime() < endTime) {
            robot.setLeftPower(-maxPower);
            robot.setRightPower(-maxPower);
        }

        robot.toggleLeftStrafe();
        robot.setDrivePower(0);
        sleep(100);
    }

    @Override
    void strafeRight(double time, double maxPower) {
        double endTime = getRuntime() + time;

        robot.toggleRightStrafe();

        while (getRuntime() < endTime) {
            robot.setLeftPower(-maxPower);
            robot.setRightPower(-maxPower);
        }

        robot.toggleRightStrafe();
        robot.setDrivePower(0);
        sleep(100);
    }
}
