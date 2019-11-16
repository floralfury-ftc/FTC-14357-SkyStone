package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Default TeleOp Move for mecanum drive
 */
@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class MecanumOpMode extends OpMode {

    /** Robot representation */
    private RobotHardware robot;

    /** Default drive power */
    private final double DRIVE_POWER = 0.5;

    /** Total runtime to output for debugging */
    private ElapsedTime runtime;

    /** Controller inputs */
    /*
    private boolean dPadUp;
    private boolean dPadDown;
    */
    private boolean buttonAChanged;
    private boolean armOn;

    /**
     * Initialize robot
     */
    @Override
    public void init() {
        runtime = new ElapsedTime();

        // Instantiate robot with hardware specifications
        robot = new RobotHardware(hardwareMap);

        buttonAChanged = false;
        armOn = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Main control loop, handle controller input and motor speeds
     */
    @Override
    public void loop() {
        // Convert joysticks to desired motion.
        Mecanum.Motion motion = Mecanum.joystickToMotion(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Convert desired motion to wheel powers, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        robot.frontLeftDrive.setPower(-wheels.frontLeft);
        robot.frontRightDrive.setPower(-wheels.frontRight);
        robot.backLeftDrive.setPower(-wheels.backLeft);
        robot.backRightDrive.setPower(-wheels.backRight);

        if (gamepad1.a && !buttonAChanged) {
            robot.armServo.setPosition(armOn ? 1 : 0);
            armOn = !armOn;
            buttonAChanged = true;
        } else if (!gamepad1.a) buttonAChanged = false;

        // Provide debugging information
        telemetry.addData("Status", "Runtime: %s", runtime.toString());
        telemetry.addData("Drive Train", "Front Left: %.2f\nFront Right: %.2f\nBack Left: %.2f\nBack Rigth: %.2f", wheels.frontLeft, wheels.frontRight, wheels.backLeft, wheels.backRight);
        telemetry.update();
    }
}
