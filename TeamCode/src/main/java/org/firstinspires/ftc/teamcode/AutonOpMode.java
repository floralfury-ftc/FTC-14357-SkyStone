package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Abstract Autonomous OpMode which automatically sets up a child OpMode,
 * contains the instantiation of all required objects, the robot configuration,
 * a simple movement API for utilisation by the children, and all movement
 * (speed/PID) constants
 */
public abstract class AutonOpMode extends LinearOpMode {

    /** Robot representation */
    RobotHardware robot;
    //Vision vision;

    /** Tunable speeds */
    final double DRIVE_POWER = 0.5;
    private final double DRIVE_MIN_POWER = 0.1;
    final double STRAFE_POWER = 0.1;
    final double ROTATE_MIN_POWER = 0.1;
    final double ROTATE_MAX_POWER = 0.5;

    /** PID Constants */
    final private double driveKp  = .0002;
    final private double driveKi  = 0;
    final private double driveKd  = 0.0001;
    final private double rotateKp = .005;
    final private double rotateKi = 0;
    final private double rotateKd = 0;

    private double correction;

    /** PID Controlles for drive and rotation */
    final PIDController pidDrive = new PIDController(driveKp, driveKi, driveKd);
    final PIDController pidRotate = new PIDController(rotateKp, rotateKi, rotateKd);

    /**
     * Initial logic for all autonomous OpModes, configure robot hardware
     * and vision, start the hang, and locate the first gold sample
     */
    @Override
    public void runOpMode() {

        // Configure IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating IMU");
        telemetry.update();

        // Calibrate IMU
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //vision = new Vision(hardwareMap, telemetry);

        // Instantiate robot subsystem
        robot = new RobotHardware(hardwareMap, imu);
        //robot.hang.setPower(-1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    /**
     * Rotate a motor to a desired encoder position
     * @param motor Motor to turn
     * @param power Power at which to turn motor
     * @param pos Desired ticks
     */
    void moveToEncoderPosition(DcMotor motor, double power, int pos) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }

    /**
     * Move the robot forward a amount, treating the drive as a tank train
     * @param distance Distance, in inches, to move forward
     * @param maxPower Power at which to move motors
     */
    void moveForward(double distance, double maxPower) {
        distance = distance - 1;

        robot.resetEncoders();
        robot.resetAngle();

        // Set up parameters for driving in a straight line.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, maxPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Values for custom PID implementation
        // TODO: Implement using dedicated PID controller
        double power = 0, error, prevError = 0, p, i = 0, d;

        double leftEncSum  = robot.calculateLeftTicks(distance);
        double rightEncSum = robot.calculateRightTicks(distance);

        telemetry.addData("Status", "Moving forward %f inches\nDesired left encoder position sum = %f\nDesired right encoder position sum = %f", distance, leftEncSum, rightEncSum);
        telemetry.update();

        // Loop until both left and right tick values match their respective sum, thus reaching the target
        while (robot.getLeftTicks() < leftEncSum && robot.getRightTicks() < rightEncSum && opModeIsActive()) {
            // During the first half rotation, speed up slowly to maxSpeed then initiate PID control
            if (-robot.frontLeftDrive.getCurrentPosition() < robot.TICKS_PER_REV / 2 && power < maxPower) {
                power = Math.abs(-robot.frontLeftDrive.getCurrentPosition() / (robot.TICKS_PER_REV / 2) * maxPower) + DRIVE_MIN_POWER;
            } else {
                error = leftEncSum - robot.getLeftTicks();
                p = error;
                i = i + error;
                d = p - prevError;
                power = Range.clip((driveKp * p + driveKi * i + driveKd * d), DRIVE_MIN_POWER, maxPower);
                //telemetry.addData("PID", "Power: %f\nError: %f, prevError: %f\nPID: %f, P: %f, I: %f, D: %f", power, error, prevError, driveKp * p + driveKi * i + driveKd * d, p, i ,d);
                prevError = error;
            }

            // Calculate adjusted correction using IMU and
            // divide by 2 to apply to both sides of the drive train
            correction = pidDrive.performPID(robot.getAngle()) / 2;

            // Set the powers of both sides of the drive train
            robot.setLeftPower(-power + correction);
            robot.setRightPower(-power - correction);

            // Provide debugging data
            telemetry.addData("2 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("3 Global Heading", robot.globalAngle);
            telemetry.addData("4 Correction", correction);
            telemetry.update();
        }

        // Wait for wheels to completely stop
        robot.setDrivePower(0);
        sleep(100);
    }

    /**
     * Move the robot forward a amount, treating the drive as a tank train
     * @param distance Distance, in inches, to move forward
     * @param maxPower Power at which to move motors
     */
    void moveBackward(double distance, double maxPower) {
        distance = distance - 1;

        robot.resetEncoders();
        robot.resetAngle();

        // Set up parameters for driving in a straight line.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, maxPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Values for custom PID implementation
        // TODO: Implement using dedicated PID controller
        double power = 0, error, prevError = 0, p, i = 0, d;

        double leftEncSum  = robot.calculateLeftTicks(distance);
        double rightEncSum = robot.calculateRightTicks(distance);

        telemetry.addData("Status", "Moving forward %f inches\nDesired left encoder position sum = %f\nDesired right encoder position sum = %f", distance, leftEncSum, rightEncSum);

        // Loop until both left and right tick values match their respective sum, thus reaching the target
        while (robot.getLeftTicks() < leftEncSum && robot.getRightTicks() < rightEncSum && opModeIsActive()) {
            // During the first half rotation, speed up slowly to maxSpeed then initiate PID control
            if (robot.frontLeftDrive.getCurrentPosition() < robot.TICKS_PER_REV / 2 && power < maxPower) {
                power = Math.abs(robot.frontLeftDrive.getCurrentPosition() / (robot.TICKS_PER_REV / 2) * maxPower) + DRIVE_MIN_POWER;
            } else {
                error = leftEncSum - robot.getLeftTicks();
                p = error;
                i = i + error;
                d = p - prevError;
                power = Range.clip((driveKp * p + driveKi * i + driveKd * d), DRIVE_MIN_POWER, maxPower);
                //telemetry.addData("PID", "Power: %f\nError: %f, prevError: %f\nPID: %f, P: %f, I: %f, D: %f", power, error, prevError, driveKp * p + driveKi * i + driveKd * d, p, i ,d);
                prevError = error;
            }

            // Calculate adjusted correction using IMU and
            // divide by 2 to apply to both sides of the drive train
            correction = pidDrive.performPID(robot.getAngle()) / 2;

            // Set the powers of both sides of the drive train
            robot.setLeftPower(power + correction);
            robot.setRightPower(power - correction);

            // Provide debugging data
            telemetry.addData("2 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("3 Global Heading", robot.globalAngle);
            telemetry.addData("4 Correction", correction);
            telemetry.update();
        }

        // Wait for wheels to completely stop
        robot.setDrivePower(0);
        sleep(100);
    }

    /**
     * Rotate the robot using PID and the expansion hub's internal IMU
     * @param degrees How far to turn
     * @param minPower Minimum power to rotate motors for adjustments
     * @param maxPower Maximum power at which to run motors
     */
    void rotate(int degrees, double minPower, double maxPower) {
        // restart imu angle tracking.
        robot.resetAngle();

        // Start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(minPower, maxPower);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0)
            {
                robot.setLeftPower(-maxPower);
                robot.setRightPower(maxPower);
                sleep(100);
            }

            do
            {
                maxPower = pidRotate.performPID(robot.getAngle()); // power will be - on right turn.
                robot.setLeftPower(maxPower);
                robot.setRightPower(-maxPower);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else {
            do {
                maxPower = pidRotate.performPID(robot.getAngle()); // power will be + on left turn.
                robot.setLeftPower(maxPower);
                robot.setRightPower(-maxPower);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        robot.setLeftPower(0);
        robot.setRightPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        robot.resetAngle();
    }

    /**
     * Strafe left with a similar mechanism to forward movement
     * TODO: Implement movement PID
     * @param distance Distance, in inches, to strafe
     * @param power Power at which to move motors
     */
    void strafeLeft(double distance, double power) {
        distance = distance + 1;
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Reset encoder values and toggle left strafe mode
        robot.resetEncoders();
        robot.toggleLeftStrafe();

        // Encoder values at which to stop movement
        double frontEncSum = robot.calculateFrontTicks(distance);
        double backEncSum  = robot.calculateBackTicks(distance);

        // Loop until both front and back tick values match their respective sum, thus reaching the target
        while (robot.getFrontTicks() < frontEncSum && robot.getBackTicks() < backEncSum && opModeIsActive()) {
            correction = pidDrive.performPID(robot.getAngle());

            // Set the powers of both sides of the drive train
            robot.setLeftPower(-power + correction);
            robot.setRightPower(-power);

            // Provide debugging data
            telemetry.addData("1 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", robot.globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.update();
        }

        // De-toggle left strafe and wait for wheels to finish spinning
        robot.toggleLeftStrafe();
        robot.setDrivePower(0);
        sleep(100);
    }

    /**
     * Strafe right with a similar mechanism to forward movement
     * TODO: Implement movement PID
     * @param distance Distance, in inches, to strafe
     * @param power Power at which to move motors
     */
    void strafeRight(double distance, double power) {
        distance = distance + 1;
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Reset encoder values and toggle right strafe mode
        robot.resetEncoders();
        robot.toggleRightStrafe();

        // Encoder values at which to stop movement
        double frontEncSum = robot.calculateFrontTicks(distance);
        double backEncSum  = robot.calculateBackTicks(distance);

        while (robot.getFrontTicks() < frontEncSum && robot.getBackTicks() < backEncSum && opModeIsActive()) {
            // Calculate adjusted correction using IMU and
            // divide by 2 to apply to both sides of the drive train
            correction = pidDrive.performPID(robot.getAngle());

            // Set the powers of both sides of the drive train
            robot.setLeftPower(-power);
            robot.setRightPower(-power + correction);

            // Provide debugging data
            telemetry.addData("1 IMU Heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 Global Heading", robot.globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.update();
        }

        // De-toggle right strafe and wait for wheels to finish spinning
        robot.toggleRightStrafe();
        robot.setDrivePower(0);
        sleep(100);
    }

    /**
     * Reverse the direction of all motors in the drive train
     */
    private void reverseDrive() {
        for (DcMotor motor : robot.driveTrain) {
            toggleMotorDirection(motor);
        }
    }

    /**
     * Toggles the direction of a passed motor
     * @param motor Motor to reverse
     */
    private void toggleMotorDirection(DcMotor motor) {
        if (motor.getDirection().equals(DcMotor.Direction.FORWARD)) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
}
