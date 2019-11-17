package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import java.util.ArrayList;

/**
 * Robot hardware configuration class, represents the entire robot
 * and contains minimalized motor movement functionality
 */
class RobotHardware {
    // Robot locational statistics
    private BNO055IMU imu;
    Orientation lastAngles;
    double globalAngle;

    // Drive motor configuration constants
    final private MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    final private double WHEEL_DIAMETER = 3.937;
    final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    final private double INCHES_PER_REV = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;

    // Drive train motors
    List<DcMotor> driveTrain;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Servo armServo;

    /*
    // Hang motor
    DcMotor hang;

    // Servos
    Servo deployment1;
    Servo deployment2;
    */
    Servo arm;

    /**
     * Robot hardware constructor including the imu
     *
     * @param hardwareMap Current hardware configuration
     * @param imu Inertial Measurement Unit object, retrieves positional information for the robot
     */
    RobotHardware(HardwareMap hardwareMap, BNO055IMU imu) {
        this(hardwareMap);
        this.imu = imu;

        lastAngles = new Orientation();
    }

    /**
     * Robot hardware constructor, configure all motor configurations
     *
     * @param hardwareMap Current hardware configuration
     */
    RobotHardware(HardwareMap hardwareMap) {
        armServo = hardwareMap.servo.get("arm_servo");

        frontLeftDrive  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDrive   = hardwareMap.dcMotor.get("back_left_drive");
        backRightDrive  = hardwareMap.dcMotor.get("back_right_drive");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Motor grouping lists */
        driveTrain = new ArrayList<>();
        driveTrain.add(this.frontLeftDrive);
        driveTrain.add(this.frontRightDrive);
        driveTrain.add(this.backLeftDrive);
        driveTrain.add(this.backRightDrive);
    }

    /**
     * Updates the global angle using the imu
     * @return The measured angle in degrees
     */
    double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Reset drive train encoder positions
     */
    void resetEncoders() {
        for (DcMotor motor : driveTrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Calculate the sum of the left motor positions
     * @return Sum of the left encoder positons
     */
    double getLeftTicks() {
        return Math.abs(frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the right motor positions
     * @return Sum of the right encoder positons
     */
    double getRightTicks() {
        return Math.abs(frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the front motor positions
     * @return Sum of the right encoder positons
     */
    double getFrontTicks() {
        return Math.abs(frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition());
    }

    /**
     * Calculate the sum of the front motor positions
     * @return Sum of the right encoder positons
     */
    double getBackTicks() {
        return Math.abs(backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition());
    }

    /**
     * Calculate the desired tick rotation of the left motors
     * @param distance The distance to travel
     * @return Sum of the projected left motor encoder positions
     */
    double calculateLeftTicks(double distance) {
        return -frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the right motors
     * @param distance The distance to travel
     * @return Sum of the projected right motor encoder positions
     */
    double calculateRightTicks(double distance) {
        return frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the front motors
     * @param distance The distance to travel
     * @return Sum of the projected front motor encoder positions
     */
    double calculateFrontTicks(double distance) {
        return -frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Calculate the desired tick rotation of the back motors
     * @param distance The distance to travel
     * @return Sum of the projected back motor encoder positions
     */
    double calculateBackTicks(double distance) {
        return frontLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV) + -backLeftDrive.getCurrentPosition() + (distance / INCHES_PER_REV);
    }

    /**
     * Change motors to or from left strafe orientation
     */
    void toggleLeftStrafe() {
        if (frontLeftDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        if (backRightDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    /**
     * Change motors to or from right strafe orientation
     */
    void toggleRightStrafe() {
        if (frontRightDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        }
        if (backLeftDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Set the power for each motor in the drive train
     * @param power Power at which to move motors
     */
    void setDrivePower(double power) {
        for (DcMotor motor : driveTrain) {
            motor.setPower(power);
        }
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setLeftPower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setRightPower(double power) {
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setFrontPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
    }

    /**
     * Set the power of all motors in the drive train
     * @param power Power at which to move motors
     */
    void setBackPower(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
}
