package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * Autonomous OpMode which simply enables vision detection
 */
@Autonomous(name="Vision Testing", group="Autonomous")
public class VisionOpMode extends LinearOpMode {

    /** Robot representation */
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        sleep(2500);

        List<Recognition> recognitions;
        Vision vision = new Vision(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Checking stuff out...");
        telemetry.update();

        while (opModeIsActive()) {
            recognitions = vision.getRecognitions();
            for (Recognition recognition : recognitions) {
                telemetry.addData("Recognition", recognition.getLabel());
            }
            telemetry.update();
        }

        vision.disableDetection();
    }
}
