package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.*;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "RedDepot")
public class RedDepot extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        settings.chooseSettings();

        /**
         * Build Trajectories here
         */

        /**
         * Move one:
         *  - After reading barcode, move to deliver preloaded box on the correct level
         */
        TrajectorySequence movementOne = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).build();

        waitForStart();

        BarcodePosition barcodePosition = ((ShippingElementDetector) r.getPipeline()).getBarcodePosition();
        r.stopCamera();


    }
}
