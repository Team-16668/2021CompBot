package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.BLUE;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.OFFSET;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.REGULAR;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.SHIPPING_AREA;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.Constants.UNLOADED_PATTERN;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.MID;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;
import static java.lang.Math.toRadians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by: barta
 * On: 11/12/2021
 */

@Autonomous(name = "Blue Depot")
public class BlueDepot extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        drive.setPoseEstimate(new Pose2d(12,65, toRadians(90)));

        settings.chooseSettings();
        alliance = BLUE;

        telemetry.addData("Status", "Building trajectories");
        telemetry.update();
        Timing.Timer timer = new Timing.Timer(30);

        /**
         * Build Trajectories here
         */

        /**
         * Move one:
         *  - After reading barcode, move to deliver preloaded box on the correct level
         */
        Trajectory deliverPreload;

        /**
         * All the movement for cycling
         */
        Pose2d cycleDeliveryPos = new Pose2d(-1, 42, toRadians(60));

        /**
         * Move three:
         *  - Park
         */
        TrajectorySequenceBuilder parkBuilder = drive.trajectorySequenceBuilder(cycleDeliveryPos);
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .addDisplacementMarker(() -> r.getDeliveryControl().deliveryServoIntake())
                    .addTemporalMarker(0.5, () -> r.getDeliveryControl().moveDelivery(STOWED))
                    .lineToLinearHeading(new Pose2d(12, 69, 0))
                    .lineToConstantHeading(new Vector2d(40, 69));
            if(settings.getParkType() == OFFSET) {
                parkBuilder.lineToConstantHeading(new Vector2d(40, 42));
            }
        }

        TrajectorySequence park = parkBuilder.build();

        telemetry.addData("Trajectories building complete", "");
        telemetry.update();

        while(!opModeIsActive()) {
            telemetry.addData("Detected duck position", ((ShippingElementDetector) r.getBackPipeline()).getBarcodePosition().name());
            telemetry.addData("Detected Position", ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition().name());
            telemetry.addData("Element loaded", r.getDeliveryControl().isElementLoaded());
            telemetry.addData("Distance sensor", r.getDeliveryControl().getDistanceSensor().getDistance(MM));
            telemetry.update();

            r.lightsLoop();
            Thread.sleep(50);
        }

        waitForStart();
        timer.start();

        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition();


        if(deliveryPosition ==HIGH) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-3, 43, toRadians(60)))
                    .build();
        } else if(deliveryPosition == MID) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-3, 42, toRadians(60)))
                    .build();
        } else {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(4, 38, toRadians(35)))
                    .build();
        }

        List<Vector2d> pickupPoints = new ArrayList<>(Arrays.asList(
                new Vector2d(43, 67),
                new Vector2d(43, 67),
                new Vector2d(43, 67)
        ));

        List<TrajectorySequence> pickups = new ArrayList<>();

        for(int i = 0; i < pickupPoints.size(); i++) {

            Pose2d startPose = cycleDeliveryPos;
            if(i == 0) {
                startPose = deliverPreload.end();
            }

            TrajectorySequence trajectory;
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> r.getDeliveryControl().deliveryServoIntake())
                    .addTemporalMarker(1, () -> r.getDeliveryControl().moveDelivery(INTAKE))
                    .lineToLinearHeading(new Pose2d(12, 67, toRadians(0)))
                    .lineToConstantHeading(pickupPoints.get(i))
                    .addDisplacementMarker(() -> r.runIntakeForward())
                    ;

            trajectory = builder.build();

            pickups.add(trajectory);
        }

        r.stopBackCamera();

        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliveryServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        r.setLightPattern(UNLOADED_PATTERN);

        //Attempt cycles as long as we have time left on the clock :D

        boolean success;
        double maximumDistance = 10;
        for(int i = 0; i < 3; i++) {
            //Go to pick up the freight
            drive.followTrajectorySequence(pickups.get(i));

            //Drive forward until the element is detected
            //If a problem is detected the auton will get killed here
            success = r.moveUntilElement(drive, maximumDistance, this);
            boolean timeLeft = timer.remainingTime() > 8;
            if(!success || !timeLeft) {
                r.runIntakeBackwards();
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX() - 4, drive.getPoseEstimate().getY())).build());
                Thread.sleep(2000);
                r.stopIntake();
                if(settings.getParkType() == OFFSET) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(40, 66))
                            .lineToConstantHeading(new Vector2d(40, 40   ))
                            .build());
                }
                stop();
            }

            //TODO: Add failed as option for pre-element delivery
            //Deliver the element
            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> r.runIntakeBackwards())
                    .addTemporalMarker(1, () -> r.stopIntake())
                    .lineToConstantHeading(new Vector2d(14, 67))
                    .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
                    .lineToLinearHeading(cycleDeliveryPos)
                    .build();

            //This should take us all the way to the shipping hub
            drive.followTrajectorySequence(deliver);

            //Deliver the freight and move on
            r.getDeliveryControl().deliveryServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
            r.setLightPattern(UNLOADED_PATTERN);

            maximumDistance += 4;
        }

        //Park
        drive.followTrajectorySequence(park);

        //Put the intake back in the right position for teleop
        r.getDeliveryControl().moveDelivery(INTAKE);
        Thread.sleep(1000);

    }
}
