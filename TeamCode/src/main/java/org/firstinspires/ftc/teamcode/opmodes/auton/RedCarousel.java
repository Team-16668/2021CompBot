package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.*;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.*;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.*;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds.FAST;
import static org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds.NORMAL;
import static java.lang.Math.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import java.util.Arrays;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "Red Carousel")
public class RedCarousel extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    //TODO: Remove commented code once I get the vision version working
    //TODO: Do this for blue

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
//        r = new Robot(hardwareMap, true, new ShippingElementDetector(), true,
//                new DuckDetector(new Vector2d(8, 0), -62, telemetry, drive));

        telemetry.addData("Got here", "indeed");
        telemetry.update();
        //TODO: Adjust this to reflect the actual time we have
        settings = new AutonSettings(gamepad1, telemetry, 0, 6);

        r.getDashboard().startCameraStream(r.getFrontWebcam(), 30);

        drive.setPoseEstimate(new Pose2d(-36, -65, toRadians(270)));

        settings.chooseSettings();
        alliance = RED;

        telemetry.addData("Building trajectories", "");
        telemetry.update();

        /**
         * Build Trajectories
         */

        Pose2d deliverDuckPose = new Pose2d(-29, -38, toRadians(215));
        Pose2d intermediateParkPose = new Pose2d(-36, -48, 0);

        /**
         * Go to the Carousel
         */
        Trajectory toCarousel = drive.trajectoryBuilder(deliverPreload.end())
                .addDisplacementMarker(() -> {
                    r.getDeliveryControl().deliverServoStow();
                })
                .addTemporalMarker(0.5, () -> {
                    r.getDeliveryControl().moveDelivery(STOWED);
                })
                .lineToLinearHeading(new Pose2d(-60, -60, toRadians(225)))
                .build();

        telemetry.addData("Building trajectories", "park");
        telemetry.update();

        /**
         * Detect and Deliver Duck
         */
        Trajectory detectDuck = drive.trajectoryBuilder(toCarousel.end())
                .lineToLinearHeading( new Pose2d(-50, -40, toRadians(270)))
                .build();

        /**
         * Park
         */
        Trajectory intermediatePark = drive.trajectoryBuilder(deliverDuckPose)
                .addDisplacementMarker(() -> {
                    r.getDeliveryControl().deliverServoStow();
                })
                .addTemporalMarker(0.5, () -> {
                    r.getDeliveryControl().moveDelivery(STOWED);
                })
                .lineToLinearHeading(intermediateParkPose)
                .build();

        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(intermediatePark.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR) {
            parkBuilder
                    .splineToConstantHeading(new Vector2d(-12, -65), 0)
                    .splineToConstantHeading(new Vector2d(12, -65), 0)
                    .splineToConstantHeading(new Vector2d(40, -65), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));

//            if(settings.getParkType() == OFFSET) {
//                parkBuilder.splineToConstantHeading(new Vector2d(38, -36), 0);
//            }

        } else if (settings.getParkType() == SHIPPING_AREA) {
            parkBuilder.lineToLinearHeading(new Pose2d(-66, -40, 0));
        }

        Trajectory park = parkBuilder.build();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        while(!opModeIsActive()) {
                telemetry.addData("Detected Position", ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition().name());
                telemetry.update();
                Thread.sleep(50);
            }

            waitForStart();

            DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition();
            r.stopBackCamera();

            /**
             * Deliver Preload Element
             */

            Trajectory deliverPreload;

            if(deliveryPosition == HIGH) {
                deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-29, -38, toRadians(215)))
                        .build();

            } else if(deliveryPosition == MID) {
                deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28, -38, toRadians(215)))
                        .build();
            } else {
                //LOW
                deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-27, -37, toRadians(215)))
                        .build();
            }

            //Deliver Preload
            r.getDeliveryControl().moveDelivery(deliveryPosition);
            drive.followTrajectory(deliverPreload);

            //TODO: Change this to the new delivery method for the other autons (Red carousel should be done already)
            r.getDeliveryControl().deliverServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

            //Deliver duck to the field
            drive.followTrajectory(toCarousel);
            r.carouselCounterClockwise(NORMAL);
            //TODO: Tune this time
            Thread.sleep(1500);
            r.carouselCounterClockwise(FAST);
            Thread.sleep(1500);
            r.stopCarousel();

            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(intermediateParkPose).build());

            //Park
            Thread.sleep((long) settings.getChosenParkDelay());
            drive.followTrajectory(park);
            r.getDeliveryControl().moveDelivery(INTAKE);

            //TODO: Separate delivery pos for preload on low level

            if(settings.getParkType() == REGULAR || settings.getParkType() == OFFSET) {
                r.runIntakeForward();
                r.moveUntilElement(drive, 10, this);
                r.runIntakeBackwards();
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(4).build());
                Thread.sleep(1000);
                r.stopIntake();
            }
    }
}
