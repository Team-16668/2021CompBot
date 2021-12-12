package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.RED;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.OFFSET;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.REGULAR;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.SHIPPING_AREA;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.MID;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;
import static java.lang.Math.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "Red Depot")
public class RedDepot extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        drive.setPoseEstimate(new Pose2d(12,-65, toRadians(270)));

        settings.chooseSettings();
        alliance = RED;

        telemetry.addData("Status", "Building trajectories");
        telemetry.update();
        Timing.Timer timer = new Timing.Timer(30);

        /**
         * Build Trajectories here
         */

        /**
         * Move one:
         *  Deliver the preloaded box to the correct level
         */
        Trajectory deliverPreload;

        /**
         * All the movement for cycling
         */
        Pose2d cycleDeliveryPos = new Pose2d(-5, -46, toRadians(300));

        /**
         *  Park
         */
        TrajectorySequenceBuilder parkBuilder = drive.trajectorySequenceBuilder(cycleDeliveryPos);
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .addDisplacementMarker(() -> r.getDeliveryControl().deliveryServoIntake())
                    .addTemporalMarker(0.5, () -> r.getDeliveryControl().moveDelivery(STOWED))
                    .lineToLinearHeading(new Pose2d(12, -66, 0))
                    .lineToConstantHeading(new Vector2d(38, -66), new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                   ;
            if(settings.getParkType() == OFFSET) {
                parkBuilder.lineToConstantHeading(new Vector2d(34, -42));
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

        //Detect the position of the Team Shipping Element
        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition();

        if(deliveryPosition ==HIGH) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-3, -47, toRadians(300)))
                    .build();
        } else if(deliveryPosition == MID) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-3, -46, toRadians(300)))
                    .build();
        } else {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-4, -44, toRadians(300)))
                    .build();
        }


        List<Vector2d> pickupPoints = new ArrayList<Vector2d>(Arrays.asList(
                new Vector2d(40, -67),
                new Vector2d(40, -67)
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
                    .lineToLinearHeading(new Pose2d(12, -67, toRadians(0)))
                    .lineToConstantHeading(pickupPoints.get(i))
                    .addDisplacementMarker(() -> r.runIntakeForward())
                    ;

            trajectory = builder.build();

            pickups.add(trajectory);
        }

        r.stopBackCamera();

        //Delivery the preloaded element
        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliveryServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

        //Attempt cycles as long as we have time left on the clock :D

        boolean success;
        double maximumDistance = 10;
        for(int i = 0; i < 2; i++) {
            //Go to pick up the freight
            drive.followTrajectorySequence(pickups.get(i));

            //Drive forward until the element is detected
            //If a problem is detected the auton will get killed here
            success = r.moveUntilElement(drive, maximumDistance, this);

            if(!success) {
                r.runIntakeBackwards();
                Thread.sleep(2000);
                r.stopIntake();
                if(settings.getParkType() == OFFSET) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToConstantHeading(new Vector2d(36, -66))
                            .lineToConstantHeading(new Vector2d(36, -36))
                            .build());
                }
                stop();
            }

            //Deliver the element
            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> r.runIntakeBackwards())
                    .addTemporalMarker(2, () -> r.stopIntake())
                    .lineToConstantHeading(new Vector2d(14, -67))
                    .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
                    .lineToLinearHeading(cycleDeliveryPos)
                    .build();

            //This should take us all the way to the shipping hub
            drive.followTrajectorySequence(deliver);

            //Deliver the freight and move on
            r.getDeliveryControl().deliveryServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

            maximumDistance += 4;
        }

        //Go park
        drive.followTrajectorySequence(park);


        //Put the intake back in the correct position for teleop
        r.getDeliveryControl().moveDelivery(INTAKE);
        Thread.sleep(1000);

    }
}
