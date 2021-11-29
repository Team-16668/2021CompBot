package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.RED;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.OFFSET;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.REGULAR;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.SHIPPING_AREA;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.LOW;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;
import static java.lang.Math.*;

import java.util.Arrays;

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
         *  - After reading barcode, move to deliver preloaded box on the correct level
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, -38, toRadians(335)))
                .build();


        /**
         * Move two:
         *  - Move to shipping area and cycle
         */

        Trajectory toWarehouse = drive.trajectoryBuilder(deliverPreload.end())
                .addDisplacementMarker(() -> {
                    r.getDeliveryControl().deliverServoStow();
                })
                .addTemporalMarker(0.5, () -> {
                    r.getDeliveryControl().moveDelivery(STOWED);
                })
                .lineToLinearHeading(new Pose2d(12, -67, toRadians(0)))
                .build();


        Trajectory freightPickup = drive.trajectoryBuilder(toWarehouse.end())
                .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
                .splineToConstantHeading(new Vector2d(46, -67), 0)
                .addDisplacementMarker(() -> {
                    r.runIntakeForward();
                })

                .build();

        Trajectory newPickup = drive.trajectoryBuilder(new Pose2d(5, -38, toRadians(335)))
                .addDisplacementMarker(() -> {
                    r.getDeliveryControl().deliverServoStow();
                })
                .addTemporalMarker(0.5, () -> {
                    r.getDeliveryControl().moveDelivery(STOWED);
                })
                .splineToSplineHeading(new Pose2d(8, -48, toRadians(0)), toRadians(270))
                .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
                .splineToConstantHeading(new Vector2d(16, -67), toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -67), 0)
                .build();


        Trajectory deliverCycle = drive.trajectoryBuilder(toWarehouse.end())
            .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
            .lineToLinearHeading(new Pose2d(3, -38, toRadians(335)))
            .build();

        Trajectory toHub = drive.trajectoryBuilder(freightPickup.end())
            .addDisplacementMarker(() -> {
                r.stopIntake();
                r.getDeliveryControl().moveDelivery(HIGH);
            })
            .lineToLinearHeading(new Pose2d(1, -38, toRadians(335)))
            .build();


        /**
         * Parking XD
         */
        Trajectory parkReg = drive.trajectoryBuilder(toHub.end())
                .addDisplacementMarker(() -> r.getDeliveryControl().deliverServoStow())
                .addTemporalMarker(0.5, () -> r.getDeliveryControl().moveDelivery(STOWED))
                .splineToSplineHeading(new Pose2d(12, -69, toRadians(0)), 0)
                .splineToConstantHeading(new Vector2d(40, -69), 0)
                .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
                .build();



        /**
         * Move three:
         *  - Park
         */
        Trajectory intermediatePark = drive.trajectoryBuilder(toHub.end())
                .lineToLinearHeading(new Pose2d(0, -60, 0))
                .build();

        TrajectoryBuilder oldParkBuilder = drive.trajectoryBuilder(intermediatePark.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            oldParkBuilder
                    .splineToConstantHeading(new Vector2d(12, -66), toRadians(0))
                    .splineToConstantHeading(new Vector2d(36, -66), 0);
            if(settings.getParkType() == OFFSET) {
                oldParkBuilder.splineToConstantHeading(new Vector2d(36, -36), 0);
            }
        }

        Trajectory oldPark = oldParkBuilder.build();

        telemetry.addData("Trajectories building complete", "");
        telemetry.update();

        while(!opModeIsActive()) {
            telemetry.addData("Detected duck position", ((ShippingElementDetector) r.getBackPipeline()).getBarcodePosition().name());
            telemetry.addData("Detected Position", ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition().name());
            telemetry.update();
            Thread.sleep(50);
        }

        waitForStart();

        timer.start();

        //Detect the position of the Team Shipping Element
        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition();

        if(deliveryPosition == LOW) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-4, -44, toRadians(300)))
                    .build();
        }

        r.stopBackCamera();

        //Delivery the preloaded element
        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliverServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

        //Attempt cycles as long as we have time left on the clock :D
        double maximumDistance = 10;
        for(int i = 1; i <= 2; i++) {
            //Move to pick up the cube to cycle
            //TODO: Go through and remove extra comments once everything is working correctly
//            drive.followTrajectory(toWarehouse);
//            drive.followTrajectory(freightPickup);
            drive.followTrajectory(newPickup);

            //Drive forward until the element is detected
            boolean keepGoing = r.moveUntilElement(drive, maximumDistance, this);
            if(!keepGoing) {
                r.runIntakeBackwards();
                Thread.sleep(2000);
                r.stopIntake();
                stop();
            }

            //Deliver the element
            TrajectorySequence cycleDelivery = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> r.runIntakeBackwards())
                    .addTemporalMarker(1, () -> r.stopIntake())
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(10, -67), toRadians(180))
                    .setReversed(false)
                    .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
                    .lineToLinearHeading(new Pose2d(3, -38, toRadians(335)))
                    .build();

            drive.followTrajectorySequence(cycleDelivery);

//            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .addDisplacementMarker(() -> {
//                        r.runIntakeBackwards();
//                    })
//                    .addTemporalMarker(1, () -> r.stopIntake())
//                    .lineToLinearHeading(new Pose2d(12, -67, toRadians(0)))
//                    .build());
//
//            drive.followTrajectory(deliverCycle);

            r.getDeliveryControl().deliverServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

            maximumDistance += 4;
        }

        //Move to pick up the cube to end the auton
        drive.followTrajectory(parkReg);

        //Drive forward until the element is detected
//        r.moveUntilElement(drive, 15, this);

        //This time we're just picking up an element and then parking, so that's what we'll do. Then, we'll park in the correct position
        //Since the position we're moving from is unpredictable (we don't know how far we had to move to intake an element), the trajectory is getting built here
        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(drive.getPoseEstimate());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .splineToConstantHeading(new Vector2d(36, -66), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        r.stopIntake();
                    });
            if(settings.getParkType() == OFFSET) {
                parkBuilder.splineToConstantHeading(new Vector2d(36, -36), 0);
            }
        }

        Trajectory park = parkBuilder.build();


        drive.followTrajectory(park);


        //Old parking code - Replacing it currently
        //TODO: remove these trajectories from the code when we get the newer route working
//        drive.followTrajectory(intermediatePark);
//        drive.followTrajectory(park);

        //Put the intake back in the correct position for teleop
        r.getDeliveryControl().moveDelivery(INTAKE);
        Thread.sleep(1000);

    }
}
