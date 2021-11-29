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

import android.os.FileUriExposedException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

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
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, -38, toRadians(335)))
                .build();

        /**
         * All the movement for cycling
         */
        Pose2d cycleDeliveryPos = new Pose2d(3, -38, toRadians(335));
        //TODO: remove unused code once I get things working

//        Trajectory toWarehouse = drive.trajectoryBuilder(deliverPreload.end())
//                .addDisplacementMarker(() -> {
//                    r.getDeliveryControl().deliverServoStow();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    r.getDeliveryControl().moveDelivery(STOWED);
//                })
//                .lineToLinearHeading(new Pose2d(12, -67, toRadians(0)))
//                .build();

//        Trajectory freightPickup = drive.trajectoryBuilder(toWarehouse.end())
//                .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
//                .splineToConstantHeading(new Vector2d(46, -67), 0)
//                .addDisplacementMarker(() -> {
//                    r.runIntakeForward();
//                })
//
//                .build();

        List<Vector2d> pickupPoints = new ArrayList<Vector2d>() {{
           add(new Vector2d(46, -67));
           add(new Vector2d(47, -63));
           add(new Vector2d(47, -59));
        }};

        List<Trajectory> pickups = new ArrayList<>();

        for(int i = 0; i < pickupPoints.size(); i++) {

            pickups.add(drive.trajectoryBuilder(cycleDeliveryPos)
                    .addDisplacementMarker(() -> {
                        r.getDeliveryControl().deliverServoStow();
                    })
                    .addTemporalMarker(0.5, () -> {
                        r.getDeliveryControl().moveDelivery(STOWED);
                    })
                    .splineToSplineHeading(new Pose2d(8, -48, toRadians(0)), toRadians(270))
                    .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
                    .splineToConstantHeading(new Vector2d(16, -67), toRadians(0))
                    .splineToConstantHeading(pickupPoints.get(i), 0)
                    .build());
        }

        Vector2d outOfWarehousePos = new Vector2d(10, -67);

        Trajectory cycleDelivery = drive.trajectoryBuilder(new Pose2d(outOfWarehousePos.getX(), outOfWarehousePos.getY(), toRadians(0)))
            .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
            .lineToLinearHeading(cycleDeliveryPos)
            .build();


//        Trajectory deliverCycle = drive.trajectoryBuilder(toWarehouse.end())
//            .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
//            .lineToLinearHeading(new Pose2d(3, -38, toRadians(335)))
//            .build();

//        Trajectory toHub = drive.trajectoryBuilder(freightPickup.end())
//            .addDisplacementMarker(() -> {
//                r.stopIntake();
//                r.getDeliveryControl().moveDelivery(HIGH);
//            })
//            .lineToLinearHeading(new Pose2d(1, -38, toRadians(335)))
//            .build();

//        Trajectory parkReg = drive.trajectoryBuilder(toHub.end())
//                .addDisplacementMarker(() -> r.getDeliveryControl().deliverServoStow())
//                .addTemporalMarker(0.5, () -> r.getDeliveryControl().moveDelivery(STOWED))
//                .splineToSplineHeading(new Pose2d(12, -69, toRadians(0)), 0)
//                .splineToConstantHeading(new Vector2d(40, -69), 0)
//                .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(INTAKE))
//                .build();

        /**
         *  Park
         */
        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(cycleDeliveryPos);
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .addDisplacementMarker(() -> r.getDeliveryControl().deliverServoStow())
                    .addTemporalMarker(0.5, () -> r.getDeliveryControl().moveDelivery(STOWED))
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
//        Trajectory intermediatePark = drive.trajectoryBuilder(toHub.end())
//                .lineToLinearHeading(new Pose2d(0, -60, 0))
//                .build();
//
//        TrajectoryBuilder oldParkBuilder = drive.trajectoryBuilder(intermediatePark.end());
//        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
//            oldParkBuilder
//                    .splineToConstantHeading(new Vector2d(12, -66), toRadians(0))
//                    .splineToConstantHeading(new Vector2d(36, -66), 0);
//            if(settings.getParkType() == OFFSET) {
//                oldParkBuilder.splineToConstantHeading(new Vector2d(36, -36), 0);
//            }
//        }
//
//        Trajectory oldPark = oldParkBuilder.build();

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
//            drive.followTrajectory(toWarehouse);
//            drive.followTrajectory(freightPickup);

            //Go to pick up the freight
            drive.followTrajectory(pickups.get(i));

            //Drive forward until the element is detected
            //If a problem is detected the auton will get killed here
            boolean keepGoing = r.moveUntilElement(drive, maximumDistance, this);
            if(!keepGoing) {
                r.runIntakeBackwards();
                Thread.sleep(2000);
                r.stopIntake();
                stop();
            }

            //Deliver the element
            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> r.runIntakeBackwards())
                    .addTemporalMarker(1, () -> r.stopIntake())
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(36, -67), toRadians(180))
                    //TODO: Move this point closer to the barrier if possible
                    .splineToConstantHeading(new Vector2d(10, -67), toRadians(180))
                    .addDisplacementMarker(() -> r.getDeliveryControl().moveDelivery(HIGH))
                    .splineToConstantHeading(new Vector2d(9, -48), toRadians(90))
                    .splineToSplineHeading(cycleDeliveryPos, toRadians(135))
                    .setReversed(false)
                    .build();

            //This should take us all the way to the shipping hub
            drive.followTrajectorySequence(deliver);

            //Move to the shipping hub to deliver the freight
//            drive.followTrajectory(cycleDelivery);

//            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .addDisplacementMarker(() -> {
//                        r.runIntakeBackwards();
//                    })
//                    .addTemporalMarker(1, () -> r.stopIntake())
//                    .lineToLinearHeading(new Pose2d(12, -67, toRadians(0)))
//                    .build());
//
//            drive.followTrajectory(deliverCycle);

            //Deliver the freight and move on
            r.getDeliveryControl().deliverServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);

            maximumDistance += 4;
        }

        //This time we're just picking up an element and then parking, so that's what we'll do. Then, we'll park in the correct position
        //Since the position we're moving from is unpredictable (we don't know how far we had to move to intake an element), the trajectory is getting built here


        //Go park
        Thread.sleep((long) settings.getChosenParkDelay());
        drive.followTrajectory(park);


        //Old parking code - Replacing it currently
//        drive.followTrajectory(intermediatePark);
//        drive.followTrajectory(park);

        //Put the intake back in the correct position for teleop
        r.getDeliveryControl().moveDelivery(INTAKE);
        Thread.sleep(1000);

    }
}
