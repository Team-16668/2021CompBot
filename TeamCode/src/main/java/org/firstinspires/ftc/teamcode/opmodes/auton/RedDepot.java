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
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.RED;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.parkTypes.OFFSET;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.parkTypes.REGULAR;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.parkTypes.SHIPPING_AREA;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.LOW;
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
         *  - After reading barcode, move to deliver preloaded box on the correct level
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, -36, toRadians(335)))
                .build();


        /**
         * Move two:
         *  - Move to shipping area and cycle
         */

        Trajectory toCycle = drive.trajectoryBuilder(deliverPreload.end())
                .lineToLinearHeading(new Pose2d(12, -66, toRadians(0)))
                .build();

        List<Trajectory> cycles = new ArrayList<>();

        //TODO: Tune the positions these move to
        List<Vector2d> cyclePickupPoints = new ArrayList<Vector2d>() {{
            add(new Vector2d(42, -66));
            add(new Vector2d(42, -62));
            add(new Vector2d(42, -58));
        }};

        for(Vector2d pickup : cyclePickupPoints) {
            cycles.add(drive.trajectoryBuilder(toCycle.end())
                    .addDisplacementMarker(() -> {
                        r.runIntakeForward();
                        r.getDeliveryControl().moveDelivery(INTAKE);
                    })
                    .splineToConstantHeading(new Vector2d(36, -66), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(pickup, 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(new Vector2d(12, -66), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {
                        r.stopIntake();
                        r.getDeliveryControl().moveDelivery(HIGH);
                    })
                    .build());
        }

        Trajectory toHub = drive.trajectoryBuilder(cycles.get(0).end())
                .lineToLinearHeading(new Pose2d(2, -38, toRadians(335)))
                .build();


        /**
         * Move three:
         *  - Park
         */
        Trajectory intermediatePark = drive.trajectoryBuilder(toHub.end())
                .lineToLinearHeading(new Pose2d(0, -60, 0))
                .build();

        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(intermediatePark.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .splineToConstantHeading(new Vector2d(12, -66), toRadians(0))
                    .splineToConstantHeading(new Vector2d(36, -66), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));
            if(settings.getParkType() == OFFSET) {
                parkBuilder.splineToConstantHeading(new Vector2d(36, -36), 0);
            }
        }

        Trajectory park = parkBuilder.build();

        telemetry.addData("Trajectories building complete", "");
        telemetry.update();

        while(!opModeIsActive()) {
            telemetry.addData("Detected duck position", ((ShippingElementDetector) r.getPipeline()).getBarcodePosition().name());
            telemetry.addData("Detected Position", ((ShippingElementDetector) r.getPipeline()).getDeliveryPosition().name());
            telemetry.update();
            Thread.sleep(50);
        }

        waitForStart();

        timer.start();

        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getPipeline()).getDeliveryPosition();

        if(deliveryPosition == LOW) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-4, -44, toRadians(300)))
                    .build();
        }

        r.stopCamera();

        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliverServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        r.getDeliveryControl().deliverServoStow();
        Thread.sleep(500);
        r.getDeliveryControl().moveDelivery(STOWED);

        for(Trajectory cycle : cycles) {
            drive.followTrajectory(toCycle);
            drive.followTrajectory(cycle);
            drive.followTrajectory(toHub);
            r.getDeliveryControl().deliverServoDeliver();
            Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
            r.getDeliveryControl().deliverServoStow();
            Thread.sleep(500);
            r.getDeliveryControl().moveDelivery(STOWED);

            //If there's not enough time yet, just break and don't continue any farther.
            //TODO: Tune the time needed to park here
            if(timer.remainingTime() < 10) {
                break;
            }
        }

        drive.followTrajectory(intermediatePark);
        drive.followTrajectory(park);
        r.getDeliveryControl().moveDelivery(INTAKE);

        Thread.sleep(1000);

    }
}
