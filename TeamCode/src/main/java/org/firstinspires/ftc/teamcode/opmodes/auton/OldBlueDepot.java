package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.BLUE;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.OFFSET;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.REGULAR;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.ParkTypes.SHIPPING_AREA;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.LOW;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import java.util.Arrays;

/**
 * Created by: barta
 * On: 11/12/2021
 */


//TODO: Delete me when the new one is working
@Disabled
@Autonomous(name = "Old Blue Depot")
public class OldBlueDepot extends LinearOpMode {

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

        /**
         * Build Trajectories here
         */

        /**
         * Move one:
         *  - After reading barcode, move to deliver preloaded box on the correct level
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, 38, toRadians(35)))
                .build();


        /**
         * Move two:
         *  - Move to shipping area and cycle
         */
        Trajectory cycle = drive.trajectoryBuilder(deliverPreload.end())
                .splineToSplineHeading(new Pose2d(12, 66, toRadians(0)), 0)
                .addTemporalMarker(5, () -> {
                    r.runIntakeForward();
                    r.getDeliveryControl().moveDelivery(INTAKE);
                })
                .splineToSplineHeading(new Pose2d(42, 66, toRadians(0)), 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(12, 66), 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    r.stopIntake();
                    r.getDeliveryControl().moveDelivery(HIGH);
                })
                .splineToSplineHeading(new Pose2d(6, 36, toRadians(135)), toRadians(135))
                .build();

        /**
         * Move three:
         *  - Park
         */
        Trajectory intermediatePark = drive.trajectoryBuilder(deliverPreload.end())
                .lineToLinearHeading(new Pose2d(0, 60, 0))
                .build();

        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(intermediatePark.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR || settings.getParkType() == SHIPPING_AREA) {
            parkBuilder
                    .splineToConstantHeading(new Vector2d(12, 69), toRadians(0))
                    .splineToConstantHeading(new Vector2d(38, 69), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));
            if(settings.getParkType() == OFFSET) {
                parkBuilder.splineToConstantHeading(new Vector2d(38, 38), 0);
            }
        }

        Trajectory park = parkBuilder.build();

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

        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getBackPipeline()).getDeliveryPosition();

        if(deliveryPosition == LOW) {
            deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(4, 38, toRadians(35)))
                    .build();
        }

        r.stopBackCamera();

        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliveryServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        r.getDeliveryControl().deliveryServoIntake();
        Thread.sleep(500);
        r.getDeliveryControl().moveDelivery(STOWED);

        //drive.followTrajectory(cycle);
        //r.getDeliveryControl().deliverServoDeliver();
        //Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        //r.getDeliveryControl().deliverServoStow();
        //Thread.sleep(500);
        //r.getDeliveryControl().moveDelivery(STOWED);

        drive.followTrajectory(intermediatePark);
        drive.followTrajectory(park);
        r.getDeliveryControl().moveDelivery(INTAKE);

        Thread.sleep(1000);

    }
}
