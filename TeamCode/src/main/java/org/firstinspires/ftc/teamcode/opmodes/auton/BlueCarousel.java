package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.*;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.*;
import static org.firstinspires.ftc.teamcode.Robot.AutonSettings.parkTypes.*;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds.*;
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
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import java.util.Arrays;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "Blue Carousel")
public class BlueCarousel extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        drive.setPoseEstimate(new Pose2d(-36, 65, toRadians(90)));

        settings.chooseSettings();
        alliance = BLUE;

        telemetry.addData("Building trajectories", "");
        telemetry.update();

        /**
         * Build Trajectories
         */

        /**
         * Deliver Preload Element
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-28, 37, toRadians(145)))
                .build();

        telemetry.addData("Building trajectories", "toCarousel");
        telemetry.update();

        /**
         * Go to the Carousel
         */
        Trajectory toCarousel = drive.trajectoryBuilder(deliverPreload.end())
                .lineToLinearHeading(new Pose2d(-59, 60, toRadians(135)))
                .build();

        telemetry.addData("Building trajectories", "park");
        telemetry.update();

        /**
         * Park
         */
        Trajectory intermediatePark = drive.trajectoryBuilder(toCarousel.end())
                .lineToLinearHeading(new Pose2d(-24, 48, 0))
                .build();

        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(intermediatePark.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR) {
            telemetry.addData("Building trajectories", "park part 1");
            telemetry.update();
            parkBuilder
                    //.splineToSplineHeading(new Pose2d(-24, 48, Math.toRadians(0)), 0)
                    .splineToConstantHeading(new Vector2d(12, 72), 0)
                    .splineToConstantHeading(new Vector2d(38, 72), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));

            telemetry.addData("Building trajectories", "park part 2");
            telemetry.update();
            if(settings.getParkType() == OFFSET) {
                parkBuilder.splineToConstantHeading(new Vector2d(38, 36), 0);
            }

        } else if (settings.getParkType() == SHIPPING_AREA) {
            parkBuilder.lineToLinearHeading(new Pose2d(-66, 36, 0));
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

        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getPipeline()).getDeliveryPosition();
        r.stopCamera();

        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliverServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        r.getDeliveryControl().deliverServoStow();
        Thread.sleep(500);
        r.getDeliveryControl().moveDelivery(STOWED);

        drive.followTrajectory(toCarousel);
        r.carouselClockwise(NORMAL);
        Thread.sleep(1000);
        r.carouselClockwise(FAST);
        Thread.sleep(1000);
        r.stopCarousel();

        drive.followTrajectory(intermediatePark);
        Thread.sleep((long) settings.getChosenParkDelay());
        drive.followTrajectory(park);
        r.getDeliveryControl().moveDelivery(INTAKE);

        Thread.sleep(1000);
    }
}
