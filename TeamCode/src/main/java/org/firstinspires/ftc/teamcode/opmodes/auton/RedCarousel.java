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

@Autonomous(name = "Red Carousel")
public class RedCarousel extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        drive.setPoseEstimate(new Pose2d(-36, -65, toRadians(270)));

        settings.chooseSettings();
        alliance = RED;

        telemetry.addData("Building trajectories", "");
        telemetry.update();

        /**
         * Build Trajectories
         */

        /**
         * Deliver Preload Element
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-30, -36, toRadians(225)))
                .build();

        telemetry.addData("Building trajectories", "delivery preload");
        telemetry.update();

        /**
         * Go to the Carousel
         */
        Trajectory toCarousel = drive.trajectoryBuilder(deliverPreload.end())
                .lineToConstantHeading(new Vector2d(-60, -60))
                .build();

        telemetry.addData("Building trajectories", "toCarousel");
        telemetry.update();

        /**
         * Park
         */
        TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(toCarousel.end());
        if(settings.getParkType() == OFFSET || settings.getParkType() == REGULAR) {
            parkBuilder. splineToSplineHeading(new Pose2d(-24, -48, Math.toRadians(0)), 0)
                    .splineToConstantHeading(new Vector2d(12, -66), 0)
                    .splineToConstantHeading(new Vector2d(24, -66), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));
            if(settings.getParkType() == OFFSET) {
                parkBuilder.splineToConstantHeading(new Vector2d(24, -36), 0);
            }

        } else if (settings.getParkType() == SHIPPING_AREA) {
            parkBuilder.lineToLinearHeading(new Pose2d(-66, -36));
        }

        Trajectory park = parkBuilder.build();

        telemetry.addData("Trajectories building complete", "");
        telemetry.update();

        while(!opModeIsActive()) {
            telemetry.addData("Detected Position", ((ShippingElementDetector) r.getPipeline()).getDeliveryPosition().name());
            Thread.sleep(50);
        }

        waitForStart();

        DeliveryPositions deliveryPosition = ((ShippingElementDetector) r.getPipeline()).getDeliveryPosition();
        r.stopCamera();

        //TODO: Set up this function to raise the delivery mechanism
        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliverServoDeliver();
        Thread.sleep(DELIVERY_SERVO_WAIT_TIME);
        r.getDeliveryControl().deliverServoStow();
        r.getDeliveryControl().moveDelivery(STOWED);

        drive.followTrajectory(toCarousel);
        r.carouselCounterClockwise(NORMAL);
        Thread.sleep(1000);
        r.carouselCounterClockwise(FAST);
        Thread.sleep(1000);
        r.stopCarousel();

        Thread.sleep((long) settings.getChosenParkDelay());

        drive.followTrajectory(park);
        r.getDeliveryControl().moveDelivery(INTAKE);

        Thread.sleep(1000);
    }
}
