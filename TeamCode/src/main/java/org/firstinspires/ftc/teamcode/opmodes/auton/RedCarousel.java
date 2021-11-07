package org.firstinspires.ftc.teamcode.opmodes.auton;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.*;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds.*;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition.*;
import static java.lang.Math.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.AutonSettings;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "RedCarousel")
public class RedCarousel extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;
    AutonSettings settings;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());
        drive = new SampleMecanumDrive(hardwareMap);
        settings = new AutonSettings(gamepad1, telemetry, 0, 10);

        //TODO: Set Starting Position
        drive.setPoseEstimate(new Pose2d(-36, -65, toRadians(270)));

        settings.chooseSettings();
        alliance = RED;

        /**
         * Build Trajectories
         */

        /**
         * Deliver Preload Element
         */
        Trajectory deliverPreload = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-24, -36, toRadians(225)))
                .build();

        /**
         * Go to the Carousel
         */
        Trajectory toCarousel = drive.trajectoryBuilder(deliverPreload.end())
                .lineToConstantHeading(new Vector2d(-66, -66))
                .build();

        /**
         * Park
         */
        Trajectory toDepot = drive.trajectoryBuilder(toCarousel.end())
                .splineToLinearHeading(new Pose2d(12, -66, Math.toRadians(0)), 0)
                .splineToConstantHeading(new Vector2d(36, -66), 0)
                .build();

        waitForStart();

        BarcodePosition position = ((ShippingElementDetector) r.getPipeline()).getBarcodePosition();

        DeliveryArmControl.DeliveryPositions deliveryPosition = HIGH;

        if(position == LEFT) {
            //Level 1
            deliveryPosition = LOW;
        } else if(position == MIDDLE) {
            //Level 2
            deliveryPosition = MID;
        } else if(position == RIGHT || position == NONE) {
            //Level 3
            deliveryPosition = HIGH;
        }

        //TODO: Set up this function to raise the delivery mechanism
        r.getDeliveryControl().moveDelivery(deliveryPosition);
        drive.followTrajectory(deliverPreload);

        r.getDeliveryControl().deliverServoDeliver();
        Thread.sleep(500);
        r.getDeliveryControl().deliverServoStow();

        r.getDeliveryControl().moveDelivery(STOWED);
        drive.followTrajectoryAsync(toCarousel);
        r.carouselClockwise(NORMAL);
        Thread.sleep(1000);
        r.carouselClockwise(FAST);
        Thread.sleep(200);
        r.stopCarousel();

        Thread.sleep((long) settings.getChosenParkDelay());

        drive.followTrajectory(toDepot);
    }
}
