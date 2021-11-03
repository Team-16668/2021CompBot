package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Robot.Constants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name="Game Teleop")
public class GameTeleop extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);
        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        while(opModeIsActive()) {

            //Loop for main driving code (wheel control)
            driveCode();

            carouselCode();

            //TODO: Intake Control

            //TODO: Delivery Control


        }
    }

    /**
     * Main loop for control of the wheels and driving around (including speed changes)
     */
    void driveCode() {
        double multiplier;

        if(gamepad1.left_bumper) {
            multiplier = SLOW_SPEED;
        } else if(gamepad1.right_bumper) {
            multiplier = FAST_SPEED;
        } else {
            multiplier = NORMAL_SPEED;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * multiplier,
                        -gamepad1.left_stick_x * multiplier,
                        -gamepad1.right_stick_x * multiplier
                ));
    }

    void carouselCode() {
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;

        if(leftTrigger > 0) {
            r.carouselClockwise();
        } else if(rightTrigger > 0) {
            r.carouselCounterClockwise();
        } else {
            if(r.getCarouselMotor().getPower() != 0) {
                r.carouselStop();
            }
        }
    }

}
