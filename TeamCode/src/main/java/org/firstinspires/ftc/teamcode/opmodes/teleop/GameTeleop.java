package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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

        while (opModeIsActive()) {
            //Loop for main driving code (wheel control)
            r.driveControlLoop(drive, gamepad1);

            //Loop for control of the carousel
            r.carouselControlLoop(gamepad1);

            //Loop for the control of the delivery arm and intake.
            r.armControlLoopTeleOp(gamepad2);

        }
    }

}
