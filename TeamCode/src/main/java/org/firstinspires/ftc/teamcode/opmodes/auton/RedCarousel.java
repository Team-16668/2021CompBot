package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="Red Carousel")
public class RedCarousel extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }
}
