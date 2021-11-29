package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "move until element test")
public class MoveUntilElementTest extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null, false, null);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        r.runIntakeForward();
        r.moveUntilElement(drive, 10, this);
        r.stopIntake();
    }
}