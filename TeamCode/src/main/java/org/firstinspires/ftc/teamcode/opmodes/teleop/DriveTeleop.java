package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name="Just driving")
@Disabled
public class DriveTeleop extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            r.driveControlLoop(drive, gamepad1);
        }
    }
}
