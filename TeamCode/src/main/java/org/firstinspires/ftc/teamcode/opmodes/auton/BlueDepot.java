package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.DuckDetectorPipeline;

/**
 * Created by: barta
 * On: 11/2/2021
 */

@Autonomous(name = "BlueDepot")
public class BlueDepot extends LinearOpMode {

    Robot r;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new DuckDetectorPipeline());
        drive = new SampleMecanumDrive(hardwareMap);
    }
}
