package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

import static java.lang.Math.toRadians;

@TeleOp(name = "Duck Vision Test")
public class DuckVisionTest extends LinearOpMode {
    Robot r;
    SampleMecanumDrive drive;

    boolean currentA, prevA = false;
    boolean currDpadUp, prevDpadUp = false;
    boolean currDpadDown, prevDpadDown = false;
    boolean currDpadLeft, prevDpadLeft = false;
    boolean currDpadRight, prevDpadRight = false;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null, true,
                new DuckDetector(new Vector2d(6, 0), -66, telemetry));

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36, -24, toRadians(270)));

        r.getDashboard().startCameraStream(r.getFront_webcam(), 30);

        waitForStart();

        while(opModeIsActive()) {

            currentA = gamepad1.a;
            currDpadUp = gamepad1.dpad_up;
            currDpadDown = gamepad1.dpad_down;
            currDpadLeft = gamepad1.dpad_left;
            currDpadRight = gamepad1.dpad_right;

            if(currentA && currentA != prevA) {
                ((DuckDetector) r.getFront_pipeline()).loopMats();
            }
            if(currDpadUp && currDpadUp != prevDpadUp) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 2, drive.getPoseEstimate().getHeading()));
            } else if(currDpadDown && currDpadDown != prevDpadDown) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 2, drive.getPoseEstimate().getHeading()));
            } else if(currDpadLeft && currDpadLeft != prevDpadLeft) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX() - 2, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            } else if(currDpadLeft && currDpadRight != prevDpadRight) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            }

            ((DuckDetector) r.getFront_pipeline()).setRobotPose(drive.getPoseEstimate());


            prevA = currentA;

            prevDpadUp = currDpadUp;
            prevDpadDown = currDpadDown;
            prevDpadLeft = currDpadLeft;
            prevDpadRight = currDpadRight;
        }
    }
}
