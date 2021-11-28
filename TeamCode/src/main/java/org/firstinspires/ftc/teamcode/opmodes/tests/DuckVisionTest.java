package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.DuckDetector;

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
        DuckDetector pipeline = new DuckDetector(new Vector2d(8, 0), -62, telemetry);
        r = new Robot(hardwareMap, false, null, true, pipeline);

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36, -24, toRadians(270)));

        waitForStart();

        r.getDashboard().startCameraStream(r.getFrontWebcam(), 30);

        while(opModeIsActive()) {

            currentA = gamepad1.a;
            currDpadUp = gamepad1.dpad_up;
            currDpadDown = gamepad1.dpad_down;
            currDpadLeft = gamepad1.dpad_left;
            currDpadRight = gamepad1.dpad_right;

            if(currentA && currentA != prevA) {
                ((DuckDetector) r.getFrontPipeline()).loopMats();
            }
            if(currDpadUp && currDpadUp != prevDpadUp) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 2, drive.getPoseEstimate().getHeading()));
            } else if(currDpadDown && currDpadDown != prevDpadDown) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 2, drive.getPoseEstimate().getHeading()));
            } else if(currDpadLeft && currDpadLeft != prevDpadLeft) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX() - 2, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            } else if(currDpadRight && currDpadRight != prevDpadRight) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX() + 2, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            }

            ((DuckDetector) r.getFrontPipeline()).setRobotPose(drive.getPoseEstimate());
            drive.update();

            if(gamepad1.a) {
                Vector2d goToPoint = ((DuckDetector) r.getFrontPipeline()).getGoToPoint();

                //Pickup the duck
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .addDisplacementMarker(() -> {
                                    r.runIntakeForward();
                                })
                                .lineToConstantHeading(goToPoint)
                                .build()
                );
            }

            prevA = currentA;

            prevDpadUp = currDpadUp;
            prevDpadDown = currDpadDown;
            prevDpadLeft = currDpadLeft;
            prevDpadRight = currDpadRight;
        }
    }
}
