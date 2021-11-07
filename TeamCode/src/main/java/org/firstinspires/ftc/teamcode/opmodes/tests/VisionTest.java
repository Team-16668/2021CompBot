package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.vision.ShippingElementDetector;

@TeleOp(name="Vision Test")
public class VisionTest extends LinearOpMode {

    Robot r;

    boolean currentA, prevA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, true, new ShippingElementDetector());

        r.getDashboard().startCameraStream(r.getWebcam(), 30);

        while(!opModeIsActive()) {
            telemetry.addData("Position", ((ShippingElementDetector) r.getPipeline()).getBarcodePosition().name());
            telemetry.addData("Active Mat", ((ShippingElementDetector) r.getPipeline()).getMat());
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Position", ((ShippingElementDetector) r.getPipeline()).getBarcodePosition().name());
            telemetry.addData("Active Mat", ((ShippingElementDetector) r.getPipeline()).getMat());
            telemetry.update();

            currentA = gamepad1.a;
            if(currentA && currentA != prevA) {
                ((ShippingElementDetector) r.getPipeline()).loopMats();
            }
            prevA = currentA;
        }
    }
}
