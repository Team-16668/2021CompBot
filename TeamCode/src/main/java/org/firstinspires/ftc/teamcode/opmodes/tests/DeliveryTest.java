package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Delivery Test")
public class DeliveryTest extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                r.getDeliveryControl().deliverServoDeliver();
            } else if(gamepad1.dpad_down) {
                r.getDeliveryControl().deliverServoStow();
            }

            telemetry.addData("Servo Position", r.getDeliveryControl().getDeliveryServo().getPosition());
            telemetry.update();
        }
    }
}
