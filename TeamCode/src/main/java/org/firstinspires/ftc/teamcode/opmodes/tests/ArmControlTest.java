package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Disabled
@TeleOp(name="Arm Control Test")
public class ArmControlTest extends LinearOpMode {

    Robot r;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);

        waitForStart();

        r.runIntakeForward();

        while(opModeIsActive()) {
            r.armControlLoopTeleOp(gamepad1, telemetry);

            if(gamepad1.a) {
                r.updateDeliveryPID();
            }

            telemetry.addData("Mode", r.getDeliveryControl().getMode());
            telemetry.addData("Position", r.getDeliveryControl().getSlidePosition());
            telemetry.addData("Encoder counts", r.getDeliveryControl().getEncoderCounts());
            telemetry.addData("Motor power", r.getDeliveryControl().getPower());
            telemetry.addData("Motor velocity", r.getDeliveryControl().getDeliveryMotor().getVelocity());
            telemetry.update();
        }
    }
}
