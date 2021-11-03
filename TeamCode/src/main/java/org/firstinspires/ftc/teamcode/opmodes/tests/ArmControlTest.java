package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.Robot.ArmModes.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryPositions.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot.ArmModes;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Arm Control Test")
public class ArmControlTest extends LinearOpMode {

    Robot r;

    boolean prevGamepadX, currGamepadX;
    boolean dpadUp, dpadDown;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);

        waitForStart();

        while(opModeIsActive()) {
            currGamepadX = gamepad1.x;
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            if(currGamepadX && prevGamepadX != currGamepadX && r.getDelivery().getMode() == AUTOMATIC) {
                if(r.getDelivery().getPosition() == HIGH) {
                    r.getDelivery().moveDelivery(STOWED);
                } else {
                    r.getDelivery().moveDelivery(HIGH);
                }
            } else if((dpadUp || dpadDown)) {
                r.getDelivery().manualDeliveryMove(dpadUp, dpadDown);
            }

            if(gamepad1.y) {
                r.getDelivery().resetEncoder();
            }

            currGamepadX = prevGamepadX;

            telemetry.addData("Mode", r.getDelivery().getMode());
            telemetry.addData("Position", r.getDelivery().getPosition());
            telemetry.addData("Encoder counts", r.getDelivery().getEncoderCounts());
            telemetry.addData("Motor power", r.getDelivery().getPower());
            telemetry.update();
        }
    }
}
