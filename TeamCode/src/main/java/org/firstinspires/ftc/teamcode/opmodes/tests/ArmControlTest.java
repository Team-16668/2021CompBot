package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.Robot.ArmModes.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryServoPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.IntakeDirections.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot.DeliveryServoPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Robot.IntakeDirections;

@TeleOp(name="Arm Control Test")
public class ArmControlTest extends LinearOpMode {

    Robot r;

    boolean prevGamepadA = false, currGamepadA, prevGamepadX = false, currGamepadX, prevGamepadB = false, currGamepadB;
    boolean dpadUp, dpadDown;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);

        waitForStart();

        while(opModeIsActive()) {
            currGamepadA = gamepad1.a;
            currGamepadB = gamepad1.b;
            currGamepadX = gamepad1.x;
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            if(currGamepadA && prevGamepadA != currGamepadA && r.getDeliveryControl().getMode() == AUTOMATIC) {
                if(r.getDeliveryControl().getSlidePosition() == HIGH) {
                    r.getDeliveryControl().moveDelivery(STOWED);
                    r.runIntakeForward();
                } else {
                    r.getDeliveryControl().moveDelivery(HIGH);
                    r.stopIntake();
                }
            } else if((dpadUp || dpadDown)) {
                r.getDeliveryControl().manualDeliveryMove(dpadUp, dpadDown);
            }

            if(gamepad1.y) {
                r.getDeliveryControl().resetEncoder();
            }

            if(currGamepadX && currGamepadX != prevGamepadX) {
                if(r.getDeliveryControl().getServoPosition() == STOWED_SERVO)
                    r.getDeliveryControl().deliverServoStow();
                else if(r.getDeliveryControl().getServoPosition() == DELIVER_SERVO)
                    r.getDeliveryControl().deliverServoDeliver();
            }

            if(currGamepadB && currGamepadB != prevGamepadB) {
                if(r.getIntakeDirection() == FORWARD) {
                    r.runIntakeBackwards();
                } else if(r.getIntakeDirection() == BACKWARD) {
                    r.runIntakeForward();
                }
            }

            prevGamepadA = currGamepadA;
            prevGamepadX = currGamepadX;
            prevGamepadB = currGamepadB;

            telemetry.addData("Mode", r.getDeliveryControl().getMode());
            telemetry.addData("Position", r.getDeliveryControl().getSlidePosition());
            telemetry.addData("Encoder counts", r.getDeliveryControl().getEncoderCounts());
            telemetry.addData("Motor power", r.getDeliveryControl().getPower());
            telemetry.update();
        }
    }
}
