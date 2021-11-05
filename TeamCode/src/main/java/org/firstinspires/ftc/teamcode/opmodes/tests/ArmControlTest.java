package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.Robot.ArmModes.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryServoPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.IntakeDirections.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot.DeliveryPositions;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Arm Control Test")
public class ArmControlTest extends LinearOpMode {

    Robot r;

    boolean prevDeliveryAuto = false, currDeliveryAuto, prevServoSwitch = false, currServoSwitch, prevIntakeSwitch = false, currIntakeSwitch;
    boolean deliveryUpManual, deliveryDownManual;
    boolean prevLowerDelivery = false, currLowerDelivery, prevRaiseDelivery = false, currRaiseDelivery;
    boolean resetEncoder;

    DeliveryPositions automaticPosition = HIGH;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, false, null);

        waitForStart();

        //TODO: Add a way to change which level you're going to

        while(opModeIsActive()) {
            armControlLoop();
        }
    }

    private void armControlLoop() {
        /**
         * CONTROLS
         *  A: Automatic toggle between low and high
         *  B: Switch intake direction. Will also start a stopped intake in the forward direction
         *  X: Move the servo between the delivered and stowed position
         *  Y: Reset the encoder of the delivery arm
         *  Dpad (Up and down): Manually move delivery arm
         *  Left trigger: Lower the level of the delivery
         *  Right trigger: Raise the level of the delivery
         */

        currDeliveryAuto = gamepad1.a;
        currIntakeSwitch = gamepad1.b;
        currServoSwitch = gamepad1.x;
        resetEncoder = gamepad1.y;

        deliveryUpManual = gamepad1.dpad_up;
        deliveryDownManual = gamepad1.dpad_down;

        currLowerDelivery = gamepad1.left_bumper;
        currRaiseDelivery = gamepad1.right_bumper;

        //Logic for Automatically moving the delivery arm
        if(currDeliveryAuto && prevDeliveryAuto != currDeliveryAuto && r.getDeliveryControl().getMode() == AUTOMATIC) {
            if(r.getDeliveryControl().getSlidePosition() != STOWED) {
                r.getDeliveryControl().moveDelivery(STOWED);
                r.runIntakeForward();
            } else {
                r.getDeliveryControl().moveDelivery(automaticPosition);
                r.stopIntake();
            }
        } else if((deliveryUpManual || deliveryDownManual)) {
            //Logic for manual control of the delivery arm
            r.getDeliveryControl().manualDeliveryMove(deliveryUpManual, deliveryDownManual);
        }

        //Switch the level that the intake automatically goes to
        if(currRaiseDelivery && currRaiseDelivery != prevRaiseDelivery) {
            if(automaticPosition == LOW) {
                automaticPosition = MID;
            } else if(automaticPosition == MID) {
                automaticPosition = HIGH;
            }
        } else if(currLowerDelivery && currLowerDelivery != prevLowerDelivery) {
            if(automaticPosition == HIGH) {
                automaticPosition = MID;
            } else if(automaticPosition == MID) {
                automaticPosition = LOW;
            }
        }


        //Reset the encoder when it has been manually moved to the bottom (hopefully this doesn't need to be used
        if(resetEncoder) {
            r.getDeliveryControl().resetEncoder();
        }

        //Move the servo between the two positions
        if(currServoSwitch && currServoSwitch != prevServoSwitch) {
            if(r.getDeliveryControl().getServoPosition() == STOWED_SERVO)
                r.getDeliveryControl().deliverServoStow();
            else if(r.getDeliveryControl().getServoPosition() == DELIVER_SERVO)
                r.getDeliveryControl().deliverServoDeliver();
        }

        //Switch the direction of the intake
        if(currIntakeSwitch && currIntakeSwitch != prevIntakeSwitch) {
            if(r.getIntakeDirection() == FORWARD || r.getIntakeDirection() == STOPPED) {
                r.runIntakeBackwards();
            } else if(r.getIntakeDirection() == BACKWARD) {
                r.runIntakeForward();
            }
        }

        prevDeliveryAuto = currDeliveryAuto;
        prevServoSwitch = currServoSwitch;
        prevIntakeSwitch = currIntakeSwitch;
        prevLowerDelivery = currLowerDelivery;
        prevRaiseDelivery = currRaiseDelivery;

        telemetry.addData("Mode", r.getDeliveryControl().getMode());
        telemetry.addData("Position", r.getDeliveryControl().getSlidePosition());
        telemetry.addData("Encoder counts", r.getDeliveryControl().getEncoderCounts());
        telemetry.addData("Motor power", r.getDeliveryControl().getPower());
        telemetry.update();
    }
}
