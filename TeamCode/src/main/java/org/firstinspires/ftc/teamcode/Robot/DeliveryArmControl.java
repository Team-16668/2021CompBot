package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.ArmModes.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryPositions.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DeliveryArmControl {
    int stowedCounts;
    int lowCounts;
    int midCounts;
    int extendedCounts;
    double power;

    DcMotorEx delivery;

    ArmModes mode;
    DeliveryPositions position;

    public DeliveryArmControl(int stowedCounts, int lowCounts, int midCounts, int extendedCounts, double power, DcMotorEx delivery) {
        this.stowedCounts = stowedCounts;
        this.lowCounts = lowCounts;
        this.midCounts = midCounts;
        this.extendedCounts = extendedCounts;
        this.power = power;
        this.delivery = delivery;

        this.delivery.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.delivery.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.delivery.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mode = AUTOMATIC;
    }

    public void moveDelivery(DeliveryPositions position) {
        if(mode == MANUAL) {
            mode = AUTOMATIC;
        }

        int chosenCounts = 0;
        if(position == STOWED) {
            chosenCounts = stowedCounts;
        } else if(position == LOW) {
            chosenCounts = lowCounts;
        } else if(position == MID) {
            chosenCounts = midCounts;
        } else if(position == HIGH) {
            chosenCounts = extendedCounts;
        }

        int currentPower = delivery.getCurrentPosition() > stowedCounts ? 1 : -1;

        delivery.setPower(power * currentPower);
        delivery.setTargetPosition(chosenCounts);

        this.position = position;
    }

    public void resetEncoder() {
        delivery.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delivery.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mode = AUTOMATIC;
        position = STOWED;
    }

    public void manualDeliveryMove(boolean up, boolean down) {
        if(mode == AUTOMATIC) {
            delivery.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mode = MANUAL;
        }

        if(up) {
            delivery.setPower(power);
        } else {
            delivery.setPower(-power);
        }
    }

    public DcMotorEx getDeliveryMotor() {
        return delivery;
    }
    public DeliveryPositions getPosition() {
        return position;
    }
    public ArmModes getMode() {
        return mode;
    }
    public int getEncoderCounts() {
        return delivery.getCurrentPosition();
    }
    public double getPower() {
        return delivery.getPower();
    }
}
