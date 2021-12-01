package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.*;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_DISTANCE;
import static org.firstinspires.ftc.teamcode.Robot.Constants.ELEMENT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.ArmModes.*;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_DELIVER_POS_SERVO;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_SERVO;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DELIVERY_STOWED_POS_SERVO;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryServoPositions.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryArmControl {
    int stowedCounts;
    int lowCounts;
    int midCounts;
    int extendedCounts;
    int intakeCounts;
    double power;

    DcMotorEx delivery;
    Servo deliveryServo;

    ArmModes mode;
    DeliveryPositions slidePosition;
    DeliveryServoPositions servoPosition;

    DistanceSensor distanceSensor;

    public DeliveryArmControl(int stowedCounts, int lowCounts, int midCounts, int extendedCounts, int intakeCounts, double power, DcMotorEx delivery, HardwareMap hardwareMap) {
        this.stowedCounts = stowedCounts;
        this.lowCounts = lowCounts;
        this.midCounts = midCounts;
        this.extendedCounts = extendedCounts;
        this.intakeCounts = intakeCounts;
        this.power = power;
        this.delivery = delivery;
        deliveryServo = hardwareMap.get(Servo.class, DELIVERY_SERVO);
        distanceSensor = hardwareMap.get(DistanceSensor.class, DELIVERY_DISTANCE);

        //this.delivery.setDirection(REVERSE);

        this.delivery.setZeroPowerBehavior(BRAKE);
        this.delivery.setMode(STOP_AND_RESET_ENCODER);
        //this.delivery.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        deliveryServo.setPosition(DELIVERY_STOWED_POS_SERVO);
        servoPosition = STOWED_SERVO;

        mode = AUTOMATIC;
        slidePosition = INTAKE;
    }

    public void moveDelivery(DeliveryPositions position) {
        if(mode == MANUAL) {
            mode = AUTOMATIC;
        }

        int chosenCounts = 0;
        if(position == STOWED) {
            chosenCounts = stowedCounts;
            deliverServoStow();
        } else if(position == LOW) {
            chosenCounts = lowCounts;
        } else if(position == MID) {
            chosenCounts = midCounts;
        } else if(position == HIGH) {
            chosenCounts = extendedCounts;
        } else if (position == INTAKE) {
            chosenCounts =  intakeCounts;
        }

        int currentPower = delivery.getCurrentPosition() < stowedCounts ? 1 : -1;

        delivery.setPower(power * currentPower);
        delivery.setTargetPosition(chosenCounts);
        delivery.setMode(RUN_TO_POSITION);

        this.slidePosition = position;
    }

    public void resetEncoder() {
        delivery.setMode(STOP_AND_RESET_ENCODER);
        delivery.setTargetPosition(0);
        delivery.setMode(RUN_USING_ENCODER);
        //delivery.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mode = AUTOMATIC;
        slidePosition = INTAKE;
    }

    public void manualDeliveryMove(boolean up, boolean down) {
        if(mode == AUTOMATIC) {
            delivery.setMode(RUN_USING_ENCODER);
            mode = MANUAL;
        }
        if(up) {
            delivery.setPower(power);
        } else if (down) {
            delivery.setPower(-power);
        } else {
            delivery.setPower(0);
        }
    }

    public void deliverServoDeliver() {
        deliveryServo.setPosition(DELIVERY_DELIVER_POS_SERVO);
        servoPosition = DELIVER_SERVO;
    }

    public void deliverServoStow() {
        deliveryServo.setPosition(DELIVERY_STOWED_POS_SERVO);
        servoPosition = STOWED_SERVO;
    }

    public boolean isElementLoaded() {
        return distanceSensor.getDistance(MM) < ELEMENT_THRESHOLD ? true : false;
    }
    public boolean isDistanceError() {
        return distanceSensor.getDistance(MM) > 200 ? true : false;
    }
    public DeliveryServoPositions getServoPosition() {
        return servoPosition;
    }
    public DcMotorEx getDeliveryMotor() {
        return delivery;
    }
    public DeliveryPositions getSlidePosition() {
        return slidePosition;
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
    public Servo getDeliveryServo() {
        return deliveryServo;
    }
    public DistanceSensor getDistanceSensor() {
        return distanceSensor;
    }

    public enum DeliveryPositions {
        STOWED, LOW, MID, HIGH, INTAKE
    }

    public enum DeliveryServoPositions {
        STOWED_SERVO, DELIVER_SERVO
    }

    public enum ArmModes {
        AUTOMATIC, MANUAL
    }
}
