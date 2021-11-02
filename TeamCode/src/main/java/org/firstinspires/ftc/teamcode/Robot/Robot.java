package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    DcMotorEx intakeMotor;
    DcMotorEx deliveryMotor;
    DcMotorEx carouselMotor;
    Servo deliveryServo;

    public Robot(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR);
        deliveryMotor = hardwareMap.get(DcMotorEx.class, Constants.DELIVERY_MOTOR);
        deliveryServo = hardwareMap.get(Servo.class, Constants.DELIVERY_SERVO);
        carouselMotor = hardwareMap.get(DcMotorEx.class, Constants.CAROUSEL_MOTOR);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deliveryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: Reverse any needed motors here

    }

    public void deliverServo() {
        deliveryServo.setPosition(Constants.DELIVERY_DELIVER_POS);
    }

    public void stowDeliveryServo() {
        deliveryServo.setPosition(Constants.DELIVERY_STOWED_POS);
    }

    public void lowerDeliveryArm() {
         deliveryMotor.setPower(-Constants.DELIVERY_SPEED);
    }

    public void raiseDeliveryArm() {
        deliveryMotor.setPower(Constants.DELIVERY_SPEED);
    }

    public void stopDeliveryArm() {
        deliveryMotor.setPower(0);
    }

    //TODO: Implement touch sensors for the delivery to automatically stop (If we're even going to have them)

    public void runIntakeForward() {
        intakeMotor.setPower(Constants.INTAKE_SPEED);
    }

    public void runIntakeBackwards() {
        intakeMotor.setPower(-Constants.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void carouselClockwise() {
        setMotorRPM(carouselMotor, Constants.MOTOR_TICKS_435_RPM, Constants.CAROUSEL_RPM);
    }

    public void carouselCounterClockwise() {
        setMotorRPM(carouselMotor, Constants.MOTOR_TICKS_435_RPM, -Constants.CAROUSEL_RPM);
    }

    public void carouselStop() {
        carouselMotor.setPower(0);
    }



    public void setMotorRPM(DcMotorEx motor, double ticks, double RPM) {
        motor.setVelocity(RPM / 60 * ticks);
    }

}
