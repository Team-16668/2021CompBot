package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {
    //Constants for the intake and delivery
    public static final double DELIVERY_INTAKE_POS_SERVO = 0.95;
    public static final double DELIVERY_STOWED_POS_SERVO = 0.5;
    public static final double DELIVERY_DELIVER_POS_SERVO = 0.1;
    public static final int DELIVERY_SERVO_WAIT_TIME = 1000;
    public static final double DELIVERY_EXTENDED_COUNTS = 850;
    public static final double DELIVERY_MIDDLE_COUNTS = 600;
    public static final double DELIVERY_LOW_COUNTS = 325;
    public static final double DELIVERY_STOWED_COUNTS = 200;
    public static final double DELIVERY_INTAKE_COUNTS = 0;

    //Distance sensor threshold (in MM)
    public static final double ELEMENT_THRESHOLD = 90;

    //Delivery Motor PIDF
    public static PIDFCoefficients DELIVERY_PID = new PIDFCoefficients(0, 0, 0, 0);

    //Intake Element speed
    public static final double INTAKE_ELEMENT_SPEED = 0.05;

    public static final double INTAKE_SPEED = 1.00;
    public static final double DELIVERY_SPEED = 0.50;
    public static final double CAROUSEL_RPM = 265 * 0.9;
    public static final double CAROUSEL_FAST_RPM = 435 * 0.9;
    public static final double MOTOR_TICKS_435_RPM = 384.5;

    //Different Drivetrain speeds
    public static final double FAST_SPEED = 0.90;
    public static final double NORMAL_SPEED = 0.55;
    public static final double SLOW_SPEED = 0.25;

    //Patterns for the lights
    public static final BlinkinPattern LOADED_PATTERN = GREEN;
    public static final BlinkinPattern UNLOADED_PATTERN = BLACK;
    public static final BlinkinPattern ERROR_PATTERN = RED;

    //Hardware ID's
    public static final String LEFT_FRONT = "left_front";
    public static final String LEFT_BACK = "left_back";
    public static final String RIGHT_FRONT = "right_front";
    public static final String RIGHT_BACK = "right_back";
    public static final String INTAKE_MOTOR = "intake_motor";
    public static final String DELIVERY_MOTOR = "delivery_motor";
    public static final String CAROUSEL_MOTOR = "carousel_motor";
    public static final String DELIVERY_SERVO = "delivery_servo";
    public static final String IMU = "imu";
    public static final String BACK_CAM = "back_webcam";
    public static final String FRONT_CAM = "front_webcam";
    public static final String DELIVERY_DISTANCE = "delivery_distance";
    public static final String LIGHTS = "lights";
}
