package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robot.Constants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot {

    DcMotorEx intakeMotor;
    DcMotorEx deliveryMotor;
    DcMotorEx carouselMotor;
    Servo deliveryServo;

    OpenCvWebcam webcam;
    OpenCvPipeline pipeline;
    DeliveryArmControl delivery;

    public Robot(HardwareMap hardwareMap, boolean initializeVision, OpenCvPipeline visionPipeline) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        deliveryMotor = hardwareMap.get(DcMotorEx.class, DELIVERY_MOTOR);
        deliveryServo = hardwareMap.get(Servo.class, DELIVERY_SERVO);
        carouselMotor = hardwareMap.get(DcMotorEx.class, CAROUSEL_MOTOR);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        delivery = new DeliveryArmControl((int) DELIVERY_STOWED_COUNTS, (int) DELIVERY_LOW_COUNTS, (int) DELIVERY_MIDDLE_COUNTS, (int) DELIVERY_EXTENDED_COUNTS, DELIVERY_SPEED, deliveryMotor);

        //TODO: Reverse any needed motors here




        if(initializeVision) {
            //Webcam initialization
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
            pipeline = visionPipeline;

            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
                @Override
                public void onError(int errorCode) { //Called if the camera can't be opened
                    }
            });
        }

    }

    public void deliverServoDeliver() {
        deliveryServo.setPosition(DELIVERY_DELIVER_POS_SERVO);
    }

    public void deliverServoStow() {
        deliveryServo.setPosition(DELIVERY_STOWED_POS_SERVO);
    }

    public void lowerDeliveryArm() {
         deliveryMotor.setPower(-DELIVERY_SPEED);
    }

    public void raiseDeliveryArm() {
        deliveryMotor.setPower(DELIVERY_SPEED);
    }

    public void stopDeliveryArm() {
        deliveryMotor.setPower(0);
    }

    //TODO: Implement touch sensors for the delivery to automatically stop (If we're even going to have them)

    public void runIntakeForward() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public void runIntakeBackwards() {
        intakeMotor.setPower(-INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void carouselClockwise() {
        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, CAROUSEL_RPM);
    }

    public void carouselCounterClockwise() {
        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, -CAROUSEL_RPM);
    }

    public void carouselStop() {
        carouselMotor.setPower(0);
    }


    public void setMotorRPM(DcMotorEx motor, double ticks, double RPM) {
        motor.setVelocity(RPM / 60 * ticks);
    }

    public OpenCvPipeline getPipeline() {
        return pipeline;
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getCarouselMotor() {
        return carouselMotor;
    }

    public Servo getDeliveryServo() {
        return deliveryServo;
    }

    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    public DeliveryArmControl getDelivery() {
        return delivery;
    }
}
