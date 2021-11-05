package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robot.Constants.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.IntakeDirections.*;

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

    OpenCvWebcam webcam;
    OpenCvPipeline pipeline;
    DeliveryArmControl delivery;
    IntakeDirections intakeDirection;
    FtcDashboard dashboard;

    public Robot(HardwareMap hardwareMap, boolean initializeVision, OpenCvPipeline visionPipeline) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        deliveryMotor = hardwareMap.get(DcMotorEx.class, DELIVERY_MOTOR);
        carouselMotor = hardwareMap.get(DcMotorEx.class, CAROUSEL_MOTOR);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        delivery = new DeliveryArmControl((int) DELIVERY_STOWED_COUNTS, (int) DELIVERY_LOW_COUNTS, (int) DELIVERY_MIDDLE_COUNTS, (int) DELIVERY_EXTENDED_COUNTS, DELIVERY_SPEED, deliveryMotor, hardwareMap);
        intakeDirection = FORWARD;

        dashboard = FtcDashboard.getInstance();

        //TODO: Reverse any needed motors here


        if(initializeVision) {
            //Webcam initialization
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
            pipeline = visionPipeline;

            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT); }
                @Override
                public void onError(int errorCode) { }});

        }

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

    public void runIntakeForward() {
        intakeMotor.setPower(INTAKE_SPEED);
        intakeDirection = FORWARD;
    }

    public void runIntakeBackwards() {
        intakeMotor.setPower(-INTAKE_SPEED);
        intakeDirection = BACKWARD;
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        intakeDirection = STOPPED;
    }

    public void carouselClockwise() {
        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, CAROUSEL_RPM);
    }

    public void carouselCounterClockwise() {
        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, -CAROUSEL_RPM);
    }

    public void stopCarousel() {
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

    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    public DeliveryArmControl getDeliveryControl() {
        return delivery;
    }

    public IntakeDirections getIntakeDirection() {
        return intakeDirection;
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public enum IntakeDirections {
        FORWARD, BACKWARD, STOPPED
    }
}
