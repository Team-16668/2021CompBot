package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.BLUE;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.RED;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.alliance;
import static org.firstinspires.ftc.teamcode.Robot.Constants.*;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.ArmModes.MANUAL;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.LOW;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.MID;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.STOWED;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryServoPositions.DELIVER_SERVO;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryServoPositions.STOWED_SERVO;
import static org.firstinspires.ftc.teamcode.Robot.Robot.CarouselSpeeds.*;
import static org.firstinspires.ftc.teamcode.Robot.Robot.IntakeDirections.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
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

    boolean prevDeliveryAuto = false, currDeliveryAuto, prevServoSwitch = false, currServoSwitch;
    boolean deliveryUpManual, deliveryDownManual;
    boolean prevLowerDelivery = false, currLowerDelivery, prevRaiseDelivery = false, currRaiseDelivery;
    boolean resetEncoder;
    boolean intakeForward, intakeBackward;
    boolean intakeStopped = true;

    DeliveryPositions automaticPosition = HIGH;
    CarouselSpeeds carouselSpeed = NORMAL;


    public Robot(HardwareMap hardwareMap, boolean initializeVision, OpenCvPipeline visionPipeline) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        deliveryMotor = hardwareMap.get(DcMotorEx.class, DELIVERY_MOTOR);
        carouselMotor = hardwareMap.get(DcMotorEx.class, CAROUSEL_MOTOR);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        delivery = new DeliveryArmControl((int) DELIVERY_STOWED_COUNTS, (int) DELIVERY_LOW_COUNTS, (int) DELIVERY_MIDDLE_COUNTS, (int) DELIVERY_EXTENDED_COUNTS, (int) DELIVERY_INTAKE_COUNTS, DELIVERY_SPEED, deliveryMotor, hardwareMap);
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

    /**
     * Main loop for control of the wheels and driving around (including speed changes)
     * @param drive The SampleMecanumDrive instance in the teleop program.
     * @param gamepad
     */
    public void driveControlLoop(SampleMecanumDrive drive, Gamepad gamepad) {
        double multiplier;

        if(gamepad.left_bumper) {
            multiplier = SLOW_SPEED;
        } else if(gamepad.right_bumper) {
            multiplier = FAST_SPEED;
        } else {
            multiplier = NORMAL_SPEED;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * multiplier,
                        -gamepad.left_stick_x * multiplier,
                        -gamepad.right_stick_x * multiplier
                ));
    }

    /**
     * Main loop for the control of the delivery arm, as well as the intake
     * UPDATE on 11/10/21:
     *  Changing controls for the slide to only go down all the way when intaking.
     *
     * @param gamepad Gamepad (preferably 2)
     */
    public void armControlLoopTeleOp(Gamepad gamepad, Telemetry telemetry) {
        /**
         * CONTROLS
         *  A: Automatic toggle between low and high
         *  X: Move the servo between the delivered and stowed position
         *  Y: Reset the encoder of the delivery arm
         *  Dpad (Up and down): Manually move delivery arm
         *  Left bumper: Lower the level of the delivery
         *  Right bumper: Raise the level of the delivery
         *  Right trigger: Intake forward
         *  Left trigger: Intake Backward
         */

        currDeliveryAuto = gamepad.a;
        currServoSwitch = gamepad.x;
        resetEncoder = gamepad.y;

        deliveryUpManual = gamepad.dpad_up;
        deliveryDownManual = gamepad.dpad_down;

        currLowerDelivery = gamepad.left_bumper;
        currRaiseDelivery = gamepad.right_bumper;

        intakeForward = gamepad.right_trigger > 0;
        intakeBackward = gamepad.left_trigger > 0;

        //Logic for Automatically moving the delivery arm
        if(currDeliveryAuto && prevDeliveryAuto != currDeliveryAuto && getDeliveryControl().getServoPosition() == STOWED_SERVO) {
            if(getDeliveryControl().getSlidePosition() != STOWED) {
                getDeliveryControl().moveDelivery(STOWED);
                //runIntakeForward();
            } else {
                getDeliveryControl().moveDelivery(automaticPosition);
                //stopIntake();
            }
        } else if((deliveryUpManual || deliveryDownManual)) {
            //Logic for manual control of the delivery arm
            getDeliveryControl().manualDeliveryMove(deliveryUpManual, deliveryDownManual);
        } else if (getDeliveryControl().getMode() == MANUAL && !deliveryUpManual && !deliveryDownManual) {
            getDeliveryControl().manualDeliveryMove(deliveryUpManual, deliveryDownManual);
        }

        //Switch the level that the intake automatically goes to
        if(currRaiseDelivery && currRaiseDelivery != prevRaiseDelivery) {
            if(automaticPosition == LOW) {
                automaticPosition = MID;
            } else if(automaticPosition == MID) {
                automaticPosition = HIGH;
            }

            if(getDeliveryControl().getSlidePosition() != STOWED) {
                getDeliveryControl().moveDelivery(automaticPosition);
            }
        } else if(currLowerDelivery && currLowerDelivery != prevLowerDelivery) {
            if(automaticPosition == HIGH) {
                automaticPosition = MID;
            } else if(automaticPosition == MID) {
                automaticPosition = LOW;
            }

            if(getDeliveryControl().getSlidePosition() != STOWED) {
                getDeliveryControl().moveDelivery(automaticPosition);
            }
        }


        //Reset the encoder when it has been manually moved to the bottom (hopefully this doesn't need to be used
        if(resetEncoder) {
            getDeliveryControl().resetEncoder();
        }

        //Move the servo between the two positions
        if(currServoSwitch && currServoSwitch != prevServoSwitch) {
            if(getDeliveryControl().getSlidePosition() != STOWED && getDeliveryControl().getSlidePosition() != INTAKE) {
                if (getDeliveryControl().getServoPosition() == STOWED_SERVO) {
                    getDeliveryControl().deliverServoDeliver();
                }else if (getDeliveryControl().getServoPosition() == DELIVER_SERVO) {
                    getDeliveryControl().deliverServoStow();
                }
            }
        }

        telemetry.update();

        //Switch the direction of the intake
        if(intakeForward) {
            runIntakeForward();
            getDeliveryControl().moveDelivery(INTAKE);
            intakeStopped = false;
        } else if(intakeBackward) {
            runIntakeBackwards();
            getDeliveryControl().moveDelivery(INTAKE);
            intakeStopped = false;
        } else if(!intakeForward && !intakeBackward && !intakeStopped){
            intakeStopped = true;
            stopIntake();
            getDeliveryControl().moveDelivery(STOWED);
        }

        prevDeliveryAuto = currDeliveryAuto;
        prevServoSwitch = currServoSwitch;
        prevLowerDelivery = currLowerDelivery;
        prevRaiseDelivery = currRaiseDelivery;
    }

    /**
     * Main loop for the control of the carousel wheel
     * @param gamepad gamepad (probably 1)
     */
    public void carouselControlLoop(Gamepad gamepad) {
        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;

        if(rightTrigger > 0) {
            carouselSpeed = FAST;
        } else if(leftTrigger > 0 && rightTrigger == 0){
            carouselSpeed = NORMAL;
        }

        if(leftTrigger > 0) {
            if (alliance == RED) {
                carouselCounterClockwise(carouselSpeed);
            } else if (alliance == BLUE) {
                carouselCounterClockwise(carouselSpeed);
            }
        }else {
            if(getCarouselMotor().getPower() != 0) {
                stopCarousel();
            }
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

    public void carouselClockwise(CarouselSpeeds speed) {
        double rpm = 0;
        if(speed == FAST) {
            rpm = CAROUSEL_FAST_RPM;
        } else if(speed == NORMAL) {
            rpm = CAROUSEL_RPM;
        }

        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, rpm);
    }

    public void carouselCounterClockwise(CarouselSpeeds speed) {
        double rpm = 0;
        if(speed == FAST) {
            rpm = -CAROUSEL_FAST_RPM;
        } else if(speed == NORMAL) {
            rpm = -CAROUSEL_RPM;
        }
        setMotorRPM(carouselMotor, MOTOR_TICKS_435_RPM, rpm);
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

    public enum CarouselSpeeds {
        NORMAL, FAST
    }
}
