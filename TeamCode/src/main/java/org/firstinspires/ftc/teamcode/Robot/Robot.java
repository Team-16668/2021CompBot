package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
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

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.util.concurrent.TimeUnit.MILLISECONDS;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPSIDE_DOWN;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    OpenCvWebcam back_webcam;
    OpenCvPipeline back_pipeline;
    OpenCvWebcam front_webcam;
    OpenCvPipeline front_pipeline;
    DeliveryArmControl delivery;
    IntakeDirections intakeDirection;
    FtcDashboard dashboard;

    boolean prevDeliveryAuto = false, currDeliveryAuto, prevServoSwitch = false, currServoSwitch;
    boolean deliveryUpManual, deliveryDownManual;
    boolean prevLowerDelivery = false, currLowerDelivery, prevRaiseDelivery = false, currRaiseDelivery;
    boolean resetEncoder;
    boolean intakeForward, intakeBackward;
    boolean intakeStopped = true;

    boolean switchedAlliance = false;

    GamepadEx gamepad1;
    GamepadEx gamepad2;

    DeliveryPositions automaticPosition = HIGH;
    CarouselSpeeds carouselSpeed = NORMAL;

    ElapsedTime deliveryTimer;
    boolean waitDeliveryMove = false;

    public Robot(HardwareMap hardwareMap, boolean initializeBackVision, OpenCvPipeline backPipeline, boolean initializeFrontVision, OpenCvPipeline frontPipeline) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        deliveryMotor = hardwareMap.get(DcMotorEx.class, DELIVERY_MOTOR);
        carouselMotor = hardwareMap.get(DcMotorEx.class, CAROUSEL_MOTOR);

        deliveryMotor.setDirection(REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        delivery = new DeliveryArmControl((int) DELIVERY_STOWED_COUNTS, (int) DELIVERY_LOW_COUNTS, (int) DELIVERY_MIDDLE_COUNTS, (int) DELIVERY_EXTENDED_COUNTS, (int) DELIVERY_INTAKE_COUNTS, DELIVERY_SPEED, deliveryMotor, hardwareMap);
        intakeDirection = FORWARD;

        //deliveryMotor.setPIDFCoefficients(RUN_TO_POSITION, DELIVERY_PID);

        dashboard = FtcDashboard.getInstance();

        deliveryTimer = new ElapsedTime();

        if(initializeBackVision && !initializeFrontVision) {
            //Webcam initialization
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            back_webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, BACK_CAM), cameraMonitorViewId);
            back_pipeline = backPipeline;

            back_webcam.setPipeline(back_pipeline);

            back_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() { back_webcam.startStreaming(320, 240, SIDEWAYS_LEFT); }
                @Override
                public void onError(int errorCode) { }});

        } else if(initializeFrontVision && !initializeBackVision) {
            //Webcam initialization
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            front_webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, FRONT_CAM), cameraMonitorViewId);
            front_pipeline = frontPipeline;

            front_webcam.setPipeline(front_pipeline);

            front_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() { front_webcam.startStreaming(320, 240, UPSIDE_DOWN); }
                @Override
                public void onError(int errorCode) { }});
        } else if(initializeBackVision && initializeFrontVision) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            /**
             * This is the only thing you need to do differently when using multiple cameras.
             * Instead of obtaining the camera monitor view and directly passing that to the
             * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
             * on that view in order to split that view into multiple equal-sized child views,
             * and then pass those child views to the constructor.
             */
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

            back_webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, BACK_CAM), viewportContainerIds[0]);
            front_webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, FRONT_CAM), viewportContainerIds[1]);

            back_pipeline = backPipeline;
            front_pipeline = frontPipeline;

            back_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    back_webcam.setPipeline(back_pipeline);
                    back_webcam.startStreaming(320, 240, SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            front_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    front_webcam.setPipeline(front_pipeline);
                    front_webcam.startStreaming(320, 240, UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

        }
    }

    public Robot(HardwareMap hardwareMap, boolean initializeBackVision, OpenCvPipeline backPipeline) {
        this(hardwareMap, initializeBackVision, backPipeline, false, null);
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this(hardwareMap, false, null);

        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);

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
        //TODO: Add code for changing the lights when an element is loaded
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

        telemetry.addData("Element loaded", getDeliveryControl().isElementLoaded());
        telemetry.addData("Distance readout", getDeliveryControl().getDistanceSensor().getDistance(MM));
        telemetry.update();

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
        //TODO: Make all these controls based on GamepadEx
        if( gamepad2.wasJustPressed(A)//currDeliveryAuto && prevDeliveryAuto != currDeliveryAuto) {
        ){
            if(getDeliveryControl().getServoPosition() == STOWED_SERVO) {
                if (getDeliveryControl().getSlidePosition() != STOWED) {
                    getDeliveryControl().moveDelivery(STOWED);
                } else {
                    getDeliveryControl().moveDelivery(automaticPosition);
                }
            } else if(getDeliveryControl().getSlidePosition() != STOWED){
                getDeliveryControl().deliverServoStow();
                deliveryTimer.reset();
                waitDeliveryMove = true;
            }
        } else if((deliveryUpManual || deliveryDownManual)) {
            //Logic for manual control of the delivery arm
            getDeliveryControl().manualDeliveryMove(deliveryUpManual, deliveryDownManual);
        } else if (getDeliveryControl().getMode() == MANUAL && !deliveryUpManual && !deliveryDownManual) {
            getDeliveryControl().manualDeliveryMove(deliveryUpManual, deliveryDownManual);
        }
        telemetry.update();

        if(deliveryTimer.time(MILLISECONDS) > 500 && waitDeliveryMove) {
            getDeliveryControl().moveDelivery(STOWED);
            waitDeliveryMove = false;
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
        if(intakeForward && getDeliveryControl().getServoPosition() == STOWED_SERVO) {
            runIntakeForward();
            getDeliveryControl().moveDelivery(INTAKE);
            intakeStopped = false;
        } else if(intakeBackward && getDeliveryControl().getServoPosition() == STOWED_SERVO) {
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
                carouselClockwise(carouselSpeed);
            }
        }else {
            if(getCarouselMotor().getPower() != 0) {
                stopCarousel();
            }
        }
    }

    public void switchAlliance() {
        if(gamepad1.isDown(A) && gamepad1.isDown(B) && !switchedAlliance) {
            if(alliance == RED) {
                alliance = BLUE;
            } else if(alliance == BLUE) {
                alliance = RED;
            }

            switchedAlliance = true;
        } else if(switchedAlliance && !gamepad1.isDown(A) && !gamepad1.isDown(B)) {
            switchedAlliance = false;
        }
    }

    public boolean moveUntilElement(SampleMecanumDrive drive, double maximumDistance, LinearOpMode opMode) throws InterruptedException {

        Pose2d startPose = drive.getPoseEstimate();

        drive.setWeightedDrivePower(new Pose2d(
                INTAKE_ELEMENT_SPEED,
                0,
                0
        ));
        //TODO: Test stopping if we've moved beyond the maximum distance

        boolean element = false;
        boolean movedTooFar = false;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(!element && !movedTooFar && opMode.opModeIsActive()) {
            drive.update();
            Thread.sleep(50);
            element = getDeliveryControl().isElementLoaded();

            movedTooFar = getDistance(startPose, drive.getPoseEstimate()) > maximumDistance ? true : false;
            if(movedTooFar || timer.seconds() > 5) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                return false;
            }
        }

        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        return true;
    }

    double getDistance(Pose2d pose1, Pose2d pose2) {
        return getDistance(new Vector2d(pose1.getX(), pose1.getY()), new Vector2d(pose2.getX(), pose2.getY()));
    }
    double getDistance(Vector2d vector1, Vector2d vector2) {
        return sqrt(pow(vector1.getX() - vector2.getX(), 2) + pow(vector1.getY() - vector2.getY(), 2));
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

    public OpenCvPipeline getBackPipeline() {
        return back_pipeline;
    }

    public void stopBackCamera() {
        back_webcam.stopStreaming();
        back_webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }

    public void stopFrontCamera() {
        front_webcam.stopStreaming();
        front_webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getCarouselMotor() {
        return carouselMotor;
    }

    public OpenCvWebcam getBackWebcam() {
        return back_webcam;
    }

    public OpenCvWebcam getFrontWebcam() {
        return front_webcam;
    }

    public OpenCvPipeline getFrontPipeline() {
        return front_pipeline;
    }

    public DeliveryArmControl getDeliveryControl() {
        return delivery;
    }

    public IntakeDirections getIntakeDirection() {
        return intakeDirection;
    }

    public PIDFCoefficients getDeliveryPID() {
        return DELIVERY_PID;
    }

    public void updateDeliveryPID() {
        deliveryMotor.setPIDFCoefficients(RUN_TO_POSITION, DELIVERY_PID);
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
