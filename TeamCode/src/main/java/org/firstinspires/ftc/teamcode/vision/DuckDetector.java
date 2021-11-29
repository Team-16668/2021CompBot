package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.Robot.Alliance.*;
import static org.firstinspires.ftc.teamcode.Robot.Alliance.Alliances.*;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.HSV;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INPUT;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INRANGE;

import static java.lang.Math.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class DuckDetector extends OpenCvPipeline {
    
    //TODO: replace this with the correct color range
    public static int hue1 = 25;
    public static int saturation1 = 100;
    public static int value1 = 0;
    public static int hue2 = 30;
    public static int saturation2 = 255;
    public static int value2 = 255;
    public Scalar lowerB = new Scalar(hue1, saturation1, value1);
    public Scalar upperB = new Scalar(hue2, saturation2, value2);

    private Mat normalImage = new Mat();
    private Mat HSVMat = new Mat();
    private Mat inRangeMat = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private MatOfPoint biggestContour;
    private Rect boundingRect;

    Pose2d robotPose = new Pose2d();
    Pose2d correctedRobotPose = new Pose2d();
    Pose2d correctedCameraPose = new Pose2d();
    Pose2d fieldCameraPose = new Pose2d();

    Vector2d goToPoint = new Vector2d();
    Vector2d relativeCameraPoint;

    double correctedAngleToDuck = 0;
    double distanceFromCenter;
    double cameraAngle;
    double centerX;
    double centerY;
    double yLine;

    boolean duckDetected;

    private ShippingElementDetector.Mats activeMat = INPUT;

    Telemetry telemetry;
    SampleMecanumDrive drive;


    /**
     *
     * @param relativeCameraPoint the relative point of the camera on the robot based on the following diagram
     *                                         (y+)
     *                                        |
     *                                        |
     *                 ---------------------------------------------
     *                 |                      |                    |
     *                 |                      |                    |
     *                 |                      |                    |
     *                 |                      |                    |  (Front of  bot)
     *-----------------|----------------------|--------------------|--------------------- (x+)
     *                 |                      |                    |
     *                 |                      |                    |
     *                 |                      |                    |
     *                 |                      |                    |
     *                 ---------------------------------------------
     *                                        |
     *                                        |
     *                                        |

     * @param yLine the y position that the robot will ultimately move towards
     */
    public DuckDetector(Vector2d relativeCameraPoint, double yLine, Telemetry telemetry, SampleMecanumDrive drive) {
        this.relativeCameraPoint = relativeCameraPoint;
        this.yLine = yLine;
        this.drive = drive;

        goToPoint = new Vector2d(robotPose.getX() + relativeCameraPoint.getX(), yLine);

        //Get the distance from the center of the robot
        distanceFromCenter = sqrt(pow(relativeCameraPoint.getX(), 2) + pow(relativeCameraPoint.getY(), 2));
        //Get the angle of the center to the camera (using atan2)
        cameraAngle = atan2(relativeCameraPoint.getY(), relativeCameraPoint.getX());

        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat mat) {

        int imageWidth = mat.width();
        int imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;
    }

    @Override
    public Mat processFrame(Mat input) {
        setRobotPose(drive.getPoseEstimate());
        lowerB = new Scalar(hue1, saturation1, value1);
        upperB = new Scalar(hue2, saturation2, value2);

        normalImage = input;
        //Convert to HSV, then to binary Mat, then find the contours of everyhting
        //that's yellow enough in the field of view
        Imgproc.cvtColor(normalImage, HSVMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(HSVMat, lowerB, upperB, inRangeMat);

        contours.clear();

        Imgproc.findContours(inRangeMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(getActiveMat(), contours, -1, new Scalar(255, 255, 0));

        //Filter the biggest contour
        if(!contours.isEmpty()) {
            biggestContour = Collections.max(contours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            boundingRect = Imgproc.boundingRect(biggestContour);
            Imgproc.rectangle(getActiveMat(), boundingRect, new Scalar(255, 0, 0), 1);
        } else {
            boundingRect = null;
        }

        if(boundingRect != null) {
            Point duckCenter = new Point(boundingRect.x + (0.5 * boundingRect.width), boundingRect.y + (boundingRect.height));
            Imgproc.drawMarker(getActiveMat(), duckCenter, new Scalar(0, 0, 255));

            duckDetected = true;

            Point imageCenter = new Point(centerX, getActiveMat().height());

            Imgproc.line(getActiveMat(), imageCenter, new Point(centerX, 0), new Scalar(255, 0, 0), 3);

            Point correctedDuckCenter = new Point(
                    -(imageCenter.x - duckCenter.x),
                    -(duckCenter.y - imageCenter.y)
            );

            double rawAngleToDuck = atan2(correctedDuckCenter.y, correctedDuckCenter.x);
            correctedAngleToDuck = correctedDuckCenter.x > 0 ? toRadians(90) - rawAngleToDuck : - (rawAngleToDuck - toRadians(90));

            final double FUDGE_FACTOR = toRadians(5);

            double xOffset = abs(yLine - (fieldCameraPose.getY())) * tan(abs(correctedAngleToDuck));

            //Doing logic to determine how the value should be added assuming pointing towards red alliance wall
            xOffset *= correctedAngleToDuck < 0 ? 1 : -1;

            //Adjust if alliance is blue
            xOffset *= alliance == BLUE ? -1 : 1;

            goToPoint = new Vector2d(fieldCameraPose.getX() + xOffset, goToPoint.getY());

//            telemetry.addData("CORRECT: rawAngleToDuck", rawAngleToDuck);
//            telemetry.addData("CORRECT: correctedAngleToDuck", correctedAngleToDuck);
//            telemetry.addData("CORRECT: corrected Duck Pos", correctedDuckCenter.x + ", " + correctedDuckCenter.y);
//            telemetry.addData("CORRECT: xOffset", xOffset);
        } else {
            duckDetected = false;
        }

//        telemetry.addData("CORRECT: Duck detected", duckDetected);
//        telemetry.addData("CORRECT: Corrected robot pose", "(" + correctedRobotPose.getX() + ", " + correctedRobotPose.getY() + ", " + correctedRobotPose.getHeading());
//        telemetry.addData("CORRECT: Camera angle", cameraAngle);
//        telemetry.addData("CORRECT: Corrected camera pose", "(" + correctedCameraPose.getX() + ", " + correctedCameraPose.getY() + ", " + correctedCameraPose.getHeading());
//        telemetry.addData("CORRECT: Field camera pose", "(" + fieldCameraPose.getX() + ", " + fieldCameraPose.getY() + ", " + fieldCameraPose.getHeading());
//        telemetry.addData("CORRECT goToPoint", "(" + goToPoint.getX() + ", " + goToPoint.getY() + ")");
//        telemetry.update();


        return input;
    }

    public void setRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
        //Corrected robot pose is the pose of the robot on a normal coordiante plane
        //  (looking from the top with the audience in the third and fourth quadrants)
        this.correctedRobotPose = new Pose2d(- robotPose.getY(), robotPose.getX(), robotPose.getHeading() + toRadians(90));
        calculateCameraPose();
        fieldCameraPose = new Pose2d(correctedCameraPose.getY() + correctedRobotPose.getY(), -(correctedCameraPose.getX() + correctedRobotPose.getX()), correctedRobotPose.getHeading() - toRadians(90));
    }

    public void calculateCameraPose() {
        double angle = correctedRobotPose.getHeading();

        correctedCameraPose = new Pose2d(
                cos(angle) * distanceFromCenter,
                sin(angle) * distanceFromCenter,
                correctedRobotPose.getHeading()
        );
    }

    public boolean isDuckDetected() {
        return duckDetected;
    }

    public Vector2d getGoToPoint() {
        return goToPoint;
    }
    public double getCorrectedAngleToDuck() {
        return correctedAngleToDuck;
    }

    private Mat getActiveMat(){
        //This just returns the Mat variable that corresponds to each Enum state in Mats
        if(activeMat == INPUT) {
            return normalImage;
        } else if(activeMat == ShippingElementDetector.Mats.HSV) {
            return HSVMat;
        } else {
            //activeMat is INRANGE
            return inRangeMat;
        }
    }

    public void loopMats() {
        if(activeMat == INPUT) {
            activeMat = HSV;
        } else if (activeMat == HSV) {
            activeMat = INRANGE;
        } else if (activeMat == INRANGE) {
            activeMat = INPUT;
        }
    }
}
