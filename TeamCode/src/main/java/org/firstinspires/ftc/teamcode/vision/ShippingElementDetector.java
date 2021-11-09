package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl;
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

import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.HIGH;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.LOW;
import static org.firstinspires.ftc.teamcode.Robot.DeliveryArmControl.DeliveryPositions.MID;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition.LEFT;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition.NONE;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.BarcodePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.HSV;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INPUT;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INRANGE;

@Config
public class ShippingElementDetector extends OpenCvPipeline {

    //Upper and lower color boundaries for duck
    //TODO: Tune these values
    public static int hue1 = 40;
    public static int saturation1 = 100;
    public static int value1 = 0;
    public static int hue2 = 70;
    public static int saturation2 = 255;
    public static int value2 = 255;
    public Scalar lowerB = new Scalar(hue1, saturation1, value1);
    public Scalar upperB = new Scalar(hue2, saturation2, value2);
    //Lines for the left, middle, and right barcodes
    //TODO: Tune these values also
    public double leftBoundary = 200;
    public double rightBoundary = 100;

    double centerX;
    double centerY;

    private Mat NormalImage;
    private Mat HSVMat;
    private Mat InRangeMat;
    private List<MatOfPoint> contours;
    private MatOfPoint biggestContour;
    private Rect boundingRect;

    private Mats activeMat = INPUT;
    private BarcodePosition barcodePosition = BarcodePosition.LEFT;

    public ShippingElementDetector() {
        NormalImage = new Mat();
        HSVMat = new Mat();
        InRangeMat = new Mat();
        contours = new ArrayList<>();
        biggestContour = new MatOfPoint();
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
        lowerB = new Scalar(hue1, saturation1, value1);
        upperB = new Scalar(hue2, saturation2, value2);

        NormalImage = input;
        //Convert to HSV, then to binary Mat, then find the contours of everyhting
        //that's yellow enough in the field of view
        Imgproc.cvtColor(NormalImage, HSVMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(HSVMat, lowerB, upperB, InRangeMat);

        contours.clear();

        Imgproc.findContours(InRangeMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(getActiveMat(), contours, -1, new Scalar(255, 255, 0));

        //TODO: Eliminate contours found outside of the realistic range (if necessary). This is from a vertical or even horizontal perspective (probably vertical)

        //Filter the biggest contour
        if(!contours.isEmpty()) {
            biggestContour = Collections.max(contours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            boundingRect = Imgproc.boundingRect(biggestContour);
            Imgproc.rectangle(getActiveMat(), boundingRect, new Scalar(255, 0, 0), 3);
        } else {
            boundingRect = null;
        }

        Imgproc.line(getActiveMat(), new Point(0, leftBoundary), new Point(getActiveMat().width(), leftBoundary), new Scalar(255, 0, 0));
        Imgproc.line(getActiveMat(), new Point(0, rightBoundary), new Point(getActiveMat().width(), rightBoundary), new Scalar(0, 255, 0));

        if(boundingRect != null) {
            if(boundingRect.y > leftBoundary) {
                barcodePosition = BarcodePosition.LEFT;
            } else if(boundingRect.y < rightBoundary) {
                barcodePosition = BarcodePosition.RIGHT;
            } else {
                barcodePosition = BarcodePosition.MIDDLE;
            }
        } else {
            barcodePosition = BarcodePosition.NONE;
        }

        return getActiveMat();
    }

    public enum Mats {INPUT, HSV, INRANGE;}

    public void SwitchMat(Mats mat) {
        //This changes the active mat (what will drawn on and displayed to the phone screen)
        activeMat = mat;
    }

    private Mat getActiveMat(){
        //This just returns the Mat variable that corresponds to each Enum state in Mats
        if(activeMat == INPUT) {
            return NormalImage;
        } else if(activeMat == Mats.HSV) {
            return HSVMat;
        } else {
            //activeMat is INRANGE
            return InRangeMat;
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

    public Mats getMat() {
        return activeMat;
    }

    public static enum BarcodePosition {LEFT, MIDDLE, RIGHT, NONE}

    public BarcodePosition getBarcodePosition() {
        return barcodePosition;
    }

    public DeliveryArmControl.DeliveryPositions getDeliveryPosition() {
        BarcodePosition position = getBarcodePosition();

        if(position == LEFT) {
            //Level 1
            return LOW;
        } else if(position == MIDDLE) {
            //Level 2
            return MID;
        } else {
            //Level 3
            return HIGH;
        }
    }
}
