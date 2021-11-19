package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.HSV;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INPUT;
import static org.firstinspires.ftc.teamcode.vision.ShippingElementDetector.Mats.INRANGE;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;
import java.util.List;

public class DuckDetector extends OpenCvPipeline {

    public static int hue1 = 61;
    public static int saturation1 = 100;
    public static int value1 = 0;
    public static int hue2 = 70;
    public static int saturation2 = 255;
    public static int value2 = 255;
    public Scalar lowerB = new Scalar(hue1, saturation1, value1);
    public Scalar upperB = new Scalar(hue2, saturation2, value2);

    private Mat normalImage;
    private Mat HSVMat;
    private Mat inRangeMat;

    private List<MatOfPoint> contours;
    private MatOfPoint biggestContour;
    private Rect boundingRect;

    double centerX;
    double centerY;

    private ShippingElementDetector.Mats activeMat = INPUT;

    public DuckDetector() {
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
            Imgproc.rectangle(getActiveMat(), boundingRect, new Scalar(255, 0, 0), 3);
        } else {
            boundingRect = null;
        }



        return input;
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
