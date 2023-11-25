package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class redPipeLine extends OpenCvPipeline {
    Mat mat = new Mat();
    public enum Location {
        Left,
        Mid,
        Right
    }
    private Location location;
    static  final Rect Left_ROI = new Rect(
            new Point(170, 110),
            new Point(215,140));
    static final Rect Right_ROI = new Rect(
            new Point(260,110),
            new Point(310,150));
    static double PERCENT_COLOR_THRESHOLD = 0.3; //number of pixels in the box to sense something maybe change to be higher for better
@Override
    public Mat processFrame(Mat input) {
    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
    Scalar lowHSV = new Scalar(0, 0, 0); //HSV Hue Saturation Value colors looking for colors 0-250
    Scalar highHSV = new Scalar(30, 255, 255); //these are the values for green

    Core.inRange(mat, lowHSV, highHSV, mat);

    Mat left = mat.submat(Right_ROI);
    Mat right = mat.submat(Left_ROI);

    double leftValue = Core.sumElems(left).val[0] / Left_ROI.area() / 255;
    double rightValue = Core.sumElems(right).val[0] / Right_ROI.area() / 255;

    left.release();
    right.release();

    boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;
    boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;

    if (elementLeft) {
        location = Location.Left;
    }
    else if (elementRight) {
        location = Location.Right;
    }
    else {
        location = Location.Mid;
    }
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

    Scalar leftField = new Scalar(255, 0, 0);
    Scalar rightField = new Scalar(0, 255, 0);

    Imgproc.rectangle(mat, Right_ROI, location == Location.Right? rightField:leftField);
    Imgproc.rectangle(mat, Left_ROI, location == Location.Left? leftField:rightField);

    return mat;
             }
    public Location getLocation() {
    return location;
    }
}

