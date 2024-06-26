package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//red
@Disabled
public class VisionRedFar extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        //middle
        RIGHT,
        //left
        MIDDLE
        //left
    }




    private Location location = Location.MIDDLE;
    static final Rect MIDDLE_ROI = new Rect(
            new Point(150, 120),
            new Point(240, 175));
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 130),
            new Point(85, 200));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public VisionRedFar(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 0, 0); //HSV Hue Saturation Value colors looking for colors 0-250
        Scalar highHSV = new Scalar(30, 255, 255); //these are the values for green
        
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(LEFT_ROI);
        Mat left = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Right raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 90) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneRight) {
            location = Location.LEFT;
            telemetry.addData("Cube Location", "left");
        }
        else if (stoneLeft) {
            location = Location.MIDDLE;
            telemetry.addData("Cube Location", "middle");
        }
        else {
            location = Location.RIGHT;
            telemetry.addData("Cube Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0,0);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.LEFT? green:red);//middle
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.RIGHT? green:red);//right

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}