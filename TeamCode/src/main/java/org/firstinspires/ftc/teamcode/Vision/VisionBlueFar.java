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
//blue
@Disabled
public class VisionBlueFar extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        RIGHT,
        //middle
        MIDDLE,
        //left
        NOT_FOUND
        //left
    }




    private Location location = Location.MIDDLE;
    static final Rect RIGHT_ROI = new Rect(
            new Point(250, 80),
            new Point(320, 200));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(80, 90),
            new Point(200, 170));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public VisionBlueFar(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //COLOR BGR240,51,100

//        Scalar lowHSV = new Scalar(100,100,100);
//        Scalar highHSV = new Scalar(180,255,255);
        Scalar lowHSV = new Scalar(90,50,70);
        Scalar highHSV = new Scalar(128,255,255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(RIGHT_ROI);
        Mat right = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / RIGHT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Right raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 100) + "%");

        boolean stoneRight = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMiddle = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Cube Location", "LEFT");
        }
        else if (stoneMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Cube Location", "MIDDLE");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cube Location", "RIGHT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? green:red);//middle
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? green:red);//right

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}