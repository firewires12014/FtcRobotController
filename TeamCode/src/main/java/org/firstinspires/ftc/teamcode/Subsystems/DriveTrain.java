package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain {
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    public double headingError = 0;
    public double targetHeading = 0;
    public double driveSpeed = 0;
    public double turnSpeed = 0;
    public double leftSpeed = 0;
    public double rightSpeed = 0;
    public int leftTarget = 0;
    public int rightTarget = 0;
    private YawPitchRollAngles lastHeading;
    private double currentHeading = 0;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.01;
    static final double P_DRIVE_GAIN = 0.03;

    public IMU imu = null;      // Control/Expansion Hub IMU

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        lastHeading = imu.getRobotYawPitchRollAngles();
        myOpMode = this.myOpMode;

    }

    public void teleopDrive(float left_stick_y, float left_stick_x, float right_stick_x) {
        double max;

        double axial = -left_stick_y;
        double lateral = left_stick_x;
        double yaw = right_stick_x;


        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;


        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }


        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        distance = distance * 0.624;

            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = leftFront.getCurrentPosition() + moveCounts;
            rightTarget = rightFront.getCurrentPosition() + moveCounts;

            leftFront.setTargetPosition(leftTarget);
            rightFront.setTargetPosition(rightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (leftFront.isBusy() && rightFront.isBusy()) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0) turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);


            }

            moveRobot(0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void TeleOpStrafeL(double speed) {
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }
    public void TeleOpStrafeR(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
    }
    public void TeleOpDriveF(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }
    public void TeleOpDriveB(double speed) {
        leftFront.setPower(-speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(-speed);
    }
    public void strafeLeft(float speed, float time) {
        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time) {
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(-speed);
        }
        die();
    }

    public void die() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void strafeRight(float speed, float time) {
        ElapsedTime runTime = new ElapsedTime();
        while  (runTime.seconds() < time) {
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
        }
        die();
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while  (Math.abs(headingError) > 50) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.

        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while  (holdTimer.time() < holdTime) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);


        }

        moveRobot(0, 0);
    }

    //reset angle
    public void resetHeading(){
        imu.resetYaw();
        currentHeading = 0;
    }

    //get heading
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES)-lastHeading.getYaw(AngleUnit.DEGREES);
       // double angle = orientation.getYaw(AngleUnit.DEGREES);
        if (deltaAngle > 180) deltaAngle -= 360;
        else if (deltaAngle <= -180) deltaAngle += 360;
        currentHeading += deltaAngle;
        lastHeading = orientation;
        return currentHeading;
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }


        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);
    }

    public float joystick_conditioning(float x, float db, double off, double gain) {
        float output = 0;
        boolean sign = (x > 0);

        x = Math.abs(x);
        if (x > db) {
            output = (float) (off - ((off - 1) * Math.pow(((db - x) / (db - 1)), gain)));
            output *= sign ? 1 : -1;
        }
        return output;
    }
    public void setPower(double speed){
        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
    }

    public void setTurnPower(double speed){
        leftBack.setPower(-speed);
        leftFront.setPower(-speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
    }

    public void setStrafePower(double speed){
        leftBack.setPower(-speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(-speed);
    }

    public List<Integer> getWheelPosition() {
        List<Integer> positions = new ArrayList<>();
        positions.add(leftBack.getCurrentPosition());
        positions.add(leftFront.getCurrentPosition());
        positions.add(rightBack.getCurrentPosition());
        positions.add(rightFront.getCurrentPosition());
        return positions;
    }

    public void resetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
//    public void encoderDrive(double counts, double speed) {
//
//        //LinearOpMode linearOpMode = (LinearOpMode) myOpMode;
//        leftBack.setPower(speed);
//        leftFront.setPower(speed);
//        rightBack.setPower(speed);
//        rightFront.setPower(speed);
//
//        //while (linearOpMode.opModeIsActive()) {
//            for (int position : getWheelPosition()) {
//                if (Math.abs(position) > counts) {
//                    die();
//                    return;
//                }
//            }
//       // }
//        die();
//    }
//
//    public void turn(double angle, double speed) {
//        //LinearOpMode linearOpMode = (LinearOpMode) myOpMode;
//        leftBack.setPower(-speed);
//        leftFront.setPower(-speed);
//        rightBack.setPower(speed);
//        rightFront.setPower(speed);
//
//        //while (linearOpMode.opModeIsActive()) {
//                if (getHeading() > angle) {
//                    die();
//                    return;
//                }
//            //}
//        die();
//        double threshhold = 1;
//        double error = getHeading()-angle;
//                if (error <= threshhold){
//
//                }
//                else {
//                    if (error > 0) {
//                        leftBack.setPower(-.3);
//                        leftFront.setPower(-.3);
//                        rightBack.setPower(.3);
//                        rightFront.setPower(.3);
//                    }
//                    else {
//                        leftBack.setPower(.3);
//                        leftFront.setPower(.3);
//                        rightBack.setPower(-.3);
//                        rightFront.setPower(-.3);
//                    }
//                    //while (linearOpMode.opModeIsActive()) {
//                        if (getHeading()-angle < threshhold ) {
//                            die();
//                            return;
//                        }
//                    //}
//                    die();
//                }
//        }
}
