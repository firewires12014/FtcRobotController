package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

public class DriveTrain {
    RobotHardware robot       = new RobotHardware(this);
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public double          headingError  = 0;

    public double  targetHeading = 0;
    public double  driveSpeed    = 0;
    public double  turnSpeed     = 0;
    public double  leftSpeed     = 0;
    public double  rightSpeed    = 0;
    public int     leftTarget    = 0;
    public int     rightTarget   = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.2;
    static final double     HEADING_THRESHOLD       = 1.0 ;
    static final double     P_TURN_GAIN            = 0.02;
    static final double     P_DRIVE_GAIN           = 0.03;




    public void teleopDrive(float left_stick_y, float left_stick_x, float right_stick_x) {
        double max;

        double axial   = -left_stick_y;
        double lateral =  left_stick_x;
        double yaw     =  right_stick_x;


        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;


        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightBack.setPower(rightBackPower);

    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        distance = distance * 0.624;

        if (myOpMode.opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            rightTarget = robot.rightFront.getCurrentPosition() + moveCounts;

            robot.leftFront.setTargetPosition(leftTarget);
            robot.rightFront.setTargetPosition(rightTarget);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (myOpMode.opModeIsActive() &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);


            }

            moveRobot(0, 0);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot. rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void strafeLeft (float speed, float time){
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && (runTime.seconds() < time)) {
            robot.leftFront.setPower(-speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(-speed);
        }
        die();
    }
    public void die (){
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
    public void strafeRight (float speed, float time){
        ElapsedTime  runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && (runTime.seconds() < time)) {
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
        }
        die();
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

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

        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);


        }

        moveRobot(0, 0);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }


        robot.leftFront.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightBack.setPower(rightSpeed);
    }


    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    HardwareMap hardwareMap;
}