package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;

    public Lift(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveLift(double left_stick_y) {
        rightLift.setPower(left_stick_y);
        leftLift.setPower(-left_stick_y);
        telemetry.addData("encoder", rightLift.getCurrentPosition());
        telemetry.update();
    }

    public void liftToHeight(int height) {
        double leftPower;
        double rightPower;
        if ( height < rightLift.getCurrentPosition()){
          leftPower = -0.8;
          rightPower = 0.8;
        }
        else {
            leftPower = 0.7;
            rightPower = -0.7;
        }
        telemetry.addData("encoder", rightLift.getCurrentPosition());
        telemetry.update();
//        leftLift.setTargetPosition(height);
//        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (rightLift.getCurrentPosition() < height) {
            telemetry.addData("Loop", "");
            telemetry.addData("encoder", rightLift.getCurrentPosition());
            telemetry.update();
            rightLift.setPower(rightPower);
            leftLift.setPower(leftPower);
        }
        rightLift.setPower(0);
        leftLift.setPower(0);
        }
        public void holdLift() {
        leftLift.setPower(0.07);
        rightLift.setPower(-0.07);
        }
        public void lowerLift() {
        leftLift.setPower(-0.2);
        rightLift.setPower(0.2);
        }
    }


