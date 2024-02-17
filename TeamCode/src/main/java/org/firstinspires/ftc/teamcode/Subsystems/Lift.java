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
    }

    public void moveLift(double left_stick_y) {
        rightLift.setPower(left_stick_y);
        leftLift.setPower(-left_stick_y);
    }

    public void liftToHeight(int height) {
        telemetry.addData("encoder", rightLift.getCurrentPosition());
        telemetry.update();
        rightLift.setTargetPosition(height);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftLift.getCurrentPosition() > height) {
            telemetry.addData("Loop", "");
            telemetry.addData("encoder", leftLift.getCurrentPosition());
            telemetry.update();
            rightLift.setPower(0.25);
            leftLift.setPower(-0.25);
        }
        rightLift.setPower(0);
        leftLift.setPower(0);
        }
    }


