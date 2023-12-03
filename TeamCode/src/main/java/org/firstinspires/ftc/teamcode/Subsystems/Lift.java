package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    //    private Telemetry telemetry;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;

    public Lift(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
//        telemetry.addData(">", "lift up");
//        telemetry.update();
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveLift(float left_stick_y) {
        rightLift.setPower(left_stick_y);
        leftLift.setPower(-left_stick_y);
    }
}
