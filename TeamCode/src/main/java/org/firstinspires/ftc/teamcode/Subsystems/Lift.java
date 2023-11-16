package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    //    private Telemetry telemetry;
    DcMotor leftLift = null;
    DcMotor rightLift = null;

    public Lift(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
//        telemetry.addData(">", "lift up");
//        telemetry.update();
    }

    public void lift(float power) {
        leftLift.setPower(power);

    }
}
