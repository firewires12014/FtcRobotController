package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb {
//    private Telemetry telemetry;
    public DcMotor climb = null;

    public Climb(HardwareMap hardwareMap) {
        climb = hardwareMap.get(DcMotor.class, "climb");
    }
    public void moveClimb(double right_stick_y){
        climb.setPower(right_stick_y);
    }

}
