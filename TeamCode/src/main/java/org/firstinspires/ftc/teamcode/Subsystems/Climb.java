package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private Telemetry telemetry;
    DcMotor climb = null;

    public Climb(LinearOpMode opmode, Telemetry telemetry) {
        myOpMode = opmode;
    }

    public void init() {
        climb = myOpMode.hardwareMap.get(DcMotor.class, "climb");
        telemetry.addData(">", "Climbc initialized");
        telemetry.update();
    }

}
