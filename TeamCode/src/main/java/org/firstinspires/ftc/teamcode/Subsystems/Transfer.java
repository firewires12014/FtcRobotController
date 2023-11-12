package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private Telemetry telemetry;
    CRServo transfer = null;

    public Transfer(LinearOpMode opmode, Telemetry telemetry) {
        myOpMode = opmode;
    }

    public void init() {
        transfer = myOpMode.hardwareMap.get(CRServo.class, "transfer");
        telemetry.addData(">", "Transfer initialized");
        telemetry.update();
    }

}
