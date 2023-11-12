package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {   // gain access to methods in the calling OpMode.
    private Telemetry telemetry;
    CRServo transfer = null;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(CRServo.class, "transfer");
        telemetry.addData(">", "Transfer initialized");
        telemetry.update();
    }

}
