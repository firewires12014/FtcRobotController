package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Plane {
    Servo plane = null;

    public Plane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(Servo.class, "launcher");
    }
public void launch() {
        plane.setPosition(1);
}
public void reset() {
        plane.setPosition(0);
}
}
