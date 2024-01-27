package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class dropper {
    Servo dropper = null;

    static final double Drop = 0.18;
    static final double Hold = 1;
    public dropper(HardwareMap hardwareMap) {

        dropper = hardwareMap.get(Servo.class, "dropper");

    }

    public void Drop() {
        dropper.setPosition(Drop);

    }
    public void Hold() {
        dropper.setPosition(Hold);

    }
}
