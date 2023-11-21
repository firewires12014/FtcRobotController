package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor primaryRoller;
    private CRServo secondaryRoller;
    private Servo intakeheight;

    public Intake(HardwareMap hardwareMap){
        primaryRoller = hardwareMap.get(DcMotor.class, "primaryRoller");
        secondaryRoller = hardwareMap.get(CRServo.class, "secondaryRoller");
        intakeheight = hardwareMap.get(Servo.class, "intakeHeight");
    }
    public void setHeight(){
        intakeheight.setPosition(0.4);
    }
    public void in(){git
        secondaryRoller.setPower(-1);
        primaryRoller.setPower(1);

    }

    public void out(){
        secondaryRoller.setPower(1);
        primaryRoller.setPower(-1);
    }

    public void die(){
        secondaryRoller.setPower(0);
        primaryRoller.setPower(0);
    }
}
