package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor primaryRoller;
    private Servo intakeHeight;

    public Intake(HardwareMap hardwareMap){
        primaryRoller = hardwareMap.get(DcMotor.class, "primaryRoller");
        intakeHeight = hardwareMap.get(Servo.class, "intakeHeight");
    }
    public void initIntake(){
        intakeHeight.setPosition(0.025);
    }
    public void stackIntake() {
        intakeHeight.setPosition(1);}
    public void resetIntake(){
        intakeHeight.setPosition(0.4);
    }
    public void in(){
        primaryRoller.setPower(-1);

    }

    public void out(){
        primaryRoller.setPower(1);
    }

    // auto: deposits purple pixel
    public void dropOff(){
        primaryRoller.setPower(.5);
    }
    public void stack() { // save for later
        intakeHeight.setPosition(0.425);
    }
    public void specialStack () {intakeHeight.setPosition(0.567);}
    public void score() { //this is for scoring on the ground during auto
        intakeHeight.setPosition(0.95);
    }

    public void die(){
        primaryRoller.setPower(0);
    }
}
