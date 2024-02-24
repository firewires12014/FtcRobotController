package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor primaryRoller;
    private Servo intakeHeight;
    private CRServo secondaryRoller;

    public Intake(HardwareMap hardwareMap){
        primaryRoller = hardwareMap.get(DcMotor.class, "primaryRoller");
        secondaryRoller = hardwareMap.get(CRServo.class, "secondaryRoller");
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
        secondaryRoller.setPower(1);

    }

    public void out(){

        primaryRoller.setPower(1);
        secondaryRoller.setPower(-1);
    }


    // auto: deposits purple pixel
    public void dropOff(){
        primaryRoller.setPower(.5);
    }
    public void stack() { // save for later
        intakeHeight.setPosition(0.46);
    }
    public void specialStack () {intakeHeight.setPosition(0.43);}
    public void score() { //this is for scoring on the ground during auto
        intakeHeight.setPosition(0.95);
    }
    public void auto() {intakeHeight.setPosition(1);}
    public void up () {intakeHeight.setPosition(0.3);}
    public void closeAuto () {intakeHeight.setPosition(0.6);}
    public void die(){
        primaryRoller.setPower(0);
        secondaryRoller.setPower(0);
    }

}
