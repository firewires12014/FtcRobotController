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
    public void outBelt() {
        secondaryRoller.setPower(-1);
    }
    public void beltDie() {
        secondaryRoller.setPower(0);
    }
    public void outRoller(){
primaryRoller.setPower(1);
    }
    public void inSlow() {
        primaryRoller.setPower(-0.5);
        secondaryRoller.setPower(1);
    }

    public void out(){
        primaryRoller.setPower(1);
        secondaryRoller.setPower(-1);
    }

    public void outSlow() {
        primaryRoller.setPower(0.5);
        secondaryRoller.setPower(-1);
    }
        public void killRoller() {
        primaryRoller.setPower(0);
        }
    // auto: deposits purple pixel
    public void dropOff(){
        primaryRoller.setPower(.5);
    }
    public void stack() { // save for later
        intakeHeight.setPosition(0.46);
    }
    public void stackHeight() {intakeHeight.setPosition(0.85);}
    public void stackHeightTwo() {intakeHeight.setPosition(0.87);}
    public void stackHeightThree() {intakeHeight.setPosition(0);} //grabbing four pixels
    public void score() { //this is for scoring on the ground during auto
        intakeHeight.setPosition(0.97);
    }
    public void grabThree () {intakeHeight.setPosition(0.91);} // for far cycle second grab or grabbing three  pixels
    public void grabOne() {intakeHeight.setPosition(0.85);}
    public void grabTwo(){intakeHeight.setPosition(0.8756);}
    public void grabTwoSpecial(){intakeHeight.setPosition(0.8797);}
 public void grabFour(){intakeHeight.setPosition(0.93578);}
    public void allUp(){
        intakeHeight.setPosition(0);
    }
    public void allDown(){ intakeHeight.setPosition(1);}
    public void grabOneSpecial(){intakeHeight.setPosition(0.85986);}

    public void auto() {intakeHeight.setPosition(0.5);}
    public void up () {intakeHeight.setPosition(0.65);}
    public void closeAuto () {intakeHeight.setPosition(0.6);}
    public void die(){
        primaryRoller.setPower(0);
        secondaryRoller.setPower(0);
    }
 public void beltIn() {
        secondaryRoller.setPower(1);
 }
}
