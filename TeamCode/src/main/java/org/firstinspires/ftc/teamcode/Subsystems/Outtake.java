package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    private Servo pivot, frontGrabber, backGrabber, leftPivot, rightPivot;

    static final double LOCK_TOP = 0.0;
    static final double LOCK_BOTTOM = 0.00;
    static final double UNLOCK_MAIN = 1.0;
    static final double UNLOCK_SECONDARY = 1.0;
    static final double PIVOT_INTERIM = 0.6;


    public Outtake(HardwareMap hardwareMap){
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        backGrabber = hardwareMap.get(Servo.class, "backGrabber");
        frontGrabber = hardwareMap.get(Servo.class, "frontGrabber");
    }


    public void lockPixels(){
        frontGrabber.setPosition(LOCK_BOTTOM);
        backGrabber.setPosition(LOCK_TOP);
    }
    public void lockPrimary() {
        frontGrabber.setPosition(LOCK_TOP);
    }
    public void pivotAuto(){
        pivot.setPosition(1);
    }

    public void lockSecondary() {
        backGrabber.setPosition(LOCK_BOTTOM);
    }

    public void releaseMain(){ frontGrabber.setPosition(UNLOCK_MAIN);}
    public void releaseSecondary() {
        backGrabber.setPosition(UNLOCK_SECONDARY);
    }
    public void releasePixels () {
        backGrabber.setPosition(UNLOCK_SECONDARY); frontGrabber.setPosition(UNLOCK_MAIN);}
    public void pivotInterim(){
        pivot.setPosition(PIVOT_INTERIM);
    }
    public void intakePosition(){
        leftPivot.setPosition(0.29); //.296
        rightPivot.setPosition(.755); //intake
    }
    public void autoDrop(){
        leftPivot.setPosition(0.367);
        rightPivot.setPosition(0.648);
    }

public void diffyPosition(int position) {
    if (position == 0) {
        rightPivot.setPosition(0.0138); // Inverted Left
        leftPivot.setPosition(0.07);
    } else if (position == 1){
        rightPivot.setPosition(.225);
        leftPivot.setPosition(.282); // Horizontal Left
    } else if (position == 2){
        rightPivot.setPosition(0.4);
        leftPivot.setPosition(0.456); // Angled Left
    } else if (position == 3) {
        leftPivot.setPosition(.535);// Prime Vertical
        rightPivot.setPosition(.478);
    } else if (position == 4){
        rightPivot.setPosition(.591); // Angled Right
        leftPivot.setPosition(0.647);
    } else if (position ==5){
        rightPivot.setPosition(.74);
        leftPivot.setPosition(.83); // Horizontal Right
    } else if (position == 6){
        rightPivot.setPosition(.943); // Inverted Right
        leftPivot.setPosition(1);
    }

}


}
