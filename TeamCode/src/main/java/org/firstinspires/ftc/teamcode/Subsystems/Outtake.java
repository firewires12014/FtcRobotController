package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    private Servo pivot, primaryLock, secondaryLock, left, right;

    static final double LOCK_TOP = 1.0;
    static final double LOCK_BOTTOM = 1.00;
    static final double UNLOCK_MAIN = 0.0;
    static final double UNLOCK_SECONDARY = 0.0;
    static final double PIVOT_INTERIM = 0.6;


    public Outtake(HardwareMap hardwareMap){
        pivot = hardwareMap.get(Servo.class, "outtakePivot");
        primaryLock = hardwareMap.get(Servo.class, "primaryLock");
        secondaryLock = hardwareMap.get(Servo.class, "secondaryLock");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
    }


    public void lockPixels(){
        primaryLock.setPosition(LOCK_BOTTOM);
        secondaryLock.setPosition(LOCK_TOP);
    }

    public void pivotAuto(){
        pivot.setPosition(1);
    }

    public void lockSecondary() {
        secondaryLock.setPosition(LOCK_TOP);
    }

    public void releaseMain(){ primaryLock.setPosition(UNLOCK_MAIN);}
    public void releaseSecondary() {
        secondaryLock.setPosition(UNLOCK_SECONDARY);
    }
    public void releasePixels () {secondaryLock.setPosition(UNLOCK_SECONDARY); primaryLock.setPosition(UNLOCK_MAIN);}
    public void pivotInterim(){
        pivot.setPosition(PIVOT_INTERIM);
    }
    public void intakePosition(){
        left.setPosition(0.31); //.296
        right.setPosition(.704); //intake
    }

public void diffyPosition(int position) {
    if (position == 0) {
        right.setPosition(0.042); // Inverted Left
        left.setPosition(0.042);
    } else if (position == 1){
        right.setPosition(.231);
        left.setPosition(.231); // Horizontal Left
    } else if (position == 2){
        right.setPosition(0.405);
        left.setPosition(0.405); // Angled Left
    } else if (position == 3) {
        left.setPosition(.507);// Prime Vertical
        right.setPosition(.507);
    } else if (position == 4){
        right.setPosition(.59); // Angled Right
        left.setPosition(0.59);
    } else if (position ==5){
        right.setPosition(.78);
        left.setPosition(.78); // Horizontal Right
    } else if (position == 6){
        right.setPosition(.95); // Inverted Right
        left.setPosition(.95);
    }

}


}
