package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    private Servo pivot, transfer,fix, primaryLock, secondaryLock;

    static final double DEFAULT_POS = 0.15;
    static final double TRANSFER_POS = 0.95;
    static final double LOCK_TOP = 1.0;
    static final double LOCK_BOTTOM = 1.00;
    static final double UNLOCK_MAIN = 0.0;
    static final double UNLOCK_SECONDARY = 0.0;
    static final double PIVOT_START = 0.98;
    static final double PIVOT_ENDING = 0;
    static final double PIVOT_INTERIM = 0.6;
    static final double FIX_IN = 1;
    static final double FIX_OUT = 0;

    public Outtake(HardwareMap hardwareMap){
        pivot = hardwareMap.get(Servo.class, "outtakePivot");
        transfer = hardwareMap.get(Servo.class, "transfer");
        fix = hardwareMap.get(Servo.class, "fix");
        primaryLock = hardwareMap.get(Servo.class, "primaryLock");
        secondaryLock = hardwareMap.get(Servo.class, "secondaryLock");
    }

    public void transferPixels(){
        transfer.setPosition(TRANSFER_POS);
    }
    public void fixIn(){
        fix.setPosition(FIX_IN);
    }
    public void fixOut(){
        fix.setPosition(FIX_OUT);
    }
    public void resetBucket(){
        transfer.setPosition(DEFAULT_POS);
    }
    public void lockPixels(){
        primaryLock.setPosition(LOCK_BOTTOM);
        secondaryLock.setPosition(LOCK_TOP);
    }
    public void autoPixel(){
        transfer.setPosition(1);
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
    public void pivotStart(){
        pivot.setPosition(PIVOT_START);
    }

    public void pivotEnding(){
        pivot.setPosition(PIVOT_ENDING);
    }

    public void pivotInterim(){
        pivot.setPosition(PIVOT_INTERIM);
    }



}
