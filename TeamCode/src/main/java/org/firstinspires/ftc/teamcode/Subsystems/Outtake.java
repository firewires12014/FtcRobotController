package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    private Servo pivot, transfer, primaryLock, secondaryLock;

    static final double DEFAULT_POS = 0.15;
    static final double TRANSFER_POS = 0.9;
    static final double LOCK_TOP = 0.0;
    static final double LOCK_BOTTOM = 0.0;
    static final double UNLOCK_TOP = 1.0;
    static final double UNLOCK_BOTTOM = 1.0;
    static final double PIVOT_START = 0.0;
    static final double PIVOT_ENDING = 0.945;
    static final double PIVOT_INTERIM = 0.6;

    public Outtake(HardwareMap hardwareMap){
        pivot = hardwareMap.get(Servo.class, "outtakePivot");
        transfer = hardwareMap.get(Servo.class, "transfer");
        primaryLock = hardwareMap.get(Servo.class, "primaryLock");
        secondaryLock = hardwareMap.get(Servo.class, "secondaryLock");

    }

    public void transferPixels(){
        transfer.setPosition(TRANSFER_POS);

    }

    public void resetBucket(){
        transfer.setPosition(DEFAULT_POS);
    }
    public void lockPixels(){
        primaryLock.setPosition(LOCK_TOP);
        secondaryLock.setPosition(LOCK_BOTTOM);
    }

    public void releaseTop(){
        secondaryLock.setPosition(UNLOCK_BOTTOM);
    }
    public void releaseBottom() {
        primaryLock.setPosition(UNLOCK_TOP);
    }

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
