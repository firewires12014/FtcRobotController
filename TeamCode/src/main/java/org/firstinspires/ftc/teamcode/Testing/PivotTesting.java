package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Pivot Test", group="FireWires")
public class PivotTesting extends LinearOpMode {

    private Servo pivot;

    @Override
    public void runOpMode() throws InterruptedException {

        pivot = hardwareMap.get(Servo.class, "outtakePivot");

        waitForStart();

        pivot.setPosition(0);//basket

        sleep(3000);
        pivot.setPosition(.995);//score
        sleep(3000);
        pivot.setPosition(.6); //interim
        sleep(1000);

        ;
//        sleep(3000);
//
//        pivot.setPosition(0.57);
        pivot.setPosition(0);

        sleep(3000);

    }
}