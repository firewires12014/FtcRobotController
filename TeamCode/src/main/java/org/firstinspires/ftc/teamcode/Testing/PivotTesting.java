package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Pivot Test", group="FireWires")
public class PivotTesting extends LinearOpMode {

    private Servo pivot;
    private Servo pivotImpl;

    @Override
    public void runOpMode() throws InterruptedException {

        pivot = hardwareMap.get(Servo.class, "outtakePivot");
        pivotImpl = hardwareMap.get(ServoImplEx.class, "outakePivot");

        waitForStart();

        double inner = 1;
         double outer = 0.00398;
        pivot.setPosition(outer);//basket

        sleep(3000);
        pivot.setPosition(inner);//score
        sleep(3000);
        pivot.setPosition(.6); //interim
        sleep(1000);

        pivot.setPosition(outer);//basket

        sleep(3000);
        pivot.setPosition(inner);//score
        sleep(3000);
        pivot.setPosition(.6); //interim
        sleep(1000);


        sleep(3000);

    }
}