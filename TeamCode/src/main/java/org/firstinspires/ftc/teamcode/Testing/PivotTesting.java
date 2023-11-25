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
        pivotImpl = hardwareMap.get(ServoImplEx.class, "outtakePivot");

        waitForStart();

        double inner = 0;
         double outer = 1;
        pivot.setPosition(outer);//basket

        sleep(2500);
        pivot.setPosition(inner);//score
        sleep(3000);
        //pivot.setPosition(.5); //interim
        sleep(1000);

        pivot.setPosition(outer);//basket



        sleep(2500);
        pivot.setPosition(inner);//score
        sleep(3000);
       // pivot.setPosition(.5); //interim
        sleep(1000);


        sleep(3000);

    }
}