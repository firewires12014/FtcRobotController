package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Transfer Test", group="FireWires")
@Disabled
public class TransferTesting extends LinearOpMode {

    private Servo transfer;
    private Servo intakeheight;

    @Override
    public void runOpMode() throws InterruptedException {

        transfer = hardwareMap.get(Servo.class, "transfer");
        intakeheight = hardwareMap.get(Servo.class, "intakeHeight");

        waitForStart();
        while (opModeIsActive()) {
            //transfer.setPosition(gamepad1.right_stick_y);//basket

            double up = 0.9;
            double down = 0.15;

            if(gamepad1.y) {
                //transfer.setPosition(up);
                intakeheight.setPosition(0);
            }
            if(gamepad1.a) {
                //transfer.setPosition(down);
                intakeheight.setPosition(1);
            }

            telemetry.addData("Position", transfer.getPosition());
            telemetry.update();
        }

    }
}