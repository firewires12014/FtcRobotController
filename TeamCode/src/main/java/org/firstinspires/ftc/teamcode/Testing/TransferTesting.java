package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Transfer Test", group="FireWires")
public class TransferTesting extends LinearOpMode {

    private Servo transfer;

    @Override
    public void runOpMode() throws InterruptedException {

        transfer = hardwareMap.get(Servo.class, "transfer");

        waitForStart();
        while (opModeIsActive()) {
            //transfer.setPosition(gamepad1.right_stick_y);//basket

            double up = 0.87;
            double down = 0.15;

            if(gamepad1.y) {
                transfer.setPosition(up);
            }
            if(gamepad1.a) {
                transfer.setPosition(down);
            }

            telemetry.addData("Position", transfer.getPosition());
            telemetry.update();
        }

    }
}