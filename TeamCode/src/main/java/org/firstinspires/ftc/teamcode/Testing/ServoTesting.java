package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="ServoTesting", group="FireWires")
public class ServoTesting extends LinearOpMode {

    private Outtake outtake;

    @Override


    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            outtake = new Outtake(hardwareMap);

            waitForStart();
            if (gamepad1.a) {
                outtake.releaseMain(); //I love You
                outtake.releaseSecondary();
            }
            if (gamepad1.b) {
                outtake.lockPixels();
            }
            }
            }
        }

