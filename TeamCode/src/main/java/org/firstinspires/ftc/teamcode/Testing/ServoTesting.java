package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.dropper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="ServoTesting", group="FireWires")
public class ServoTesting extends LinearOpMode {

    private dropper dropper;
    private Intake intake;
    @Override


    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            //dropper = new dropper(hardwareMap);
            intake = new Intake(hardwareMap);
            if (gamepad1.a) {
                intake.stack(); //I love You

            }
           else {
//               dropper.Hold();
               intake.score();

                }
            }
            }
        }

