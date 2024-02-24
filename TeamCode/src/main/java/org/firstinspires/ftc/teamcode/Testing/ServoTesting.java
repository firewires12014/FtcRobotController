package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.dropper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="ServoTesting", group="FireWires")
public class ServoTesting extends LinearOpMode {

  private Outtake outtake;
  private Lift lift;
    private Plane plane;
    @Override


    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            //dropper = new dropper(hardwareMap);
            outtake=new Outtake(hardwareMap);
            lift=new Lift(hardwareMap, telemetry);
            plane=new Plane(hardwareMap);
//            outtake.lockPixels();
            if (gamepad1.a) {
              //I love You
                lift.liftToHeight(151);
                lift.holdLift();

            }
           else {
//                lift.liftToHeight(12);

                }
            }
            }
        }

