package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.dropper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="ServoTesting", group="FireWires")
//@Disabled
public class ServoTesting extends LinearOpMode {
    private DriveTrain drive;
  private Intake intake;

  private Lift lift;
    private Plane plane;
    @Override


    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            //dropper = new dropper(hardwareMap);
            intake=new Intake(hardwareMap);
            lift=new Lift(hardwareMap, telemetry);
            plane=new Plane(hardwareMap);
            drive=new DriveTrain(hardwareMap);
//            outtake.lockPixels();
            if (gamepad1.a) {
intake.grabOne();            }
            if (gamepad1.b) {
intake.grabTwo();            }
            if (gamepad1.x) {
                       intake.superStack(); }
            if (gamepad1.y){
intake.grabFour();            }
            if (gamepad1.dpad_down) {
                intake.score();
            }
            if (gamepad1.dpad_up) {
                intake.up();
            }

            if (gamepad1.right_trigger > 0) {
                intake.in();
            } else if (gamepad1.left_trigger > 0) {
                intake.out();
            } else {
                intake.die();
            }
            }
            }
        }

