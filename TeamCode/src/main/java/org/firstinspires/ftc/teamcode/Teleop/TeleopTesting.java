package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Climb;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;
import org.firstinspires.ftc.teamcode.Testing.SampleRevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleopTesting", group = "Robot")
public class TeleopTesting extends OpMode {
    private DriveTrain driveTrain;

    private void doAutoDisplay() {
    }

    private void handleGamepad() {
    }

    double Kg = 0.1;
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;


    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern glow;
    RevBlinkinLedDriver.BlinkinPattern blink;
    Telemetry.Item patternName;
    Telemetry.Item display;
    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);


    }

    @Override
    public void loop() {
//        if (gamepad1.a){
//            driveTrain.testBackLeft();
//        }
//        if (gamepad1.b){
//            driveTrain.testBackRight();
//        }
//        if (gamepad1.x){
//            driveTrain.testFrontLeft();
//        }
//        if (gamepad1.y){
//            driveTrain.testFrontRight();
//        }
    }
}

