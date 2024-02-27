package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Autonomous(name = "testPath", group = "Testing")

public class testPath extends LinearOpMode {
    Lift lift;

    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap, telemetry);

        waitForStart();
        lift.liftToHeight(500);
        sleep(30000);

        if(isStopRequested()) return;

    }
}