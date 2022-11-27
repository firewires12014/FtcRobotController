package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="The Auto", group="Robot")
    public class autotello extends LinearOpMode {
    @Override
    public void runOpMode() {
        rob rob = new rob(hardwareMap, telemetry);
        rob.init();

        telemetry.addData("Status", "Gay");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            rob.showEncoders();
            rob.straight(0.5);
            rob.showEncoders();
            sleep(2500);
            rob.die();
            rob.gay(0.25);
            rob.showEncoders();
            sleep(1000);
            rob.die();
            stop();
        }


    }

}