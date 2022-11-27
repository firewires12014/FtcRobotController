package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
    public class tello2 extends LinearOpMode {


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
            if (gamepad1.left_stick_y != 0) {
               rob.backLeft.setPower(-gamepad1.left_stick_y);
                rob.frontLeft.setPower(-gamepad1.left_stick_y);
            } else {
                rob.backLeft.setPower(0);
                rob.frontLeft.setPower(0);
            }
            if (gamepad1.right_stick_y != 0) {
                rob.backRight.setPower(gamepad1.right_stick_y);
                rob.frontRight.setPower(gamepad1.right_stick_y);
            } else {
                rob.backRight.setPower(0);
                rob.frontRight.setPower(0);
            }

            if (gamepad1.right_trigger != 0) {
                rob.backRight.setPower(-gamepad1.right_trigger);
               rob. frontRight.setPower(gamepad1.right_trigger);
                rob.backLeft.setPower(-gamepad1.right_trigger);
                rob.frontLeft.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger != 0) {
                rob.backRight.setPower(gamepad1.left_trigger);
                rob.frontRight.setPower(-gamepad1.left_trigger);
                rob.backLeft.setPower(gamepad1.left_trigger);
                rob.frontLeft.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.dpad_left) {
            rob.turret.setPower(1);
            } else  if (gamepad1.dpad_right) {
            rob.turret.setPower(-1);
            } else {
                rob.turret.setPower(0);
            }
        }
    }
}
