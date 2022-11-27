package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
    public class tello extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");


        telemetry.addData("Status", "Gay");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            if (gamepad1.left_stick_y != 0) {
                backLeft.setPower(-gamepad1.left_stick_y);
                frontLeft.setPower(-gamepad1.left_stick_y);
            } else {
                backLeft.setPower(0);
                frontLeft.setPower(0);
            }
            if (gamepad1.right_stick_y != 0) {
                backRight.setPower(gamepad1.right_stick_y);
                frontRight.setPower(gamepad1.right_stick_y);
            } else {
                backRight.setPower(0);
                frontRight.setPower(0);
            }

            if (gamepad1.right_trigger != 0) {
                backRight.setPower(-gamepad1.right_trigger);
                frontRight.setPower(gamepad1.right_trigger);
                backLeft.setPower(-gamepad1.right_trigger);
                frontLeft.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger != 0) {
                backRight.setPower(gamepad1.left_trigger);
                frontRight.setPower(-gamepad1.left_trigger);
                backLeft.setPower(gamepad1.left_trigger);
                frontLeft.setPower(-gamepad1.left_trigger);
            }
        }
    }
}
