package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Testing Motors", group = "Robot")
@Disabled
    public class TestingMotors extends LinearOpMode {

    public CRServo secondaryIntakeRoller;
    public DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        secondaryIntakeRoller = hardwareMap.get(CRServo.class, "secondaryIntakeRoller");
        intake = hardwareMap.get(DcMotor.class, "intake");
        waitForStart();

        while (opModeIsActive())
        {
            secondaryIntakeRoller.setPower(-gamepad1.left_stick_y);
            intake.setPower(gamepad1.left_stick_y);
        }
    }
}
