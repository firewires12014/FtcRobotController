/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp(name = "Teleop", group = "Robot")
public class Teleop extends LinearOpMode {
    //    DriveTrain driveTrain = new DriveTrain(hardwareMap);
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    public CRServo secondaryIntakeRoller;
    public DcMotor intake;
    public Servo intakeHeight;
    public Servo outtakePivot;

    @Override
    public void runOpMode() {
        Lift lift = new Lift(hardwareMap);

        secondaryIntakeRoller = hardwareMap.get(CRServo.class, "secondaryIntakeRoller");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHeight = hardwareMap.get(Servo.class, "intakeHeight");
        outtakePivot = hardwareMap.get(Servo.class, "outtakePivot");

        intakeHeight.setPosition(.4);
        //outtakePivot.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {
//            driveTrain.teleopDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            lift.moveLift(gamepad2.left_stick_y);

            if (gamepad1.right_bumper) {
                secondaryIntakeRoller.setPower(-1);
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                secondaryIntakeRoller.setPower(1);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
                secondaryIntakeRoller.setPower(0);
            }
            outtakePivot.setPosition(gamepad1.left_stick_y);
        }
    }
}

