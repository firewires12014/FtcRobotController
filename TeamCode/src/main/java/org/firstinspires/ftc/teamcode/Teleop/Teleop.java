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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;
import org.firstinspires.ftc.teamcode.Testing.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Testing.TransferTesting;

@TeleOp(name = "Teleop", group = "Robot")
public class Teleop extends LinearOpMode {
    private DriveTrain driveTrain;
    private Lift lift;
    private Intake intake;
    private Outtake outtake;
    private Plane plane;

    private void doAutoDisplay() {
    }

    private void handleGamepad() {
    }


    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }


    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        plane = new Plane(hardwareMap);
        ElapsedTime lockTimer = new ElapsedTime();


        waitForStart();

        while (opModeIsActive()) {
            intake.initIntake();

            lift.moveLift(gamepad2.left_stick_y);

            if (gamepad1.right_bumper) {
                intake.in();
            } else if (gamepad1.left_bumper) {
                intake.out();
            } else {
                intake.die();
            }
            if (gamepad2.touchpad) plane.launch();

            if (gamepad2.left_stick_button) plane.reset();

            if (gamepad2.left_bumper) {
                outtake.pivotEnding();
                outtake.resetBucket();
            }//score
            if (gamepad2.right_bumper) {
                outtake.pivotStart();
            }//score

            driveTrain.teleopDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad2.dpad_up) {
                outtake.transferPixels();
                sleep(1000);
                outtake.lockPixels();
            }
            if (gamepad2.dpad_down) {
                outtake.resetBucket();
            }
//            int count = 0;
//            if (gamepad2.a && lockTimer.seconds() > 4) {
//                count++;
//                lockTimer.reset();
            }
            if (gamepad2.left_trigger == (1)) {
                outtake.releaseTop();
                }
            if (gamepad2.right_trigger == (1)){
                outtake.releaseBottom();
            }
            if (gamepad2.b) {
                outtake.lockPixels();
            }

            displayKind = SampleRevBlinkinLedDriver.DisplayKind.AUTO;

            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            blinkinLedDriver.setPattern(pattern);

            display = telemetry.addData("Display Kind: ", displayKind.toString());
            patternName = telemetry.addData("Pattern: ", pattern.toString());

            ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            handleGamepad();

            if (displayKind == SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
                doAutoDisplay();
            } else {


                telemetry.addData("Heading: ", driveTrain.getHeading());
                telemetry.update();
            }
        }
    }


