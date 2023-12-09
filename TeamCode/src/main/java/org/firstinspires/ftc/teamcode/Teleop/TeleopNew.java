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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

@TeleOp(name = "TeleopNew", group = "Robot")
public class TeleopNew extends OpMode {
    private DriveTrain driveTrain;
    private Lift lift;
    private Intake intake;
    private Outtake outtake;
    private Plane plane;
    private Climb climb;
    int transfer = 0;
    double upTime = 1; //seconds
    double lockTime = 1; //seconds
    ElapsedTime upTimer = new ElapsedTime();
    ElapsedTime secureTimer = new ElapsedTime();

    private void doAutoDisplay() {
    }

    private void handleGamepad() {
    }

    double Kg = 0.1;
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

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        plane = new Plane(hardwareMap);
        climb = new Climb(hardwareMap);
        intake.initIntake();

    }

    @Override
    public void loop() {

        //Intake
        if (gamepad1.right_bumper) {
            intake.in();
        } else if (gamepad1.left_bumper) {
            intake.out();
        } else {
            intake.die();
        }
        if (gamepad2.y) {
            intake.stackIntake();
        }
        else {
            intake.initIntake();
        }

        //Transfer FSMs
        switch (transfer) {
            case 0:
                if (gamepad2.dpad_up) {
                    upTimer.reset();
                    transfer = 1;
                }
                break;
            case 1:
                outtake.transferPixels();
                if (upTimer.seconds() > upTime) {
                    secureTimer.reset();
                    transfer = 2;
                }
                break;
            case 2:
                outtake.lockPixels();
                if (secureTimer.seconds() > lockTime) {
                    transfer = 3;
                }
                break;
            case 3:
                outtake.resetBucket();
                transfer = 0;
                break;
        }

        ElapsedTime lockTimer = new ElapsedTime();

        //Lift
        lift.moveLift(gamepad2.left_stick_y - Kg);

            //Pivot
        if (gamepad2.right_bumper) {
            outtake.pivotEnding();
            outtake.resetBucket();
        }//score
        if (gamepad2.left_bumper) {
            outtake.pivotStart();
            outtake.releaseMain();
            outtake.releaseSecondary();
        }

        //Plane
        if (gamepad2.touchpad) plane.launch();
        if (gamepad2.left_stick_button) plane.reset();

        climb.moveClimb(gamepad2.right_stick_y);

       //Drive
        driveTrain.teleopDrive( gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x);
        if(gamepad1.dpad_up){
            driveTrain.TeleOpDriveF(0.5);
        }
        if (gamepad1.dpad_down){
            driveTrain.TeleOpDriveB(0.5);
        }
        if (gamepad1.dpad_right){
            driveTrain.TeleOpStrafeR(0.8);
        }
        if(gamepad1.dpad_left){
            driveTrain.TeleOpStrafeL(0.8);
        }

//            driveTrain.teleopDrive( driveTrain.joystick_conditioning(gamepad1.left_stick_y, 0, .1, .8),
//                    driveTrain.joystick_conditioning(gamepad1.left_stick_x, 0, .1, .8),
//                    driveTrain.joystick_conditioning(gamepad1.right_stick_x, 0, .1, .8));
            if (gamepad2.dpad_up) {
                outtake.transferPixels();
            }
            if (gamepad2.a) {
                outtake.lockPixels();
            }
            if (gamepad2.dpad_down) {
                outtake.resetBucket();
            }
//            int count = 0;
//            if (gamepad2.a && lockTimer.seconds() > 4) {
//                count++;
//                lockTimer.reset();
//            }
            if (gamepad2.left_trigger == (1)) {
                outtake.releaseMain();
            }
            if (gamepad2.right_trigger == (1)) {
                outtake.releaseMain();
                outtake.releaseSecondary();
            }


            //Blinkin

            displayKind = SampleRevBlinkinLedDriver.DisplayKind.AUTO;

            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            blinkinLedDriver.setPattern(pattern);

            display = telemetry.addData("Display Kind: ", displayKind.toString());
            patternName = telemetry.addData("Pattern: ", pattern.toString());

            ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            if (displayKind == SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
                doAutoDisplay();
            } else {
                telemetry.addData("Heading: ", driveTrain.getHeading());
                telemetry.update();
            }


    }

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }


}



