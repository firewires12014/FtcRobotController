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
    private Climb climb;
    private Plane plane;

    double upTime = 0.75; //seconds
    double inTime = 1; //seconds
    double outTime = 0.75; //seconds
    double lockTime = 1; //seconds
    double climbTime = 110; //seconds
    boolean inside =true;
    ElapsedTime upTimer = new ElapsedTime();
    ElapsedTime secureTimer = new ElapsedTime();
    ElapsedTime inTimer = new ElapsedTime();
    ElapsedTime outTimer = new ElapsedTime();
    ElapsedTime climbTimer = new ElapsedTime();
    int lastPos = 3;
    boolean changed = false;
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

//    RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern glow;
//    RevBlinkinLedDriver.BlinkinPattern blink;
    Telemetry.Item patternName;
    Telemetry.Item display;
//    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        plane = new Plane(hardwareMap);
        climb = new Climb(hardwareMap);
        intake.initIntake();
        outtake.intakePosition();
    }

    @Override
    public void loop() {
        //Drive
        driveTrain.teleopDrive( gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        //Slow Drive/Strafe
        if(gamepad1.dpad_up){
            driveTrain.TeleOpDriveB(0.5);
        }
        if (gamepad1.dpad_down){
            driveTrain.TeleOpDriveF(0.5);
        }
        if (gamepad1.dpad_right){
            driveTrain.TeleOpStrafeL(0.8);
        }
        if(gamepad1.dpad_left){
            driveTrain.TeleOpStrafeR(0.8);
        }

        //Intake
        if (gamepad1.right_trigger > 0) {
            intake.in();
        } else if (gamepad1.left_trigger > 0) {
            intake.out();
        } else {
//            intake.die();
        }

        if (gamepad1.right_bumper) {
            intake.inSlow();
        } else if (gamepad1.left_bumper) {
            intake.outSlow();
        }

        //Intake Stack Height
        if (gamepad2.y) {
            intake.stack();
        }
        else {
            intake.score();
        }
//
//        if (gamepad1.y) {
//            intake.stackHeight();
//        }
//        if (gamepad1.right_bumper) {
//            intake.inSlow();
//        }
//        if (gamepad1.left_bumper) {
//            intake.die();
//        }




        //Lift
        lift.moveLift(gamepad2.left_stick_y - Kg);


        //Pivot
        if (gamepad2.right_bumper) { //Scoring Pos.
            //outtake.pivotEnding();
        }


        //Locks
        if (gamepad2.a) { //Lock
            lift.lowerLift();
            outtake.lockPixels();
        }
        if (gamepad2.left_trigger == (1)) {
            outtake.releaseMain();
        }
        if (gamepad2.right_trigger == (1)) {
            outtake.releaseSecondary();
        }

        //Plane
        if (gamepad1.touchpad) plane.launch();
        if (gamepad1.left_stick_button) plane.reset();

        //Climb
        climb.moveClimb(-gamepad2.right_stick_y);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        // state changes
        if ((gamepad2.b || gamepad2.x) && !changed) {
            changed = true;
            if (gamepad2.b)  lastPos++;
            if (gamepad2.x)   lastPos = lastPos - 1;
        }

        if (lastPos > 7) lastPos = 7;
        if (lastPos < 0) lastPos = 0;

        if (!gamepad2.x && !gamepad2.b && !inside){
            changed = false;
        }


        if (gamepad2.right_bumper){
            lastPos = 3;
            inside = false;
           outtake.diffyPosition(lastPos);
        }

        if (gamepad2.left_bumper) {
         outtake.intakePosition();
         inside = true;
            lastPos = 0;
        }
        if ((gamepad2.x || gamepad2.b) && !inside ) {
            outtake.diffyPosition(lastPos);
        }
        if (gamepad2.dpad_up) {
            intake.beltIn();
        }

        telemetry.addData("lastPos", lastPos);
        telemetry.update();





    }

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }


}



